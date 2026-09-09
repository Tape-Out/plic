"""plic 的行为测试台：优先级仲裁与领取/完成的回路。

这一段是目录里最没底的：读一次 claim 要把源号交出去、同时把它标成在途，
软件做完写回同一个地址才放行。中间任何一步错，中断要么丢要么反复触发。

认矩阵：`sources` 决定挑哪两个源来比优先级——原来写死源 2 与源 5，源数少于 6 的
那些点根本没测到自己那一份。只有一个源时比不了优先级，改验「领走之后在途、
放回去又能领」这一条，别拿同一个源冒充两个。`contexts` 决定 eip 的宽度。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
nsrc = int(k.get("sources", 32))
nctx = int(k.get("contexts", 2))

NL = chr(10)
lo = 0
hi = min(nsrc - 1, 5)
pair = nsrc >= 2
mask = (1 << lo) | (1 << hi) if pair else 1
# 领取交出去的是从 1 起编的源号
first = (hi + 1) if pair else 1
second = (lo + 1) if pair else 0
why1 = (f"source {hi}, the higher priority" if pair
        else "the only source there is")
why2 = ("the first one is in flight" if pair
        else "the only source is in flight, so nothing is left")
prio = NL.join([
    f"      0: wr(prioAt({lo}), 1);",
] + ([f"      1: wr(prioAt({hi}), 4);"] if pair else ["      1: noAction;"]))

txt = f'''package Plic{label}Tb;

import RegIf::*;
import Plic::*;

// 由 tb/mkplictb.py 生成，勿手改。
// 这一点：sources={nsrc} contexts={nctx}，比的是源 {lo} 与源 {hi}

// 偏移照 regmap.yaml：优先级每源 4 字节从 4 起，使能每上下文 128 字节，
// 阈值每上下文 4096 字节，领取在阈值之后 4 字节
function Bit#(24) prioAt(Integer s) = fromInteger(4 + 4 * s);
Bit#(24) rEN0    = 24'h002000;
Bit#(24) rTHRESH = 24'h200000;
Bit#(24) rCLAIM  = 24'h200004;

typedef enum {{ Cfg, Idle, Fire, Claim, Gap1, Recheck, Complete, Gap2,
               Reclaim, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkPlic{label}Tb(Empty);
  PlicIfc#(24, 32, {nsrc}, {nctx}) p <- mkPlic(PlicCfg {{ none: ? }});

  Reg#(Phase)    ph  <- mkReg(Cfg);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  Reg#(Bit#({nsrc})) src <- mkReg(0);
  Reg#(Bool)     sawEip <- mkReg(False);

  rule pins;
    p.pins.src(src);
    if (p.eip[0] == 1) sawEip <= True;
  endrule

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 20000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(24) a, Bit#(32) d) = action
    let _ <- p.regs.access(RegReq {{ addr: a, write: True,
                                    wdata: d, wstrb: 4'hF }});
  endaction;

  rule cfg (ph == Cfg);
    case (s)
{prio}
      2: wr(rEN0, 32'h{mask:08X});
      3: wr(rTHRESH, 0);
      default: ph <= Idle;
    endcase
    s <= s + 1;
  endrule

  rule idle (ph == Idle);
    src <= {nsrc}'h{mask:X};
    ph  <= Fire;
  endrule

  rule fire (ph == Fire);
    ph <= Claim;
  endrule

  // 优先级高的先被领走：源号从 1 起编
  rule claim (ph == Claim);
    let x <- p.regs.access(RegReq {{ addr: rCLAIM, write: False,
                                    wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (x.rdata != {first}) begin
      $display("FAIL claim gave %0d, want {first} ({why1})", x.rdata);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Gap1;
  endrule

  // 在途标记本拍算完、下一拍才进仲裁，所以隔一拍再领
  rule gap1 (ph == Gap1);
    ph <= Recheck;
  endrule

  // 领走之后同一个源不该再被领第二次
  rule recheck (ph == Recheck);
    let x <- p.regs.access(RegReq {{ addr: rCLAIM, write: False,
                                    wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (x.rdata != {second}) begin
      $display("FAIL second claim gave %0d, want {second} ({why2})", x.rdata);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Complete;
  endrule

  rule complete (ph == Complete);
    wr(rCLAIM, {first});             // 把先领走的那个做完
    ph <= Gap2;
  endrule

  Reg#(Bit#(8)) g2 <- mkReg(0);
  rule gap2 (ph == Gap2);
    if (g2 > 8) ph <= Reclaim; else g2 <= g2 + 1;
  endrule

  // 放回去之后又能领到它了
  rule reclaim (ph == Reclaim);
    let x <- p.regs.access(RegReq {{ addr: rCLAIM, write: False,
                                    wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (x.rdata != {first}) begin
      $display("FAIL after completing, claim gave %0d, want {first} again", x.rdata);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Done;
  endrule

  rule fin (ph == Done);
    if (!sawEip) begin
      $display("FAIL eip never went high");
      bad <= True;
    end
    if (bad || !sawEip) $display("FAILED");
    else $display("PASS plic: priority, claim, in flight masking, complete");
    $finish((bad || !sawEip) ? 1 : 0);
  endrule
endmodule

endpackage
'''
(out / f"Plic{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  plic 行为测试台就位：sources={nsrc} contexts={nctx}")
