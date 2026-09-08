"""plic 的行为测试台：优先级仲裁与领取/完成的回路。

这一段是目录里最没底的：读一次 claim 要把源号交出去、同时把它标成在途，
软件做完写回同一个地址才放行。中间任何一步错，中断要么丢要么反复触发。
"""
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)

(out / "PlicTb.bsv").write_text('''package PlicTb;

import RegIf::*;
import Plic::*;

// 由 tb/mkplictb.py 生成，勿手改。

// 偏移照 regmap.yaml：优先级每源 4 字节从 4 起，使能每上下文 128 字节，
// 阈值每上下文 4096 字节，领取在阈值之后 4 字节
function Bit#(24) prioAt(Integer s) = fromInteger(4 + 4 * s);
Bit#(24) rEN0    = 24'h002000;
Bit#(24) rTHRESH = 24'h200000;
Bit#(24) rCLAIM  = 24'h200004;

typedef enum { Cfg, Idle, Fire, Claim, Gap1, Recheck, Complete, Gap2,
               Reclaim, Done }
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkPlicTb(Empty);
  PlicIfc#(24, 32, 8, 2) p <- mkPlic(PlicCfg { none: ? });

  Reg#(Phase)    ph  <- mkReg(Cfg);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  Reg#(Bit#(8))  src <- mkReg(0);
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
    let _ <- p.regs.access(RegReq { addr: a, write: True,
                                    wdata: d, wstrb: 4'hF });
  endaction;

  // 源 2 优先级 1、源 5 优先级 4，两个都使能，阈值 0
  rule cfg (ph == Cfg);
    case (s)
      0: wr(prioAt(2), 1);
      1: wr(prioAt(5), 4);
      2: wr(rEN0, 32'h0000_0024);   // 位 2 与位 5
      3: wr(rTHRESH, 0);
      default: ph <= Idle;
    endcase
    s <= s + 1;
  endrule

  rule idle (ph == Idle);
    src <= 8'b0010_0100;            // 源 2 与源 5 同时拉起
    ph  <= Fire;
  endrule

  rule fire (ph == Fire);
    ph <= Claim;
  endrule

  // 优先级高的先被领走：源号从 1 起编，所以该是 6
  rule claim (ph == Claim);
    let x <- p.regs.access(RegReq { addr: rCLAIM, write: False,
                                    wdata: 0, wstrb: 4'hF });
    Bool wrong = False;
    if (x.rdata != 6) begin
      $display("FAIL claim gave %0d, want 6 (source 5, the higher priority)",
               x.rdata);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Gap1;
  endrule

  // 在途标记本拍算完、下一拍才进仲裁，所以隔一拍再领
  rule gap1 (ph == Gap1);
    ph <= Recheck;
  endrule

  // 领走之后同一个源不该再被领第二次，这一次该轮到源 2（编号 3）
  rule recheck (ph == Recheck);
    let x <- p.regs.access(RegReq { addr: rCLAIM, write: False,
                                    wdata: 0, wstrb: 4'hF });
    Bool wrong = False;
    if (x.rdata != 3) begin
      $display("FAIL second claim gave %0d, want 3 (the first one is in flight)",
               x.rdata);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Complete;
  endrule

  rule complete (ph == Complete);
    wr(rCLAIM, 3);                  // 做完源 2
    ph <= Gap2;
  endrule

  Reg#(Bit#(8)) g2 <- mkReg(0);
  rule gap2 (ph == Gap2);
    if (g2 > 8) ph <= Reclaim; else g2 <= g2 + 1;
  endrule

  // 放回去之后又能领到它了
  rule reclaim (ph == Reclaim);
    let x <- p.regs.access(RegReq { addr: rCLAIM, write: False,
                                    wdata: 0, wstrb: 4'hF });
    Bool wrong = False;
    if (x.rdata != 3) begin
      $display("FAIL after completing, claim gave %0d, want 3 again", x.rdata);
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
''', encoding="utf-8")
print("  plic 行为测试台就位")
