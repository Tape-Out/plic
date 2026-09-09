package Plic;

import Vector::*;
import ConfigReg::*;
import RegIf::*;
import PlicRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
// 本版只做 PLIC 本体：AIA 的 APLIC 与 IMSIC 是另一套寄存器布局，住在 imsic 仓。
typedef struct {
  Bit#(0) none;
} PlicCfg;

interface PlicPins#(numeric type sources);
  (* always_ready, always_enabled, prefix = "" *)
  method Action src((* port = "irq_in" *) Bit#(sources) v);
endinterface

interface PlicIfc#(numeric type aw, numeric type dw,
                   numeric type sources, numeric type contexts);
  interface RegIf#(aw, dw) regs;
  interface PlicPins#(sources) pins;
  (* always_ready *) method Bit#(contexts) eip;
endinterface

module mkPlic#(PlicCfg cfg)(PlicIfc#(aw, dw, sources, contexts))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 24, aw), Add#(_b, 3, dw),
              Add#(_c, 32, dw), Add#(_d, sources, 32),
              Add#(_e, TLog#(TAdd#(contexts, 1)), 24));

  PlicRegsIfc#(aw, dw, sources, contexts) r <- mkPlicRegs;

  Wire#(Bit#(sources)) srcIn <- mkBypassWire;
  // 领走还没做完的源要屏蔽掉，否则同一个中断会被反复领
  Reg#(Bit#(32)) inflight <- mkConfigReg(0);

  // 仲裁结果留一份同拍可读的影子。驱动 volatile 的规则要排在总线方法之前，
  // 读软件脉冲的规则要排在之后——用 DWire 转手，次序就是
  // 「publish -> 总线方法 -> apply」，一条直线，没有环。
  //
  // 用 ConfigReg 转手也不成环，但那样在途标记要两拍才生效，中间那一拍
  // 同一个源会被领第二次。DWire 把窗口压到一拍：本拍发出去的号，
  // 下一拍的仲裁就看不见了。
  Vector#(contexts, Wire#(Bit#(32))) best <- replicateM(mkDWire(0));
  // 中断线打一拍。直接从 best 那根线上出的话，凡是「既驱动 src 又读 eip」的
  // 规则都会与 publish 成环，publish 被丢掉——而组合直通 src 到中断输出
  // 对时序也不是好事。
  Vector#(contexts, Reg#(Bool)) eipR <- replicateM(mkReg(False));

  function Vector#(contexts, Bit#(32)) arbitrate(Bit#(32) pend);
    Vector#(contexts, Bit#(32)) o = newVector;
    for (Integer c = 0; c < valueOf(contexts); c = c + 1) begin
      Bit#(32) win = 0;
      Bit#(3)  top = r.thresh[c];
      for (Integer s = 0; s < valueOf(sources); s = s + 1)
        if (pend[s] == 1 && r.enable[c][s] == 1 && r.prio[s] > top) begin
          top = r.prio[s];
          win = fromInteger(s + 1);   // PLIC 的 0 号不是源，编号从 1 起
        end
      o[c] = win;
    end
    return o;
  endfunction

  rule publish;
    Bit#(32) pend = zeroExtend(srcIn) & ~inflight;
    let b = arbitrate(pend);
    r.pending_in(pend);
    r.claim_in(b);
    for (Integer c = 0; c < valueOf(contexts); c = c + 1) begin
      best[c] <= b[c];
      eipR[c] <= b[c] != 0;
    end
  endrule

  // 读走即在途，写回即放行。两件事写同一个寄存器，合在一条规则里定序。
  //
  // 放行的是**软件写回来的那个源号**，不是「这个上下文最近领走的那个」。
  // 规范允许一个上下文在做完之前再领一次（高优先级抢占就是这么用的），
  // 那时手上同时有两个，认「最近」就会把还没做完的那个放回去。
  // 认不出来的号一律忽略，规范也是这么写的。
  rule apply (r.claim_rd || r.claim_wr);
    if (r.claim_rd) begin
      let id = best[r.claim_rd_i];
      if (id != 0) inflight <= inflight | (32'h1 << (id - 1));
    end else begin
      Bit#(32) id = r.claim_wr_val;
      // 还要求这个源对写回的那个上下文是使能的。规范说得明白：认不出来的
      // 完成一律静默忽略，免得一个上下文替别人把中断放回去。
      Bool ok = id != 0 && id <= fromInteger(valueOf(sources))
                && inflight[id - 1] == 1
                && r.enable[r.claim_wr_i][id - 1] == 1;
      if (ok) inflight <= inflight & ~(32'h1 << (id - 1));
    end
  endrule

  interface regs = r.regs;
  interface PlicPins pins;
    method Action src(Bit#(sources) v); srcIn._write(v); endmethod
  endinterface
  method Bit#(contexts) eip;
    Bit#(contexts) o = 0;
    for (Integer c = 0; c < valueOf(contexts); c = c + 1)
      if (eipR[c]) o[c] = 1;
    return o;
  endmethod
endmodule

endpackage
