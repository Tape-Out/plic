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
  // 每个上下文最近领走的编号。按协议一个上下文同时只有一个在手，
  // 所以软件写回时不必把值取出来——必然是这一个。
  Vector#(contexts, Reg#(Bit#(32))) taken <- replicateM(mkConfigReg(0));

  // 仲裁结果留一份影子。驱动 volatile 的规则必须排在总线方法之前，
  // 读软件脉冲的规则必须排在之后，同一条规则要不到这两件事——
  // 中间用 ConfigReg 转手，读它不带次序要求（D39 那条老经验）。
  Vector#(contexts, Reg#(Bit#(32))) best <- replicateM(mkConfigReg(0));

  Reg#(Bool) rdPend <- mkReg(False);
  Reg#(Bool) wrPend <- mkReg(False);
  Reg#(Bit#(TLog#(TAdd#(contexts, 1)))) rdIdx <- mkReg(0);
  Reg#(Bit#(TLog#(TAdd#(contexts, 1)))) wrIdx <- mkReg(0);

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
    writeVReg(best, b);
  endrule

  rule mark;
    rdPend <= r.claim_rd;
    wrPend <= r.claim_wr;
    rdIdx  <= r.claim_rd_i;
    wrIdx  <= r.claim_wr_i;
  endrule

  // 读走即在途，写回即放行。两件事写同一个寄存器，合在一条规则里定序。
  rule apply (rdPend || wrPend);
    if (rdPend) begin
      let id = best[rdIdx];
      if (id != 0) begin
        inflight <= inflight | (32'h1 << (id - 1));
        taken[rdIdx] <= id;
      end
    end else begin
      let id = taken[wrIdx];
      if (id != 0) inflight <= inflight & ~(32'h1 << (id - 1));
    end
  endrule

  interface regs = r.regs;
  interface PlicPins pins;
    method Action src(Bit#(sources) v); srcIn._write(v); endmethod
  endinterface
  method Bit#(contexts) eip;
    Bit#(contexts) o = 0;
    for (Integer c = 0; c < valueOf(contexts); c = c + 1)
      if (best[c] != 0) o[c] = 1;
    return o;
  endmethod
endmodule

endpackage
