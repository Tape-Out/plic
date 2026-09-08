`ifndef PLIC_MMIO_V
`define PLIC_MMIO_V

`timescale 1ns/1ps

`ifndef PLIC_MAX_SOURCES
`define PLIC_MAX_SOURCES  32
`endif

`ifndef PLIC_MAX_CONTEXTS
`define PLIC_MAX_CONTEXTS 8
`endif

module plic_mmio #(
    parameter [31:0]  BASE_ADDR     = 32'h0C00_0000,
    parameter integer MAX_SOURCES   = `PLIC_MAX_SOURCES,  // total num of irq contain ext and int
    parameter integer MAX_CONTEXTS  = `PLIC_MAX_CONTEXTS, // total num of ctx (numOfCore x numOfMode), eg. core1: m-mode:x1ctx, s-mode:x1ctx ...
    parameter integer PRIO_BITS     = 3                   // level of priority allowed, 3 is 2**3 -> [0,7]
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    input  wire [31:0]              mem_wdata,
    input  wire [3 :0]              mem_wstrb,
    output reg  [31:0]              mem_rdata,

    input  wire [MAX_SOURCES -1:0]   irq_sources,

    output reg  [MAX_CONTEXTS-1:0]  irq_pending,
    input  wire [MAX_CONTEXTS-1:0]  irq_claim,
    input  wire [MAX_CONTEXTS-1:0]  irq_complete
);

    reg [PRIO_BITS-1:0] source_priority [0:MAX_SOURCES-1];  // 0=disable irq, 1=lowest priority with irq
    reg [MAX_SOURCES-1:0] source_enable [0:MAX_CONTEXTS-1]; // [which irq][enable which bit irq] irq bitmap
    reg [PRIO_BITS-1:0] threshold [0:MAX_CONTEXTS-1];       // threshold of each irq
    reg [MAX_SOURCES-1:0] pending;

    reg [MAX_SOURCES-1:0] claimed [0:MAX_CONTEXTS-1];
    reg [$clog2(MAX_SOURCES)-1:0] claimed_id [0:MAX_CONTEXTS-1];

    localparam [31:0]
        RW_PLIC_PRIORITY_BASE  = BASE_ADDR + 32'h0000_0000,    // priority area, addr = BASE + 4 × source_id
        RO_PLIC_PENDING_BASE   = BASE_ADDR + 32'h0000_1000,    // pending bitmap area
        RW_PLIC_ENABLE_BASE    = BASE_ADDR + 32'h0000_2000,    // enable bitmap of each ctx
        RW_PLIC_THRESHOLD_BASE = BASE_ADDR + 32'h0020_0000,    // threshold of each ctx
        RW_PLIC_CLAIM_BASE     = BASE_ADDR + 32'h0020_0004;    // claim/complete, read as claim, write as complete

    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    function [31:0] zext32;
        input [MAX_SOURCES-1:0] in;
        integer i;
        begin
            zext32 = 32'b0;
            for (i = 0; i < 32 && i < MAX_SOURCES; i = i + 1)
                zext32[i] = in[i];
        end
    endfunction

    function [31:0] get_priority;
        input [31:0] index;
        begin
            if (index < MAX_SOURCES)
                get_priority = {29'b0, source_priority[index]};
            else
                get_priority = 32'b0;
        end
    endfunction

    function [31:0] get_enable;
        input [31:0] context;
        input [31:0] word_index;
        reg [31:0] enable_bits;
        integer i;
        begin
            enable_bits = 32'b0;
            for (i = 0; i < 32; i = i + 1) begin
                if ((word_index * 32 + i) < MAX_SOURCES && context < MAX_CONTEXTS)
                    enable_bits[i] = source_enable[context][word_index * 32 + i];
            end
            get_enable = enable_bits;
        end
    endfunction

    always @(*) begin: INTERRUPT_ARBITRATION
        integer context, source;
        reg [PRIO_BITS-1:0] max_prio;
        reg [5:0] max_prio_id;

        for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
            irq_pending[context] = 1'b0;
            max_prio = 0;
            max_prio_id = 0;

            for (source = 1; source < MAX_SOURCES; source = source + 1) begin
                if (pending[source] && source_enable[context][source] &&
                    source_priority[source] > threshold[context] &&
                    source_priority[source] > max_prio) begin
                    max_prio = source_priority[source];
                    max_prio_id = source;
                    irq_pending[context] = 1'b1;
                end
            end

            if (irq_pending[context])
                claimed_id[context] = max_prio_id;
        end
    end

    always @(posedge clk) begin: CLAIM_HANDLING
        integer context;

        if (!resetn) begin
            for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
                claimed[context] <= 0;
            end
        end else begin
            for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
                if (irq_claim[context] && irq_pending[context]) begin
                    claimed[context][claimed_id[context]] <= 1'b1;
                    pending[claimed_id[context]] <= 1'b0;
                end
            end
        end
    end

    always @(posedge clk) begin: COMPLETION_HANDLING
        integer context, source;
        if (!resetn) begin
        end else begin
            for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
                if (irq_complete[context]) begin
                    for (source = 0; source < MAX_SOURCES; source = source + 1) begin
                        if (claimed[context][source]) begin
                            claimed[context][source] <= 1'b0;
                        end
                    end
                end
            end
        end
    end

    always @(posedge clk) begin: PENDING_UPDATE
        integer source, context;
        reg is_claimed, is_completed;

        if (!resetn) begin
            pending <= 0;
        end else begin
            for (source = 0; source < MAX_SOURCES; source = source + 1) begin
                is_claimed = 1'b0;
                for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
                    if (claimed[context][source]) begin
                        is_claimed = 1'b1;
                    end
                end

                is_completed = 1'b0;
                for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
                    if (irq_complete[context] && claimed[context][source]) begin
                        is_completed = 1'b1;
                    end
                end

                if (is_completed) begin
                    pending[source] <= 1'b0;
                end else if (irq_sources[source] && !is_claimed) begin
                    pending[source] <= 1'b1;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            mem_ready <= 0;
        end else mem_ready <= mem_valid && !mem_instr;
    end

    always @(posedge clk) begin: MMIO_READ
        integer context, word_index;

        if (!resetn) begin
            mem_rdata <= 0;
        end else begin
            if (mem_valid && !mem_instr && mem_wstrb == 0) begin
                if (mem_addr >= RW_PLIC_PRIORITY_BASE && mem_addr < RO_PLIC_PENDING_BASE) begin
                    word_index = (mem_addr - RW_PLIC_PRIORITY_BASE) >> 2;
                    mem_rdata <= get_priority(word_index);
                end else if (mem_addr >= RO_PLIC_PENDING_BASE && mem_addr < RW_PLIC_ENABLE_BASE) begin
                    word_index = (mem_addr - RO_PLIC_PENDING_BASE) >> 2;
                    if (word_index == 0)
                        mem_rdata <= zext32(pending);
                    else
                        mem_rdata <= 0;
                end else if (mem_addr >= RW_PLIC_ENABLE_BASE && mem_addr < RW_PLIC_THRESHOLD_BASE) begin
                    context = (mem_addr - RW_PLIC_ENABLE_BASE) >> 12;
                    word_index = ((mem_addr - RW_PLIC_ENABLE_BASE) & 32'hFFF) >> 2;
                    if (context < MAX_CONTEXTS)
                        mem_rdata <= get_enable(context, word_index);
                    else
                        mem_rdata <= 0;
                end else if (mem_addr >= RW_PLIC_THRESHOLD_BASE) begin
                    context = (mem_addr - RW_PLIC_THRESHOLD_BASE) >> 12;

                    if (context < MAX_CONTEXTS) begin
                        case (mem_addr & 32'hFFF)
                            32'h000: mem_rdata <= {29'b0, threshold[context]};
                            32'h004: begin
                                if (irq_pending[context])
                                    mem_rdata <= {26'b0, claimed_id[context]};
                                else
                                    mem_rdata <= 0;
                            end
                            default: mem_rdata <= 0;
                        endcase
                    end else begin
                        mem_rdata <= 0;
                    end
                end else begin
                    mem_rdata <= 0;
                end
            end else begin
                mem_rdata <= 0;
            end
        end
    end

    always @(posedge clk) begin: MMIO_WRITE
        integer context, word_index, source;
        if (!resetn) begin
            for (source = 0; source < MAX_SOURCES; source = source + 1) begin
                source_priority[source] <= 0;
            end
            for (context = 0; context < MAX_CONTEXTS; context = context + 1) begin
                threshold[context] <= 0;
                for (source = 0; source < MAX_SOURCES; source = source + 1) begin
                    source_enable[context][source] <= 0;
                end
            end
        end else begin
            addr = mem_addr;

            if (mem_valid && !mem_instr && mem_wstrb != 0) begin
                if (addr >= RW_PLIC_PRIORITY_BASE && mem_addr < RO_PLIC_PENDING_BASE) begin
                    word_index = (mem_addr - RW_PLIC_PRIORITY_BASE) >> 2;
                    if (word_index < MAX_SOURCES)
                        source_priority[word_index] <= wdata[PRIO_BITS-1:0];
                end else if (mem_addr >= RW_PLIC_ENABLE_BASE && mem_addr < RW_PLIC_THRESHOLD_BASE) begin
                    context = (mem_addr - RW_PLIC_ENABLE_BASE) >> 12;
                    word_index = ((mem_addr - RW_PLIC_ENABLE_BASE) & 32'hFFF) >> 2;

                    if (context < MAX_CONTEXTS) begin
                        for (source = 0; source < 32; source = source + 1) begin
                            if ((word_index * 32 + source) < MAX_SOURCES)
                                source_enable[context][word_index * 32 + source] <= wdata[source];
                        end
                    end

                end else if (mem_addr >= RW_PLIC_THRESHOLD_BASE) begin
                    context = (mem_addr - RW_PLIC_THRESHOLD_BASE) >> 12;

                    if (context < MAX_CONTEXTS) begin
                        case (mem_addr & 32'hFFF)
                            32'h000: threshold[context] <= wdata[PRIO_BITS-1:0];
                            32'h004: begin
                                if (wdata < MAX_SOURCES && claimed[context][wdata])
                                    claimed[context][wdata] <= 1'b0;
                            end
                            default: ;
                        endcase
                    end
                end
            end
        end
    end

endmodule

`endif
