// =============================================================
// picorv32_ecc_top.v — Combined BCH Error-Resilient RISC-V CPU
//
// Integrates TWO independent BCH(45,32,t=2) protection layers:
//
//  1. FETCH-STAGE PROTECTION (friend's work — AXI_MEM project)
//     Module: ecc_instr_memory
//     All instruction fetches pass through a 3-cycle BCH(45,32)
//     decode pipeline.  The memory stores 45-bit BCH codewords;
//     faults are corrected before the CPU ever sees the opcode.
//     Data reads/writes bypass BCH (raw memory, 1-cycle latency).
//
//  2. REGISTER-FILE PROTECTION (Deepak's work — decode/writeback stage)
//     Module: bch_regfile  (via PICORV32_REGS define)
//     All 32 RISC-V registers (x0–x31) are stored as 45-bit BCH
//     codewords.  Writes encode; reads decode combinationally in
//     1 cycle — no extra pipeline stall for the register file.
//
// Together these two layers protect the CPU from soft errors at
// both the instruction fetch path and the architectural register
// state, covering the two most critical storage structures.
//
// Compile with (see Makefile for the full target):
//   iverilog -g2012 \
//            src/rtl/picorv32.v src/rtl/bch_ecc.v \
//            src/rtl/bch_regfile.v src/rtl/ecc_instr_memory.v \
//            src/rtl/picorv32_ecc_top.v \
//            src/testbench/tb_combined_ecc.v
//
// Note: picorv32.v already defines `PICORV32_REGS bch_regfile internally,
// so no -D flag is needed.  The register-file override is transparent.
//
// Memory map:
//   0x0000_0000 .. MEM_SIZE-1  — BCH-protected instr + raw data memory
//   0x0200_0004                — UART clock-divider register (write)
//   0x0200_0008                — UART data register (write, ASCII char)
//   0x0300_0000                — Test-status register (firmware writes on done)
//   0x0300_0004                — Test-signature register
// =============================================================

`timescale 1ns/1ps

module picorv32_ecc_top #(
    parameter integer MEM_SIZE      = 128*1024,     // bytes
    parameter [31:0]  PROGADDR_RESET = 32'h0000_0000,
    parameter [31:0]  PROGADDR_IRQ   = 32'h0000_0010
) (
    input  clk,
    input  resetn,
    output trap,

    // ---- Fetch-stage (instruction memory) BCH status ----
    output [31:0] fetch_single_err_cnt,   // corrected single-bit errors
    output [31:0] fetch_double_err_cnt,   // corrected double-bit errors
    output [31:0] fetch_fatal_err_cnt,    // uncorrectable fetch errors
    output        fetch_bch_fatal,        // current-cycle fatal flag

    // ---- Fetch-stage fault injection (testbench use) ----
    input         fi_fetch_en,            // enable fault injection
    input         fi_fetch_double,        // second bit-flip
    input  [31:0] fi_fetch_word_idx,      // instruction word to corrupt
    input  [ 5:0] fi_fetch_pos1,          // first bit position (0..44)
    input  [ 5:0] fi_fetch_pos2,          // second bit position (0..44)

    // ---- Test completion (written by firmware at hello.c end) ----
    output reg        test_done,
    output reg        test_passed,
    output reg [31:0] test_status,
    output reg [31:0] test_signature
    // NOTE: register-file ECC status is accessed via hierarchical
    //       reference in the testbench:
    //   dut.cpu_core.rf.reg_file_single_err
    //   dut.cpu_core.rf.reg_file_double_err
    //   dut.cpu_core.rf.reg_file_fatal
    //   dut.cpu_core.rf.inject_reg_fault  (force for fault injection)
    //   dut.cpu_core.rf.fault_reg_addr    (force)
    //   dut.cpu_core.rf.fault_reg_bit1    (force)
    //   dut.cpu_core.rf.fault_reg_bit2    (force)
    //   dut.cpu_core.rf.inject_reg_double (force)
    //   dut.cpu_core.rf.regfile_bch[n]    (force individual codeword bits)
);

    // =========================================================
    // CPU memory interface wires
    // =========================================================
    wire        cpu_mem_valid;
    wire        cpu_mem_instr;
    wire        cpu_mem_ready;
    wire [31:0] cpu_mem_addr;
    wire [31:0] cpu_mem_wdata;
    wire [ 3:0] cpu_mem_wstrb;
    wire [31:0] cpu_mem_rdata;

    // Look-ahead (tied off — not used by our memory)
    wire        mem_la_read, mem_la_write;
    wire [31:0] mem_la_addr;

    // =========================================================
    // Address decode: memory vs peripherals
    //   Peripheral space: addr >= 0x0200_0000
    // =========================================================
    localparam [31:0] UART_CLKDIV_ADDR   = 32'h0200_0004;
    localparam [31:0] UART_DATA_ADDR     = 32'h0200_0008;
    localparam [31:0] TEST_STATUS_ADDR   = 32'h0300_0000;
    localparam [31:0] TEST_SIG_ADDR      = 32'h0300_0004;
    localparam [31:0] TEST_STATUS_PASS   = 32'h600d_0001;

    wire is_peripheral = (cpu_mem_addr >= 32'h0200_0000);

    // Gate mem_valid to the memory sub-module
    wire mem_valid_mem  = cpu_mem_valid && !is_peripheral;

    // Ready and rdata mux
    wire        mem_ready_mem;
    wire [31:0] mem_rdata_mem;

    reg         mem_ready_peri;
    reg  [31:0] mem_rdata_peri;
    reg  [ 7:0] uart_char;

    assign cpu_mem_ready = is_peripheral ? mem_ready_peri : mem_ready_mem;
    assign cpu_mem_rdata = is_peripheral ? mem_rdata_peri : mem_rdata_mem;

    // =========================================================
    // PicoRV32 CPU core
    //
    // PICORV32_REGS=bch_regfile is passed at compile time via
    // -D flag, so picorv32.v instantiates bch_regfile for the
    // register file — giving BCH protection on all 32 registers.
    // =========================================================
    picorv32 #(
        .COMPRESSED_ISA (1),
        .ENABLE_MUL     (1),
        .ENABLE_DIV     (1),
        .ENABLE_COUNTERS(1),
        .ENABLE_COUNTERS64(1),
        .ENABLE_REGS_16_31(1),
        .ENABLE_REGS_DUALPORT(1),
        .REGS_INIT_ZERO (1),
        .PROGADDR_RESET (PROGADDR_RESET),
        .PROGADDR_IRQ   (PROGADDR_IRQ),
        .STACKADDR      (32'h0002_0000)   // top of 128 KB
    ) cpu_core (
        .clk            (clk),
        .resetn         (resetn),
        .trap           (trap),

        .mem_valid      (cpu_mem_valid),
        .mem_instr      (cpu_mem_instr),
        .mem_ready      (cpu_mem_ready),
        .mem_addr       (cpu_mem_addr),
        .mem_wdata      (cpu_mem_wdata),
        .mem_wstrb      (cpu_mem_wstrb),
        .mem_rdata      (cpu_mem_rdata),

        .mem_la_read    (mem_la_read),
        .mem_la_write   (mem_la_write),
        .mem_la_addr    (mem_la_addr),
        .mem_la_wdata   (),
        .mem_la_wstrb   (),

        // PCPI (disabled)
        .pcpi_wr        (1'b0),
        .pcpi_rd        (32'd0),
        .pcpi_wait      (1'b0),
        .pcpi_ready     (1'b0),

        // IRQ (disabled in basic config)
        .irq            (32'd0),
        .eoi            (),

        // Trace (disabled)
        .trace_valid    (),
        .trace_data     (),

        // Error-injection ports from uart_changes branch (tie to 0)
        .in_err         (49'd0),
        .in_err1        (12'd0),
        .in_err2        (38'd0)
    );

    // =========================================================
    // Fetch-stage BCH protection — ecc_instr_memory
    // (friend's work from AXI_MEM project)
    //
    // Handles all memory accesses in 0x0000_0000..MEM_SIZE-1:
    //   - Instruction fetch: BCH decode (3-cycle pipeline stall)
    //   - Data read/write:   raw memory (1-cycle latency)
    // =========================================================
    ecc_instr_memory #(
        .MEM_SIZE (MEM_SIZE)
    ) fetch_ecc_mem (
        .clk            (clk),
        .resetn         (resetn),

        .mem_valid      (mem_valid_mem),
        .mem_instr      (cpu_mem_instr),
        .mem_ready      (mem_ready_mem),
        .mem_addr       (cpu_mem_addr),
        .mem_wdata      (cpu_mem_wdata),
        .mem_wstrb      (cpu_mem_wstrb),
        .mem_rdata      (mem_rdata_mem),

        .bch_fatal      (fetch_bch_fatal),

        // Fault injection from testbench
        .fi_en          (fi_fetch_en),
        .fi_double      (fi_fetch_double),
        .fi_word_idx    (fi_fetch_word_idx),
        .fi_pos1        (fi_fetch_pos1),
        .fi_pos2        (fi_fetch_pos2),

        // Error counters
        .single_err_cnt (fetch_single_err_cnt),
        .double_err_cnt (fetch_double_err_cnt),
        .fatal_err_cnt  (fetch_fatal_err_cnt)
    );

    // =========================================================
    // Peripheral handler (1-cycle latency)
    //   - UART char capture (firmware uses this for $display output)
    //   - Test status / signature registers (firmware writes to signal done)
    // =========================================================
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            mem_ready_peri  <= 1'b0;
            mem_rdata_peri  <= 32'd0;
            uart_char       <= 8'd0;
            test_done       <= 1'b0;
            test_passed     <= 1'b0;
            test_status     <= 32'd0;
            test_signature  <= 32'd0;
        end else begin
            mem_ready_peri <= 1'b0;

            if (cpu_mem_valid && is_peripheral) begin
                mem_rdata_peri <= 32'd0;    // default read value

                if (|cpu_mem_wstrb) begin
                    // Write to peripheral register
                    case (cpu_mem_addr)
                        TEST_STATUS_ADDR: begin
                            test_status  <= cpu_mem_wdata;
                            test_done    <= (cpu_mem_wdata != 32'd0);
                            test_passed  <= (cpu_mem_wdata == TEST_STATUS_PASS);
                        end
                        TEST_SIG_ADDR: begin
                            test_signature <= cpu_mem_wdata;
                        end
                        UART_DATA_ADDR: begin
                            uart_char <= cpu_mem_wdata[7:0];
                            $write("%c", cpu_mem_wdata[7:0]);
                        end
                        default: begin end
                    endcase
                end else begin
                    // Read from peripheral register
                    case (cpu_mem_addr)
                        TEST_STATUS_ADDR: mem_rdata_peri <= test_status;
                        TEST_SIG_ADDR:    mem_rdata_peri <= test_signature;
                        default:          mem_rdata_peri <= 32'd0;
                    endcase
                end

                mem_ready_peri <= 1'b1;
            end
        end
    end

endmodule
