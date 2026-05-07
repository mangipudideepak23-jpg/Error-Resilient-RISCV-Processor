`timescale 1ns/1ps

// ============================================================
// Focused testbench: BCH(45,32,t=2) — 2-bit error correction.
//
// Flips bits at positions FAULT_BIT_A and FAULT_BIT_B.
// Key proof signals:
//   S1  != 0  => error detected
//   s2  != 0  => TWO errors (distinguishes from single-bit)
//   error_mask has exactly 2 bits set at the fault positions
//   data_out  == original instruction (both bits corrected)
//   fatal      = 0
//
// Pipeline depth = 3 clock cycles.
// ============================================================

module tb_2bit_correction;

    // ── DUT ports ────────────────────────────────────────────
    reg         clk, rst, valid_in;
    reg  [44:0] data_in;
    wire        valid_out;
    wire        fatal;
    wire [31:0] data_out;
    wire [44:0] error_mask;
    wire [ 5:0] sigma2;

    // ── Instruction and fault positions ──────────────────────
    localparam [31:0] INSTR      = 32'h00208033; // ADD x1,x1,x2
    localparam integer FAULT_BIT_A = 0;           // LSB of codeword (parity)
    localparam integer FAULT_BIT_B = 22;          // data region bit

    // ── Encoder ──────────────────────────────────────────────
    wire [44:0] clean_cw;
    bch_encoder_final enc_i (.data_in(INSTR), .codeword(clean_cw));

    // Flip both fault bits
    wire [44:0] corrupt_cw = clean_cw ^ (45'd1 << FAULT_BIT_A)
                                       ^ (45'd1 << FAULT_BIT_B);

    // ── DUT ──────────────────────────────────────────────────
    bch_decoder_prod uut (
        .clk      (clk),
        .rst      (rst),
        .valid_in (valid_in),
        .data_in  (data_in),
        .valid_out(valid_out),
        .data_out (data_out),
        .fatal    (fatal),
        .error_mask(error_mask),
        .sigma2   (sigma2)
    );

    // ── Clock: 10 ns period ───────────────────────────────────
    initial clk = 0;
    always  #5  clk = ~clk;

    // ── VCD: dump everything ─────────────────────────────────
    initial begin
        $dumpfile("tb_2bit_correction.vcd");
        $dumpvars(0, tb_2bit_correction);
    end

    // ── Main stimulus ─────────────────────────────────────────
    initial begin
        rst = 1; valid_in = 0; data_in = 45'd0;

        $display("============================================================");
        $display("  BCH(45,32,t=2)  Decode-Stage 2-Bit Error Correction Demo  ");
        $display("============================================================");
        $display("  Instruction : ADD x1,x1,x2  (0x%08X)", INSTR);
        $display("  Fault bits  : position %0d AND position %0d", FAULT_BIT_A, FAULT_BIT_B);
        $display("  clean_cw    : 0x%012X", clean_cw);
        $display("  corrupt_cw  : 0x%012X  (bits %0d and %0d flipped)", corrupt_cw, FAULT_BIT_A, FAULT_BIT_B);
        $display("");

        // Hold reset for 4 cycles
        repeat(4) @(posedge clk); #1;
        rst = 0;

        // ── TEST: 2-bit error ─────────────────────────────────
        $display("-- Presenting 2-bit corrupted codeword --");

        @(posedge clk); #1;
        valid_in = 1;
        data_in  = corrupt_cw;
        $display("  Cycle +0 : valid_in=1, data_in=0x%012X", corrupt_cw);

        @(posedge clk); #1;     // Stage 1 — syndrome computed
        valid_in = 0;
        data_in  = 45'd0;
        $display("  Cycle +1 : Stage-1 done | S1=0x%02X S3=0x%02X (non-zero => error detected)", uut.S1, uut.S3);

        @(posedge clk); #1;     // Stage 2 — sigma2 computed
        $display("  Cycle +2 : Stage-2 done | s2=0x%02X (NON-ZERO => TWO-bit error)", uut.s2);

        @(posedge clk); #1;     // Stage 3 — Chien search + output
        $display("  Cycle +3 : OUTPUT:");
        $display("             valid_out  = %b",         valid_out);
        $display("             data_out   = 0x%08X",      data_out);
        $display("             error_mask = 0x%012X",     error_mask);
        $display("             fatal      = %b",          fatal);
        $display("             sigma2     = 0x%02X",      sigma2);
        $display("             bit %0d in mask = %b",     FAULT_BIT_A, error_mask[FAULT_BIT_A]);
        $display("             bit %0d in mask = %b",     FAULT_BIT_B, error_mask[FAULT_BIT_B]);

        if (valid_out && !fatal && data_out == INSTR
            && error_mask[FAULT_BIT_A] && error_mask[FAULT_BIT_B])
            $display("\n>>> RESULT: PASS -- both bit %0d and bit %0d corrected. Original instruction recovered.",
                     FAULT_BIT_A, FAULT_BIT_B);
        else
            $display("\n>>> RESULT: FAIL -- unexpected result.");

        // Idle tail
        repeat(5) @(posedge clk); #1;

        $display("\nWaveform: tb_2bit_correction.vcd");
        $finish;
    end

endmodule
