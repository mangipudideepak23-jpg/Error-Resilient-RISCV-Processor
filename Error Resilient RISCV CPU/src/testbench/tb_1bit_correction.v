`timescale 1ns/1ps

// ============================================================
// Focused testbench: BCH(45,32,t=2) decode-stage 1-bit error
// correction proof.
//
// Shows ONE clean run followed by ONE 1-bit error run on
// ADD x1,x1,x2 (0x00208033), fault injected at bit 22.
//
// Pipeline depth = 3 clock cycles:
//   Cycle 1 — Syndrome computation  (S1, S3)
//   Cycle 2 — Error locator poly    (sigma2 / s2)
//   Cycle 3 — Chien search + output (error_mask, data_out)
// ============================================================

module tb_1bit_correction;

    // ── DUT ports ────────────────────────────────────────────
    reg         clk, rst, valid_in;
    reg  [44:0] data_in;
    wire        valid_out;
    wire        fatal;
    wire [31:0] data_out;
    wire [44:0] error_mask;
    wire [ 5:0] sigma2;

    // ── Test parameters ──────────────────────────────────────
    localparam [31:0] INSTR     = 32'h00208033; // ADD x1,x1,x2
    localparam integer FAULT_BIT = 22;

    // ── Encoder: derive clean codeword at elaboration time ───
    wire [44:0] clean_cw;
    bch_encoder_final enc_i (.data_in(INSTR), .codeword(clean_cw));

    // Flip exactly one bit to create the corrupted codeword
    wire [44:0] corrupt_cw = clean_cw ^ (45'd1 << FAULT_BIT);

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

    // ── VCD: dump testbench + all internals of uut ───────────
    initial begin
        $dumpfile("tb_1bit_correction.vcd");
        $dumpvars(0, tb_1bit_correction);
    end

    // ── Display helpers ───────────────────────────────────────
    task show_output;
        input [255:0] label;
        begin
            $display("  [%0s] valid_out=%b  data_out=0x%08X  fatal=%b  sigma2=0x%02X  error_mask=0x%012X",
                     label, valid_out, data_out, fatal, sigma2, error_mask);
            if (!fatal && data_out == INSTR)
                $display("            => Data RECOVERED correctly (matches original instruction)");
            else if (fatal)
                $display("            => FATAL flag set (uncorrectable)");
            else
                $display("            => data_out MISMATCH");
        end
    endtask

    // ── Main stimulus ─────────────────────────────────────────
    initial begin
        rst = 1; valid_in = 0; data_in = 45'd0;

        $display("============================================================");
        $display("  BCH(45,32,t=2)  Decode-Stage 1-Bit Error Correction Demo ");
        $display("============================================================");
        $display("  Instruction : ADD x1,x1,x2  (0x%08X)", INSTR);
        $display("  Fault bit   : position %0d / 44", FAULT_BIT);
        $display("  clean_cw    : 0x%012X", clean_cw);
        $display("  corrupt_cw  : 0x%012X  (bit %0d flipped)", corrupt_cw, FAULT_BIT);
        $display("");

        // Hold reset for 4 clock cycles
        repeat(4) @(posedge clk); #1;
        rst = 0;

        // ── TEST 1: Clean codeword (baseline) ────────────────
        $display("-- Test 1: Clean codeword (no error) --");

        @(posedge clk); #1;
        valid_in = 1;
        data_in  = clean_cw;
        $display("  [CLEAN ] Cycle +0 : valid_in=1, presenting clean_cw=0x%012X", clean_cw);

        @(posedge clk); #1;     // Stage 1 captures S1,S3
        valid_in = 0;
        data_in  = 45'd0;
        $display("  [CLEAN ] Cycle +1 : Stage-1 done  | S1=0x%02X (expect 00 = no error)", uut.S1);

        @(posedge clk); #1;     // Stage 2 captures s1,s2
        $display("  [CLEAN ] Cycle +2 : Stage-2 done  | s1=0x%02X  s2=0x%02X (both 0)", uut.s1, uut.s2);

        @(posedge clk); #1;     // Stage 3 output
        show_output("CLEAN ");
        $display("");

        // ── TEST 2: 1-bit error at position FAULT_BIT ────────
        $display("-- Test 2: 1-bit error injected at bit position %0d --", FAULT_BIT);

        @(posedge clk); #1;
        valid_in = 1;
        data_in  = corrupt_cw;
        $display("  [1-BIT ] Cycle +0 : valid_in=1, presenting corrupt_cw=0x%012X", corrupt_cw);

        @(posedge clk); #1;     // Stage 1 — syndrome non-zero
        valid_in = 0;
        data_in  = 45'd0;
        $display("  [1-BIT ] Cycle +1 : Stage-1 done  | S1=0x%02X (non-zero => error detected!)", uut.S1);

        @(posedge clk); #1;     // Stage 2 — sigma2 = 0 => single-bit
        $display("  [1-BIT ] Cycle +2 : Stage-2 done  | s2=0x%02X (zero => single-bit error confirmed)", uut.s2);

        @(posedge clk); #1;     // Stage 3 — corrected output
        $display("  [1-BIT ] Cycle +3 : Stage-3 output:");
        show_output("1-BIT ");
        $display("           error_mask bit %0d = %b  (correction applied at exact fault location)",
                 FAULT_BIT, error_mask[FAULT_BIT]);

        // Final pass/fail
        $display("");
        if (valid_out && !fatal && data_out == INSTR && error_mask == (45'd1 << FAULT_BIT))
            $display(">>> RESULT: PASS -- 1-bit error at position %0d corrected, original instruction recovered.", FAULT_BIT);
        else
            $display(">>> RESULT: FAIL -- unexpected outcome.");

        // Extra idle cycles for clean waveform tail
        repeat(5) @(posedge clk); #1;

        $display("");
        $display("Waveform written to tb_1bit_correction.vcd");
        $display("Open with: gtkwave tb_1bit_correction.vcd tb_1bit_correction.gtkw");
        $finish;
    end

endmodule
