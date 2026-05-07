`timescale 1ns/1ps

// ============================================================
// Focused testbench: BCH(45,32,t=2) — No Error (Clean) path.
//
// Proves the decoder is transparent when the codeword is
// uncorrupted: S1=0, sigma2=0, error_mask=0, data_out=original.
//
// Pipeline depth = 3 clock cycles:
//   Cycle 1 — Syndrome computation  (S1, S3)
//   Cycle 2 — Error locator poly    (sigma2 / s2)
//   Cycle 3 — Chien search + output (error_mask, data_out)
// ============================================================

module tb_no_error;

    // ── DUT ports ────────────────────────────────────────────
    reg         clk, rst, valid_in;
    reg  [44:0] data_in;
    wire        valid_out;
    wire        fatal;
    wire [31:0] data_out;
    wire [44:0] error_mask;
    wire [ 5:0] sigma2;

    // ── Instruction under test ───────────────────────────────
    localparam [31:0] INSTR = 32'h00208033; // ADD x1,x1,x2

    // ── Encoder: derive clean codeword ──────────────────────
    wire [44:0] clean_cw;
    bch_encoder_final enc_i (.data_in(INSTR), .codeword(clean_cw));

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
        $dumpfile("tb_no_error.vcd");
        $dumpvars(0, tb_no_error);
    end

    // ── Main stimulus ─────────────────────────────────────────
    initial begin
        rst = 1; valid_in = 0; data_in = 45'd0;

        $display("============================================================");
        $display("  BCH(45,32,t=2)  No-Error (Clean Codeword) Demo            ");
        $display("============================================================");
        $display("  Instruction : ADD x1,x1,x2  (0x%08X)", INSTR);
        $display("  clean_cw    : 0x%012X  (no bits flipped)", clean_cw);
        $display("");

        // Hold reset for 4 cycles
        repeat(4) @(posedge clk); #1;
        rst = 0;

        // ── TEST: Clean codeword ──────────────────────────────
        $display("-- Presenting clean codeword (zero faults) --");

        @(posedge clk); #1;
        valid_in = 1;
        data_in  = clean_cw;
        $display("  Cycle +0 : valid_in=1, data_in=0x%012X", clean_cw);

        @(posedge clk); #1;     // Stage 1 captures
        valid_in = 0;
        data_in  = 45'd0;
        $display("  Cycle +1 : Stage-1 done | S1=0x%02X S3=0x%02X (both 0 => no error)", uut.S1, uut.S3);

        @(posedge clk); #1;     // Stage 2 captures
        $display("  Cycle +2 : Stage-2 done | s1=0x%02X s2=0x%02X (both 0 => no correction needed)", uut.s1, uut.s2);

        @(posedge clk); #1;     // Stage 3 output
        $display("  Cycle +3 : OUTPUT:");
        $display("             valid_out  = %b",         valid_out);
        $display("             data_out   = 0x%08X",      data_out);
        $display("             error_mask = 0x%012X",     error_mask);
        $display("             fatal      = %b",          fatal);
        $display("             sigma2     = 0x%02X",      sigma2);

        if (valid_out && data_out == INSTR && !fatal && error_mask == 45'd0)
            $display("\n>>> RESULT: PASS -- clean codeword passed through unchanged. No correction applied.");
        else
            $display("\n>>> RESULT: FAIL -- unexpected result.");

        // Idle tail
        repeat(5) @(posedge clk); #1;

        $display("\nWaveform: tb_no_error.vcd");
        $finish;
    end

endmodule
