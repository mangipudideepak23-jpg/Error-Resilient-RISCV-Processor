// ==========================================================
// BCH-PROTECTED REGISTER FILE FOR PicoRV32
// (Deepak's work — register-file ECC at the decode/writeback stage)
//
// Drop-in replacement for the default `picorv32_regs` module.
// Activated by: `define PICORV32_REGS bch_regfile  (at compile time)
//
// Protection: BCH(45,32, t=2)  — same GF and polynomial as the
//   fetch-stage wrapper (ecc_instr_memory / bch_ecc.v).
//   - WRITE path: data is encoded with bch_encode_word and
//     stored as a 45-bit codeword in each register slot (x0-x31).
//   - READ path: a fully combinational 1-cycle syndrome checker
//     detects and corrects single-bit errors on the fly.
//     Double-bit errors are detected (asserts reg_file_double_err).
//     Uncorrectable errors: asserts reg_file_fatal.
//
// This 1-cycle correction keeps the register-read latency at one
// cycle, matching PicoRV32's combinational register read requirement.
//
// Fault injection (testbench only - driven via hierarchical force):
//   inject_reg_fault  - enable fault injection into the read path
//   inject_reg_double - also flip a second bit
//   fault_reg_addr    - register address to corrupt [5:0]
//   fault_reg_bit1    - first bit position in 45-bit codeword to flip
//   fault_reg_bit2    - second bit position to flip (if double)
//
// ECC status outputs (sticky, cleared on reset):
//   reg_file_single_err  - at least one single-bit error corrected
//   reg_file_double_err  - at least one double-bit error detected
//   reg_file_fatal       - at least one uncorrectable error detected
//
// Integration note: when instantiated via PICORV32_REGS, only the
// standard ports are wired by picorv32.v. The sticky-flag initial
// block below ensures they start at 0 even when resetn is undriven.
// Scan/status outputs are accessed via hierarchical reference in tb.
// ==========================================================
`timescale 1ns/1ps

module bch_regfile (
    input         clk,
    input         resetn,   // active-low reset
    input         wen,
    input  [5:0]  waddr,
    input  [5:0]  raddr1,
    input  [5:0]  raddr2,
    input  [31:0] wdata,
    output [31:0] rdata1,
    output [31:0] rdata2,

    // ---- Scan port: testbench can read any reg through BCH corrector ----
    input  [5:0]  scan_addr,      // register to scan
    output [31:0] scan_rdata,     // corrected data
    output        scan_single_err,// single-bit error detected
    output        scan_double_err,// double-bit error detected
    output        scan_fatal,     // uncorrectable error

    // ---- Fault injection (driven via force from testbench) ----
    output reg    reg_file_single_err,
    output reg    reg_file_double_err,
    output reg    reg_file_fatal
);

// Fault injection controls — declared as regs so
// the testbench can drive them via hierarchical force
reg        inject_reg_fault  = 0;
reg        inject_reg_double = 0;
reg [5:0]  fault_reg_addr    = 0;
reg [5:0]  fault_reg_bit1    = 0;
reg [5:0]  fault_reg_bit2    = 0;

// ----------------------------------------------------------
// Storage: 64 slots of 45-bit BCH codewords
// ----------------------------------------------------------
reg [44:0] regfile_bch [0:63];

integer ri;
initial begin
    for (ri = 0; ri < 64; ri = ri + 1)
        regfile_bch[ri] = 45'd0;
    // Safe defaults for sticky flags when resetn is not driven
    reg_file_single_err = 1'b0;
    reg_file_double_err = 1'b0;
    reg_file_fatal      = 1'b0;
end

// ----------------------------------------------------------
// WRITE PATH: encode wdata and store codeword
// Uses the same bch_encode_word function as the fetch wrapper
// ----------------------------------------------------------
function [44:0] bch_encode_word;
    input [31:0] d;
    reg [43:0] dr;
    integer j;
    begin
        dr = {d, 12'b0};
        for (j = 43; j >= 12; j = j - 1) begin
            if (dr[j]) begin
                dr[j]    = 1'b0;
                dr[j-2]  = dr[j-2]  ^ 1'b1;
                dr[j-4]  = dr[j-4]  ^ 1'b1;
                dr[j-7]  = dr[j-7]  ^ 1'b1;
                dr[j-8]  = dr[j-8]  ^ 1'b1;
                dr[j-9]  = dr[j-9]  ^ 1'b1;
                dr[j-12] = dr[j-12] ^ 1'b1;
            end
        end
        bch_encode_word = {1'b0, d, dr[11:0]};
    end
endfunction

always @(posedge clk) begin
    if (wen && |waddr[4:0])
        regfile_bch[waddr] <= bch_encode_word(wdata);
end

// ----------------------------------------------------------
// READ PATH: combinational BCH single-error correction
//
// GF(2^6) primitive polynomial: x^6 + x + 1
// Generator g(x) = LCM(m1,m3) → syndromes S1, S3
//
// Functions below mirror bch_decoder_prod but are purely
// combinational (no pipeline registers).
// ----------------------------------------------------------

// GF(2^6) multiply (same as bch_decoder_prod)
function [5:0] rf_gf_mul;
    input [5:0] a, b;
    reg [10:0] p;
    begin
        p = 0;
        if (b[0]) p = p ^ {5'b0, a};
        if (b[1]) p = p ^ {4'b0, a, 1'b0};
        if (b[2]) p = p ^ {3'b0, a, 2'b0};
        if (b[3]) p = p ^ {2'b0, a, 3'b0};
        if (b[4]) p = p ^ {1'b0, a, 4'b0};
        if (b[5]) p = p ^ {a, 5'b0};
        rf_gf_mul[0] = p[0] ^ p[6];
        rf_gf_mul[1] = p[1] ^ p[6] ^ p[7];
        rf_gf_mul[2] = p[2] ^ p[7] ^ p[8];
        rf_gf_mul[3] = p[3] ^ p[8] ^ p[9];
        rf_gf_mul[4] = p[4] ^ p[9] ^ p[10];
        rf_gf_mul[5] = p[5] ^ p[10];
    end
endfunction

function [5:0] rf_gf_inv;
    input [5:0] a;
    begin
        case(a)
            6'd0:  rf_gf_inv = 6'd0;  6'd1:  rf_gf_inv = 6'd1;
            6'd2:  rf_gf_inv = 6'd33; 6'd3:  rf_gf_inv = 6'd62;
            6'd4:  rf_gf_inv = 6'd49; 6'd5:  rf_gf_inv = 6'd43;
            6'd6:  rf_gf_inv = 6'd31; 6'd7:  rf_gf_inv = 6'd44;
            6'd8:  rf_gf_inv = 6'd57; 6'd9:  rf_gf_inv = 6'd37;
            6'd10: rf_gf_inv = 6'd52; 6'd11: rf_gf_inv = 6'd28;
            6'd12: rf_gf_inv = 6'd46; 6'd13: rf_gf_inv = 6'd40;
            6'd14: rf_gf_inv = 6'd22; 6'd15: rf_gf_inv = 6'd25;
            6'd16: rf_gf_inv = 6'd61; 6'd17: rf_gf_inv = 6'd54;
            6'd18: rf_gf_inv = 6'd51; 6'd19: rf_gf_inv = 6'd39;
            6'd20: rf_gf_inv = 6'd26; 6'd21: rf_gf_inv = 6'd35;
            6'd22: rf_gf_inv = 6'd14; 6'd23: rf_gf_inv = 6'd24;
            6'd24: rf_gf_inv = 6'd23; 6'd25: rf_gf_inv = 6'd15;
            6'd26: rf_gf_inv = 6'd20; 6'd27: rf_gf_inv = 6'd34;
            6'd28: rf_gf_inv = 6'd11; 6'd29: rf_gf_inv = 6'd53;
            6'd30: rf_gf_inv = 6'd45; 6'd31: rf_gf_inv = 6'd6;
            6'd32: rf_gf_inv = 6'd63; 6'd33: rf_gf_inv = 6'd2;
            6'd34: rf_gf_inv = 6'd27; 6'd35: rf_gf_inv = 6'd21;
            6'd36: rf_gf_inv = 6'd56; 6'd37: rf_gf_inv = 6'd9;
            6'd38: rf_gf_inv = 6'd50; 6'd39: rf_gf_inv = 6'd19;
            6'd40: rf_gf_inv = 6'd13; 6'd41: rf_gf_inv = 6'd47;
            6'd42: rf_gf_inv = 6'd48; 6'd43: rf_gf_inv = 6'd5;
            6'd44: rf_gf_inv = 6'd7;  6'd45: rf_gf_inv = 6'd30;
            6'd46: rf_gf_inv = 6'd12; 6'd47: rf_gf_inv = 6'd41;
            6'd48: rf_gf_inv = 6'd42; 6'd49: rf_gf_inv = 6'd4;
            6'd50: rf_gf_inv = 6'd38; 6'd51: rf_gf_inv = 6'd18;
            6'd52: rf_gf_inv = 6'd10; 6'd53: rf_gf_inv = 6'd29;
            6'd54: rf_gf_inv = 6'd17; 6'd55: rf_gf_inv = 6'd60;
            6'd56: rf_gf_inv = 6'd36; 6'd57: rf_gf_inv = 6'd8;
            6'd58: rf_gf_inv = 6'd59; 6'd59: rf_gf_inv = 6'd58;
            6'd60: rf_gf_inv = 6'd55; 6'd61: rf_gf_inv = 6'd16;
            6'd62: rf_gf_inv = 6'd3;  6'd63: rf_gf_inv = 6'd32;
            default: rf_gf_inv = 6'd0;
        endcase
    end
endfunction

function [5:0] rf_alpha_b;
    input [6:0] e;
    begin
        case(e)
            7'd0:  rf_alpha_b = 6'd1;  7'd1:  rf_alpha_b = 6'd2;
            7'd2:  rf_alpha_b = 6'd4;  7'd3:  rf_alpha_b = 6'd8;
            7'd4:  rf_alpha_b = 6'd16; 7'd5:  rf_alpha_b = 6'd32;
            7'd6:  rf_alpha_b = 6'd3;  7'd7:  rf_alpha_b = 6'd6;
            7'd8:  rf_alpha_b = 6'd12; 7'd9:  rf_alpha_b = 6'd24;
            7'd10: rf_alpha_b = 6'd48; 7'd11: rf_alpha_b = 6'd35;
            7'd12: rf_alpha_b = 6'd5;  7'd13: rf_alpha_b = 6'd10;
            7'd14: rf_alpha_b = 6'd20; 7'd15: rf_alpha_b = 6'd40;
            7'd16: rf_alpha_b = 6'd19; 7'd17: rf_alpha_b = 6'd38;
            7'd18: rf_alpha_b = 6'd15; 7'd19: rf_alpha_b = 6'd30;
            7'd20: rf_alpha_b = 6'd60; 7'd21: rf_alpha_b = 6'd59;
            7'd22: rf_alpha_b = 6'd53; 7'd23: rf_alpha_b = 6'd41;
            7'd24: rf_alpha_b = 6'd17; 7'd25: rf_alpha_b = 6'd34;
            7'd26: rf_alpha_b = 6'd7;  7'd27: rf_alpha_b = 6'd14;
            7'd28: rf_alpha_b = 6'd28; 7'd29: rf_alpha_b = 6'd56;
            7'd30: rf_alpha_b = 6'd51; 7'd31: rf_alpha_b = 6'd37;
            7'd32: rf_alpha_b = 6'd9;  7'd33: rf_alpha_b = 6'd18;
            7'd34: rf_alpha_b = 6'd36; 7'd35: rf_alpha_b = 6'd11;
            7'd36: rf_alpha_b = 6'd22; 7'd37: rf_alpha_b = 6'd44;
            7'd38: rf_alpha_b = 6'd27; 7'd39: rf_alpha_b = 6'd54;
            7'd40: rf_alpha_b = 6'd47; 7'd41: rf_alpha_b = 6'd29;
            7'd42: rf_alpha_b = 6'd58; 7'd43: rf_alpha_b = 6'd55;
            7'd44: rf_alpha_b = 6'd45; 7'd45: rf_alpha_b = 6'd25;
            7'd46: rf_alpha_b = 6'd50; 7'd47: rf_alpha_b = 6'd39;
            7'd48: rf_alpha_b = 6'd13; 7'd49: rf_alpha_b = 6'd26;
            7'd50: rf_alpha_b = 6'd52; 7'd51: rf_alpha_b = 6'd43;
            7'd52: rf_alpha_b = 6'd21; 7'd53: rf_alpha_b = 6'd42;
            7'd54: rf_alpha_b = 6'd23; 7'd55: rf_alpha_b = 6'd46;
            7'd56: rf_alpha_b = 6'd31; 7'd57: rf_alpha_b = 6'd62;
            7'd58: rf_alpha_b = 6'd63; 7'd59: rf_alpha_b = 6'd61;
            7'd60: rf_alpha_b = 6'd57; 7'd61: rf_alpha_b = 6'd49;
            7'd62: rf_alpha_b = 6'd33;
            default: rf_alpha_b = 6'd0;
        endcase
    end
endfunction

function [6:0] rf_mod63_mul3;
    input [5:0] i6;
    reg [7:0] t;
    begin
        t = {2'b00, i6} + {1'b0, i6, 1'b0};
        if (t >= 8'd126)      rf_mod63_mul3 = t - 8'd126;
        else if (t >= 8'd63)  rf_mod63_mul3 = t - 8'd63;
        else                  rf_mod63_mul3 = t[6:0];
    end
endfunction

function [6:0] rf_mod63_neg;
    input [5:0] k6;
    begin
        if (k6 == 6'd0)  rf_mod63_neg = 7'd0;
        else             rf_mod63_neg = 7'd63 - {1'b0, k6};
    end
endfunction

function [6:0] rf_mod63_mul2;
    input [6:0] x;
    reg [7:0] t;
    begin
        t = {1'b0, x, 1'b0};
        if (t >= 8'd63)  rf_mod63_mul2 = t - 8'd63;
        else             rf_mod63_mul2 = t[6:0];
    end
endfunction

// ----------------------------------------------------------
// Combinational BCH corrector for one codeword
// Returns corrected 32-bit data, plus status flags
// ----------------------------------------------------------
function [34:0] bch_correct_cw;
    // Returns: {fatal(1), double_err(1), single_err(1), corrected_data(32)}
    input [44:0] cw;
    reg [5:0]  S1, S3;
    reg [5:0]  s1, s2;
    reg [44:0] mask;
    reg [6:0]  exp_s1, exp_s3;
    reg [6:0]  chien_e1, chien_e2;
    integer    ii, kk;
    reg [44:0] corrected;
    reg        is_fatal, is_double, is_single;
    begin
        // --- Syndrome computation ---
        S1 = 6'd0; S3 = 6'd0;
        for (ii = 0; ii < 45; ii = ii + 1) begin
            if (cw[ii]) begin
                exp_s1 = ii[6:0];
                exp_s3 = rf_mod63_mul3(ii[5:0]);
                S1 = S1 ^ rf_alpha_b(exp_s1);
                S3 = S3 ^ rf_alpha_b(exp_s3);
            end
        end

        s1 = S1;
        if (S1 == 6'd0)
            s2 = 6'd0;
        else
            s2 = rf_gf_mul(rf_gf_mul(rf_gf_mul(S1,S1),S1) ^ S3, rf_gf_inv(S1));

        // --- Chien search for error location polynomial ---
        mask = 45'd0;
        for (kk = 0; kk < 45; kk = kk + 1) begin
            chien_e1 = rf_mod63_neg(kk[5:0]);
            chien_e2 = rf_mod63_mul2(chien_e1);
            if ((6'd1 ^
                 rf_gf_mul(s1, rf_alpha_b(chien_e1)) ^
                 rf_gf_mul(s2, rf_alpha_b(chien_e2))) == 6'd0)
                mask[kk] = 1'b1;
        end

        corrected = cw ^ mask;

        is_single  = (S1 != 6'd0) && (mask != 45'd0) && ($countones(mask) == 1);
        is_double  = (S1 != 6'd0) && (mask != 45'd0) && ($countones(mask) != 1);
        is_fatal   = (S1 != 6'd0) && (mask == 45'd0);

        bch_correct_cw = {is_fatal, is_double, is_single, corrected[43:12]};
    end
endfunction

// ----------------------------------------------------------
// Apply fault injection to a stored codeword
// ----------------------------------------------------------
function [44:0] apply_fault;
    input [44:0] cw;
    input [5:0]  addr_check;   // register address of this read
    input         do_inject;
    input         do_double;
    input [5:0]   target_addr;
    input [5:0]   bit1, bit2;
    reg [44:0] out;
    begin
        out = cw;
        if (do_inject && (addr_check == target_addr)) begin
            if (bit1 < 6'd45) out = out ^ (45'b1 << bit1);
            if (do_double && (bit2 < 6'd45)) out = out ^ (45'b1 << bit2);
        end
        apply_fault = out;
    end
endfunction

// ----------------------------------------------------------
// Read port 1 — combinational, with BCH correction
// ----------------------------------------------------------
wire [44:0] raw_cw1    = regfile_bch[raddr1];
wire [44:0] faulted_cw1 = apply_fault(raw_cw1, raddr1,
                               inject_reg_fault, inject_reg_double,
                               fault_reg_addr, fault_reg_bit1, fault_reg_bit2);
wire [34:0] corr_res1  = bch_correct_cw(faulted_cw1);

assign rdata1 = raddr1 ? corr_res1[31:0] : 32'd0;   // x0 always reads 0

// ----------------------------------------------------------
// Read port 2 — combinational, with BCH correction
// ----------------------------------------------------------
wire [44:0] raw_cw2    = regfile_bch[raddr2];
wire [44:0] faulted_cw2 = apply_fault(raw_cw2, raddr2,
                               inject_reg_fault, inject_reg_double,
                               fault_reg_addr, fault_reg_bit1, fault_reg_bit2);
wire [34:0] corr_res2  = bch_correct_cw(faulted_cw2);

assign rdata2 = raddr2 ? corr_res2[31:0] : 32'd0;   // x0 always reads 0

// ----------------------------------------------------------
// ECC status registers — sticky, cleared on reset
// Also triggered by the scan port (for testbench monitoring)
// ----------------------------------------------------------
wire [44:0] scan_cw  = regfile_bch[scan_addr];
wire [34:0] scan_res = bch_correct_cw(scan_cw);

assign scan_rdata     = scan_addr ? scan_res[31:0] : 32'd0;
assign scan_single_err = scan_res[32];
assign scan_double_err = scan_res[33];
assign scan_fatal      = scan_res[34];

wire single1 = corr_res1[32];
wire double1 = corr_res1[33];
wire fatal1  = corr_res1[34];
wire single2 = corr_res2[32];
wire double2 = corr_res2[33];
wire fatal2  = corr_res2[34];

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        reg_file_single_err <= 1'b0;
        reg_file_double_err <= 1'b0;
        reg_file_fatal      <= 1'b0;
    end else begin
        if (single1 || single2 || scan_single_err) reg_file_single_err <= 1'b1;
        if (double1 || double2 || scan_double_err) reg_file_double_err <= 1'b1;
        if (fatal1  || fatal2  || scan_fatal)      reg_file_fatal      <= 1'b1;
    end
end

// No initial block needed — reset clears flags via always block above

endmodule
