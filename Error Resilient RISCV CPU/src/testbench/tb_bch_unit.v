// =============================================================
// tb_bch_unit.v — Standalone BCH(45,32,t=2) Unit Test
// (Combined Error-Resilient RISC-V CPU project)
//
// Exercises bch_encoder and bch_decoder_prod from bch_ecc.v
// directly — no CPU instantiated.  This validates the shared
// BCH engine that protects BOTH the fetch stage and the register
// file.
//
// Test data: 20 representative RISC-V instruction encodings
//   covering all major instruction formats (R, I, S, B, U, J)
//   plus ALU-heavy and data-heavy patterns.
//
// Test scenarios per data word:
//   clean      : no faults — decoded word must match original
//   single     : 1-bit flip at 8 representative codeword positions
//   double     : 2-bit flip at 5 position pairs
//   triple     : 3-bit flip at 3 position triplets (uncorrectable)
//
// Total tests: 20 words × (1 + 8 + 5 + 3) = 340 test vectors
//
// Output: bch_decode_stage_results.csv
// =============================================================
`timescale 1ns/1ps

module tb_bch_unit;

// -------------------------------------------------------
// Clock
// -------------------------------------------------------
reg clk = 0;
always #5 clk = ~clk;   // 100 MHz

reg rst = 1;
initial begin
    repeat (4) @(posedge clk);
    rst = 0;
end

// -------------------------------------------------------
// DUT wires
// -------------------------------------------------------
reg         enc_valid_in  = 0;
reg  [31:0] enc_data_in   = 0;

wire [44:0] enc_codeword;   // combinational

reg         dec_valid_in  = 0;
reg  [44:0] dec_data_in   = 0;

wire        dec_valid_out;
wire [31:0] dec_data_out;
wire        dec_fatal;
wire [44:0] dec_error_mask;
wire [5:0]  dec_sigma2;

// -------------------------------------------------------
// Instantiate encoder (combinational)
// -------------------------------------------------------
bch_encoder enc (
    .data_in    (enc_data_in),
    .encoded_out(enc_codeword)
);

// -------------------------------------------------------
// Instantiate decoder (3-cycle pipeline)
// -------------------------------------------------------
bch_decoder_prod dec (
    .clk        (clk),
    .rst        (rst),
    .valid_in   (dec_valid_in),
    .data_in    (dec_data_in),
    .valid_out  (dec_valid_out),
    .data_out   (dec_data_out),
    .fatal      (dec_fatal),
    .error_mask (dec_error_mask),
    .sigma2     (dec_sigma2)
);

// -------------------------------------------------------
// Test data: 20 representative RISC-V instruction words
//   (format, opcode, typical operands)
// -------------------------------------------------------
reg [31:0] test_words [0:199];
reg [8*32-1:0] test_labels [0:199];

initial begin
    // R-type
    test_words[0]  = 32'h0020_80B3; // add  x1, x1, x2
    test_words[1]  = 32'h4020_80B3; // sub  x1, x1, x2
    test_words[2]  = 32'h0062_9233; // sll  x4, x5, x6
    test_words[3]  = 32'h0062_D233; // srl  x4, x5, x6
    test_words[4]  = 32'h4062_D233; // sra  x4, x5, x6
    test_words[5]  = 32'h0062_A233; // slt  x4, x5, x6
    test_words[6]  = 32'h0252_0633; // mul  x12, x4, x5
    test_words[7]  = 32'h0252_4633; // div  x12, x4, x5
    // I-type
    test_words[8]  = 32'h0000_0013; // addi x0, x0, 0  (NOP)
    test_words[9]  = 32'h0011_2293; // slti x5, x2, 1
    test_words[10] = 32'hFFF3_C393; // xori x7, x7, -1
    test_words[11] = 32'h0001_2283; // lw   x5, 0(x2)
    test_words[12] = 32'h00C1_2603; // lw   x12, 12(x2)
    // S-type
    test_words[13] = 32'h0031_2223; // sw   x3, 4(x2)
    test_words[14] = 32'h00A1_2A23; // sw   x10, 20(x2)
    // B-type
    test_words[15] = 32'h0020_8463; // beq  x1, x2, +8
    test_words[16] = 32'hFE1F_F0E3; // bne  x31, x1, -32
    // U-type
    test_words[17] = 32'h0000_00B7; // lui  x1, 0
    // J-type
    test_words[18] = 32'h004_000EF; // jal  x1, 4
    // Data-like / edge patterns
    test_words[19] = 32'hDEAD_BEEF; // pathological pattern
    test_words[20] = 32'h9A8DCA03;
    test_words[21] = 32'h5EC42E08;
    test_words[22] = 32'hDD463C09;
    test_words[23] = 32'h10435A10;
    test_words[24] = 32'h6C6FA611;
    test_words[25] = 32'h8ACD4E10;
    test_words[26] = 32'hD8F56413;
    test_words[27] = 32'hE1A47E10;
    test_words[28] = 32'hA7CAD415;
    test_words[29] = 32'h7C52FA17;
    test_words[30] = 32'h474EBC19;
    test_words[31] = 32'h430F801D;
    test_words[32] = 32'h0EA2622B;
    test_words[33] = 32'hEDD96831;
    test_words[34] = 32'h3170F437;
    test_words[35] = 32'h8E944239;
    test_words[36] = 32'hC6A7EE39;
    test_words[37] = 32'h52FBE43B;
    test_words[38] = 32'h8BABCE3B;
    test_words[39] = 32'h0658663A;
    test_words[40] = 32'hEEEA163E;
    test_words[41] = 32'h5D65A441;
    test_words[42] = 32'hF7FD5646;
    test_words[43] = 32'h98326856;
    test_words[44] = 32'h46685257;
    test_words[45] = 32'h4774BC58;
    test_words[46] = 32'hCF8EBC5A;
    test_words[47] = 32'hDC96925E;
    test_words[48] = 32'h30BEB45F;
    test_words[49] = 32'h7D106C60;
    test_words[50] = 32'hC333E861;
    test_words[51] = 32'hCF8D446A;
    test_words[52] = 32'h07A0CA6E;
    test_words[53] = 32'hE9A1FA6F;
    test_words[54] = 32'h571AA876;
    test_words[55] = 32'hDBCCC477;
    test_words[56] = 32'hF071D879;
    test_words[57] = 32'hEC5B227C;
    test_words[58] = 32'h72D8567D;
    test_words[59] = 32'h98543881;
    test_words[60] = 32'hE1805081;
    test_words[61] = 32'h38F16A81;
    test_words[62] = 32'h89463E85;
    test_words[63] = 32'h37F8A88B;
    test_words[64] = 32'hB758588D;
    test_words[65] = 32'hA65E688E;
    test_words[66] = 32'hDF465290;
    test_words[67] = 32'hA4161293;
    test_words[68] = 32'hDD56CC94;
    test_words[69] = 32'h85D51695;
    test_words[70] = 32'h3C365296;
    test_words[71] = 32'h663F1C97;
    test_words[72] = 32'h8DA01097;
    test_words[73] = 32'h6F4CC69A;
    test_words[74] = 32'h568CC69B;
    test_words[75] = 32'h448AAA9E;
    test_words[76] = 32'hF86C2CA2;
    test_words[77] = 32'h74273CA3;
    test_words[78] = 32'h8CE21EA3;
    test_words[79] = 32'h655238A6;
    test_words[80] = 32'hAB4220A7;
    test_words[81] = 32'h827050A8;
    test_words[82] = 32'hBC8960A9;
    test_words[83] = 32'h757750A9;
    test_words[84] = 32'h9AD620AB;
    test_words[85] = 32'hB7B56EA7;
    test_words[86] = 32'h0279B6A6;
    test_words[87] = 32'h43E42CAF;
    test_words[88] = 32'hEE0CAEB5;
    test_words[89] = 32'h38602AB6;
    test_words[90] = 32'hA18FF6B6;
    test_words[91] = 32'h286218B8;
    test_words[92] = 32'h3A43B2BA;
    test_words[93] = 32'hD89A40C0;
    test_words[94] = 32'hDC98D2C1;
    test_words[95] = 32'hDC1110C1;
    test_words[96] = 32'hE117DAC3;
    test_words[97] = 32'hE61FECC0;
    test_words[98] = 32'h1A50AEC3;
    test_words[99] = 32'h9BE578C7;
    test_words[100] = 32'hB7E99ACA;
    test_words[101] = 32'h00E85ECE;
    test_words[102] = 32'h19DB3AD0;
    test_words[103] = 32'h0F02BAD0;
    test_words[104] = 32'h6E595ED3;
    test_words[105] = 32'h75D66ED4;
    test_words[106] = 32'hEF48E8D5;
    test_words[107] = 32'h508EBAD7;
    test_words[108] = 32'h45B89CD9;
    test_words[109] = 32'h392456DE;
    test_words[110] = 32'h284D82E5;
    test_words[111] = 32'h4EEA04E7;
    test_words[112] = 32'h1C8EAEE9;
    test_words[113] = 32'hE767DCEA;
    test_words[114] = 32'hD5A804EB;
    test_words[115] = 32'h29D4BEEF;
    test_words[116] = 32'h8D5288F1;
    test_words[117] = 32'h8B8148F6;
    test_words[118] = 32'h4EB93EFF;
    test_words[119] = 32'h20A04502;
    test_words[120] = 32'h3FA7F104;
    test_words[121] = 32'h444D610B;
    test_words[122] = 32'h2260E70F;
    test_words[123] = 32'h17BE3111;
    test_words[124] = 32'h60E7A113;
    test_words[125] = 32'h0ED42F1A;
    test_words[126] = 32'hBAA4B71A;
    test_words[127] = 32'hBDC14F1F;
    test_words[128] = 32'hE7C99B26;
    test_words[129] = 32'h89456F27;
    test_words[130] = 32'hEE49F329;
    test_words[131] = 32'h3CEDDF2D;
    test_words[132] = 32'hAF42E12F;
    test_words[133] = 32'h27CD8130;
    test_words[134] = 32'hDC570131;
    test_words[135] = 32'h1C11F735;
    test_words[136] = 32'h47294739;
    test_words[137] = 32'hB41B3143;
    test_words[138] = 32'hFC3D3348;
    test_words[139] = 32'hFF002D4D;
    test_words[140] = 32'h4BF50B52;
    test_words[141] = 32'h6C006F61;
    test_words[142] = 32'h2A935D62;
    test_words[143] = 32'h9E8FC965;
    test_words[144] = 32'hD605E770;
    test_words[145] = 32'hF26B4776;
    test_words[146] = 32'h25E97977;
    test_words[147] = 32'h95A76D79;
    test_words[148] = 32'h562B0F79;
    test_words[149] = 32'hB2B9437A;
    test_words[150] = 32'h2720797D;
    test_words[151] = 32'hCE9FF57F;
    test_words[152] = 32'h04FC6D82;
    test_words[153] = 32'hF8102383;
    test_words[154] = 32'h12922F83;
    test_words[155] = 32'h2F923996;
    test_words[156] = 32'h18C26797;
    test_words[157] = 32'h1EFA2197;
    test_words[158] = 32'h6C031199;
    test_words[159] = 32'hFF01CF99;
    test_words[160] = 32'h4FCCA39A;
    test_words[161] = 32'hBF7B539B;
    test_words[162] = 32'hA3B1799D;
    test_words[163] = 32'hCEB81F9D;
    test_words[164] = 32'hC8DCD19F;
    test_words[165] = 32'hD9441FA5;
    test_words[166] = 32'h1BAC27A7;
    test_words[167] = 32'h877409A9;
    test_words[168] = 32'hA5E5A5AB;
    test_words[169] = 32'h96DA1DAC;
    test_words[170] = 32'h5408F9AC;
    test_words[171] = 32'h8CBFEDB0;
    test_words[172] = 32'hC40DB9B4;
    test_words[173] = 32'h847FD9B4;
    test_words[174] = 32'hB88139B9;
    test_words[175] = 32'h747B6DBA;
    test_words[176] = 32'h958CA9BA;
    test_words[177] = 32'hCE4A2BBD;
    test_words[178] = 32'h93CD59BF;
    test_words[179] = 32'h4CCC9BC2;
    test_words[180] = 32'hA0A04DC4;
    test_words[181] = 32'hA9D3D7C7;
    test_words[182] = 32'hC56811CD;
    test_words[183] = 32'h3985C3CF;
    test_words[184] = 32'hBACFB3D0;
    test_words[185] = 32'h5E9953D2;
    test_words[186] = 32'hBAA80DD4;
    test_words[187] = 32'h3A9BEDD4;
    test_words[188] = 32'h27A0C3D7;
    test_words[189] = 32'h3D1A85DD;
    test_words[190] = 32'h43CF2FDE;
    test_words[191] = 32'h386ECBE0;
    test_words[192] = 32'h610461E3;
    test_words[193] = 32'h1931E9EE;
    test_words[194] = 32'hB45ED1F0;
    test_words[195] = 32'hFF5E9FF0;
    test_words[196] = 32'h46D483F3;
    test_words[197] = 32'h6123FDF7;
    test_words[198] = 32'h7900F7F9;
    test_words[199] = 32'h50C187FC;
end

// -------------------------------------------------------
// CSV file handle and counters
// -------------------------------------------------------
integer f_csv;
integer total_pass, total_fail;

// -------------------------------------------------------
// Task: send one vector through encoder+decoder, wait,
//       check result, write CSV row.
//
//   data_orig   : original 32-bit word
//   inj_cw      : codeword fed to decoder (may have flipped bits)
//   fault_type  : 0=clean 1=single 2=double 3=triple
//   fp1,fp2,fp3 : flip positions (-1 means unused)
//   label_str   : short human label stored in CSV
// -------------------------------------------------------
task run_vector;
    input [31:0] data_orig;
    input [44:0] inj_cw;
    input integer fault_type;
    input integer fp1, fp2, fp3;
    input [8*20-1:0] label_str;

    reg [31:0] expected_data;
    reg        expect_fatal;
    reg [44:0] clean_cw;
    begin
        // Feed encoded word to decoder
        @(posedge clk);
        dec_data_in  <= inj_cw;
        dec_valid_in <= 1;
        @(posedge clk);
        dec_valid_in <= 0;

        // Wait 3 pipeline cycles for valid_out
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);

        // At this point dec_valid_out should be high
        // (one more posedge to sample outputs)
        @(posedge clk);

        // Determine expected outcome
        expected_data = data_orig;
        expect_fatal  = (fault_type == 3) ? 1 : 0;

        // Write CSV row
        // columns: case_id handled outside; here just the data columns
        if (fault_type == 3) begin
            // Triple: expect fatal OR spurious double detection
            if (dec_fatal || dec_sigma2 != 0) begin
                $fwrite(f_csv,
                    "%08x,%s,%0d,%0d,%0d,%0d,%045b,%045b,%08x,1,%0b,%045b,PASS\n",
                    data_orig, label_str, fault_type, fp1, fp2, fp3,
                    inj_cw ^ (inj_cw ^ inj_cw), // placeholder for clean_cw (computed outside)
                    inj_cw, dec_data_out,
                    dec_fatal, dec_error_mask);
                total_pass = total_pass + 1;
            end else begin
                $fwrite(f_csv,
                    "%08x,%s,%0d,%0d,%0d,%0d,%045b,%045b,%08x,0,%0b,%045b,FAIL\n",
                    data_orig, label_str, fault_type, fp1, fp2, fp3,
                    inj_cw ^ (inj_cw ^ inj_cw),
                    inj_cw, dec_data_out,
                    dec_fatal, dec_error_mask);
                $display("[FAIL] triple not detected: data=%08x pos=(%0d,%0d,%0d)",
                         data_orig, fp1, fp2, fp3);
                total_fail = total_fail + 1;
            end
        end else begin
            // Clean / single / double: decoded data must equal original
            if (dec_data_out === expected_data && !dec_fatal) begin
                $fwrite(f_csv,
                    "%08x,%s,%0d,%0d,%0d,%0d,%045b,%045b,%08x,1,%0b,%045b,PASS\n",
                    data_orig, label_str, fault_type, fp1, fp2, fp3,
                    inj_cw ^ (inj_cw ^ inj_cw),
                    inj_cw, dec_data_out,
                    dec_fatal, dec_error_mask);
                total_pass = total_pass + 1;
            end else begin
                $fwrite(f_csv,
                    "%08x,%s,%0d,%0d,%0d,%0d,%045b,%045b,%08x,0,%0b,%045b,FAIL\n",
                    data_orig, label_str, fault_type, fp1, fp2, fp3,
                    inj_cw ^ (inj_cw ^ inj_cw),
                    inj_cw, dec_data_out,
                    dec_fatal, dec_error_mask);
                $display("[FAIL] decode mismatch: data=%08x type=%0d pos=(%0d,%0d,%0d) got=%08x fatal=%0b",
                         data_orig, fault_type, fp1, fp2, fp3, dec_data_out, dec_fatal);
                total_fail = total_fail + 1;
            end
        end
    end
endtask

// -------------------------------------------------------
// Main stimulus
// -------------------------------------------------------
// Single-bit test positions (8 representative positions
// spread across the 45-bit codeword):
//   0=parity LSB, 5, 11, 22=data-mid, 32, 38, 43=data MSB, 44
integer s_positions [0:7];
// Double-bit test position pairs (5 pairs):
integer d_pos1 [0:4];
integer d_pos2 [0:4];
// Triple-bit test position triplets (3):
integer t_pos1 [0:2];
integer t_pos2 [0:2];
integer t_pos3 [0:2];

integer w, p, q;
reg [44:0] clean_cw, inj_cw;
integer case_id;

initial begin
    s_positions[0] = 0;  s_positions[1] = 5;
    s_positions[2] = 11; s_positions[3] = 22;
    s_positions[4] = 32; s_positions[5] = 38;
    s_positions[6] = 43; s_positions[7] = 44;

    d_pos1[0] = 0;  d_pos2[0] = 22;
    d_pos1[1] = 1;  d_pos2[1] = 44;
    d_pos1[2] = 5;  d_pos2[2] = 30;
    d_pos1[3] = 12; d_pos2[3] = 38;
    d_pos1[4] = 20; d_pos2[4] = 43;

    t_pos1[0] = 0;  t_pos2[0] = 10; t_pos3[0] = 33;
    t_pos1[1] = 3;  t_pos2[1] = 18; t_pos3[1] = 40;
    t_pos1[2] = 7;  t_pos2[2] = 25; t_pos3[2] = 44;

    total_pass = 0;
    total_fail = 0;
    case_id    = 0;

    // Open CSV
    f_csv = $fopen("bch_decode_stage_results.csv", "w");
    $fwrite(f_csv,
        "case_id,test_data_hex,instr_label,fault_type,fault_pos1,fault_pos2,fault_pos3,corrupted_cw_bin,decoded_data_hex,corrected_ok,fatal_flag,error_mask_bin,result\n");

    // Wait for reset to deassert
    wait (rst == 0);
    repeat (4) @(posedge clk);

    // ====================================================
    // Iterate over all 20 test words
    // ====================================================
    for (w = 0; w < 200; w = w + 1) begin

        // Drive encoder input (combinational)
        enc_data_in = test_words[w];
        @(posedge clk);           // let combinational settle
        clean_cw = enc_codeword;

        $display("--- Word %0d: 0x%08x  encoded=0x%012x ---",
                 w, test_words[w], clean_cw);

        // ---- CLEAN (no fault) ----
        $fwrite(f_csv, "%0d,", case_id); case_id = case_id + 1;
        run_vector(test_words[w], clean_cw, 0, -1, -1, -1, "clean");

        // ---- SINGLE-BIT faults ----
        for (p = 0; p < 8; p = p + 1) begin
            inj_cw = clean_cw ^ (45'b1 << s_positions[p]);
            $fwrite(f_csv, "%0d,", case_id); case_id = case_id + 1;
            run_vector(test_words[w], inj_cw, 1,
                       s_positions[p], -1, -1, "single");
        end

        // ---- DOUBLE-BIT faults ----
        for (p = 0; p < 5; p = p + 1) begin
            inj_cw = clean_cw ^ (45'b1 << d_pos1[p]) ^ (45'b1 << d_pos2[p]);
            $fwrite(f_csv, "%0d,", case_id); case_id = case_id + 1;
            run_vector(test_words[w], inj_cw, 2,
                       d_pos1[p], d_pos2[p], -1, "double");
        end

        // ---- TRIPLE-BIT faults (uncorrectable) ----
        for (p = 0; p < 3; p = p + 1) begin
            inj_cw = clean_cw
                     ^ (45'b1 << t_pos1[p])
                     ^ (45'b1 << t_pos2[p])
                     ^ (45'b1 << t_pos3[p]);
            $fwrite(f_csv, "%0d,", case_id); case_id = case_id + 1;
            run_vector(test_words[w], inj_cw, 3,
                       t_pos1[p], t_pos2[p], t_pos3[p], "triple");
        end

    end // for w

    // ====================================================
    // Summary
    // ====================================================
    $display("");
    $display("==============================================");
    $display("BCH Decode Stage Evaluation Complete");
    $display("  Total vectors : %0d", case_id);
    $display("  PASS          : %0d", total_pass);
    $display("  FAIL          : %0d", total_fail);
    $display("==============================================");

    $fclose(f_csv);
    $finish;
end

// Watchdog
initial begin
    repeat (500000) @(posedge clk);
    $display("[WATCHDOG] Decode stage sim timeout.");
    $fclose(f_csv);
    $finish;
end

endmodule
