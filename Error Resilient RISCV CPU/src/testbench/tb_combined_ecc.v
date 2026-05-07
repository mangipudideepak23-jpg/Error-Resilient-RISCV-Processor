// =============================================================
// tb_combined_ecc.v — Full System Testbench
// Combined BCH Error-Resilient RISC-V CPU
//
// Simulates the complete dual-protection error-resilient CPU:
//   - PicoRV32 (RV32IMC) running firmware/firmware.hex
//   - BCH(45,32,t=2) at the FETCH STAGE  (ecc_instr_memory)
//   - BCH(45,32,t=2) at the REGISTER FILE (bch_regfile, x0-x31)
//
// Firmware: hello.c (10 test blocks covering ALU, shifts, mul/div,
//   Fibonacci, popcount, byte-reverse, scratchpad, bitmask).
//   Firmware writes 0x600d0001 to TEST_STATUS_ADDR (0x03000000)
//   on success.
//
// TEST PLAN
// ---------
// Category A — Fetch-stage (instruction memory) fault injection:
//   A0 : Golden run — no faults injected
//   A1 : Single-bit faults — 9 cases (3 word indices × 3 bit positions)
//   A2 : Double-bit faults — 6 cases (3 word indices × 2 position pairs)
//   A3 : Triple-bit faults — 3 cases (expected fatal/trap)
//
// Category B — Register-file fault injection (via hierarchical force):
//   B0 : Golden run
//   B1 : Single-bit register faults on x1, x5, x10, x15, x28 (5 cases)
//   B2 : Double-bit register faults on same 5 registers
//   B3 : Triple-bit register faults (expected fatal) on x1, x10, x28
//
// Category C — Concurrent dual-layer fault injection:
//   C1 : Simultaneous fetch + register-file single-bit faults
//   C2 : Simultaneous fetch + register-file double-bit faults
//
// Output: combined_ecc_results.csv
//   Columns: test_id, category, fetch_fault, rf_fault, firmware_pass,
//            trap_seen, fetch_single_cnt, fetch_double_cnt, fetch_fatal_cnt,
//            rf_single_err, rf_double_err, rf_fatal, result
// =============================================================

`timescale 1ns/1ps

module tb_combined_ecc;

// =====================================================
// Clock and reset
// =====================================================
reg clk    = 0;
reg resetn = 0;
always #5 clk = ~clk;   // 100 MHz

// =====================================================
// Fetch-stage fault injection controls (DUT top ports)
// =====================================================
reg        fi_fetch_en      = 0;
reg        fi_fetch_double  = 0;
reg [31:0] fi_fetch_word_idx = 0;
reg [ 5:0] fi_fetch_pos1    = 0;
reg [ 5:0] fi_fetch_pos2    = 0;

// =====================================================
// DUT outputs
// =====================================================
wire        trap;
wire [31:0] fetch_single_err_cnt;
wire [31:0] fetch_double_err_cnt;
wire [31:0] fetch_fatal_err_cnt;
wire        fetch_bch_fatal;
wire        test_done;
wire        test_passed;
wire [31:0] test_status;
wire [31:0] test_signature;

// =====================================================
// DUT instantiation
// =====================================================
picorv32_ecc_top #(
    .MEM_SIZE      (128*1024),
    .PROGADDR_RESET(32'h0000_0000),
    .PROGADDR_IRQ  (32'h0000_0010)
) dut (
    .clk               (clk),
    .resetn            (resetn),
    .trap              (trap),

    .fetch_single_err_cnt (fetch_single_err_cnt),
    .fetch_double_err_cnt (fetch_double_err_cnt),
    .fetch_fatal_err_cnt  (fetch_fatal_err_cnt),
    .fetch_bch_fatal      (fetch_bch_fatal),

    .fi_fetch_en       (fi_fetch_en),
    .fi_fetch_double   (fi_fetch_double),
    .fi_fetch_word_idx (fi_fetch_word_idx),
    .fi_fetch_pos1     (fi_fetch_pos1),
    .fi_fetch_pos2     (fi_fetch_pos2),

    .test_done         (test_done),
    .test_passed       (test_passed),
    .test_status       (test_status),
    .test_signature    (test_signature)
);

// =====================================================
// Hierarchical paths into the BCH register file
//   (bch_regfile is instantiated as dut.cpu_core.regs)
// =====================================================
// Read back sticky error flags from the register file ECC
`define RF  dut.cpu_core.rf

// =====================================================
// CSV output
// =====================================================
integer fcsv;
integer test_id;
integer total_pass, total_fail;

// =====================================================
// Utility tasks
// =====================================================

// Reset the CPU (deassert then wait for firmware start)
task do_reset;
    integer cyc;
    begin
        resetn = 0;
        fi_fetch_en     = 0;
        fi_fetch_double = 0;
        fi_fetch_word_idx = 0;
        fi_fetch_pos1   = 0;
        fi_fetch_pos2   = 0;
        // Deassert register-file fault injection
        force `RF.inject_reg_fault  = 1'b0;
        force `RF.inject_reg_double = 1'b0;
        force `RF.fault_reg_addr    = 6'd0;
        force `RF.fault_reg_bit1    = 6'd0;
        force `RF.fault_reg_bit2    = 6'd0;
        repeat (8) @(posedge clk);
        resetn = 1;
    end
endtask

// Run firmware to completion (or timeout).
// Returns 1 if firmware passed, 0 otherwise.
task run_firmware;
    output reg fw_pass;
    output reg saw_trap;
    integer cyc;
    begin
        fw_pass  = 0;
        saw_trap = 0;
        for (cyc = 0; cyc < 200000; cyc = cyc + 1) begin
            @(posedge clk);
            if (trap) begin
                saw_trap = 1;
            end
            if (test_done) begin
                fw_pass = test_passed;
                cyc = 200000; // break
            end
        end
    end
endtask

// Write one CSV result row
task write_csv_row;
    input [8*8-1:0]  cat_str;   // category label e.g. "A0"
    input [8*16-1:0] fdesc;     // fault description
    input [8*16-1:0] rfdesc;    // rf fault description
    input            fw_pass;
    input            saw_trap;
    input            rf_single_ok;
    input            rf_double_ok;
    input            rf_fatal_ok;
    input [8*8-1:0]  result_str;
    begin
        $fwrite(fcsv,
            "%0d,%s,%s,%s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%s\n",
            test_id, cat_str,
            fdesc, rfdesc,
            fw_pass, saw_trap,
            fetch_single_err_cnt, fetch_double_err_cnt, fetch_fatal_err_cnt,
            rf_single_ok, rf_double_ok, rf_fatal_ok,
            result_str);
        if (result_str == "PASS")
            total_pass = total_pass + 1;
        else
            total_fail = total_fail + 1;
        test_id = test_id + 1;
    end
endtask

// =====================================================
// Helper: evaluate fetch single/double test
//   Expected: firmware passes and at least one error counted
// =====================================================
task eval_fetch_correctable;
    input [8*8-1:0]  cat;
    input [8*16-1:0] fdesc;
    input            fw_ok;
    input            saw_trap;
    reg passed;
    begin
        // firmware must pass; no trap; at least one error counted
        passed = fw_ok && !saw_trap &&
                 (fetch_single_err_cnt > 0 || fetch_double_err_cnt > 0);
        write_csv_row(cat, fdesc, "none",
                      fw_ok, saw_trap,
                      `RF.reg_file_single_err,
                      `RF.reg_file_double_err,
                      `RF.reg_file_fatal,
                      passed ? "PASS" : "FAIL");
        if (!passed)
            $display("[FAIL] %s %s  fw=%0d trap=%0d sc=%0d dc=%0d fc=%0d",
                     cat, fdesc, fw_ok, saw_trap,
                     fetch_single_err_cnt, fetch_double_err_cnt, fetch_fatal_err_cnt);
    end
endtask

// Helper: evaluate fetch triple test (expect fatal/trap)
task eval_fetch_fatal;
    input [8*8-1:0]  cat;
    input [8*16-1:0] fdesc;
    input            fw_ok;
    input            saw_trap;
    reg passed;
    begin
        // Either firmware saw a trap OR fatal counter incremented
        passed = saw_trap || (fetch_fatal_err_cnt > 0);
        write_csv_row(cat, fdesc, "none",
                      fw_ok, saw_trap,
                      `RF.reg_file_single_err,
                      `RF.reg_file_double_err,
                      `RF.reg_file_fatal,
                      passed ? "PASS" : "FAIL");
        if (!passed)
            $display("[FAIL] %s %s  triple not detected: fc=%0d trap=%0d",
                     cat, fdesc, fetch_fatal_err_cnt, saw_trap);
    end
endtask

// Helper: evaluate register-file fault test
task eval_rf_single;
    input [8*8-1:0]  cat;
    input [8*16-1:0] rfdesc;
    input            fw_ok;
    input            saw_trap;
    reg passed;
    begin
        // firmware must pass, no trap, rf_single_err must be set
        passed = fw_ok && !saw_trap && `RF.reg_file_single_err;
        write_csv_row(cat, "none", rfdesc,
                      fw_ok, saw_trap,
                      `RF.reg_file_single_err,
                      `RF.reg_file_double_err,
                      `RF.reg_file_fatal,
                      passed ? "PASS" : "FAIL");
        if (!passed)
            $display("[FAIL] %s %s  fw=%0d trap=%0d rf_single=%0d",
                     cat, rfdesc, fw_ok, saw_trap, `RF.reg_file_single_err);
    end
endtask

task eval_rf_double;
    input [8*8-1:0]  cat;
    input [8*16-1:0] rfdesc;
    input            fw_ok;
    input            saw_trap;
    reg passed;
    begin
        passed = fw_ok && !saw_trap && `RF.reg_file_double_err;
        write_csv_row(cat, "none", rfdesc,
                      fw_ok, saw_trap,
                      `RF.reg_file_single_err,
                      `RF.reg_file_double_err,
                      `RF.reg_file_fatal,
                      passed ? "PASS" : "FAIL");
        if (!passed)
            $display("[FAIL] %s %s  fw=%0d trap=%0d rf_double=%0d",
                     cat, rfdesc, fw_ok, saw_trap, `RF.reg_file_double_err);
    end
endtask

task eval_rf_fatal;
    input [8*8-1:0]  cat;
    input [8*16-1:0] rfdesc;
    input            fw_ok;
    input            saw_trap;
    reg passed;
    begin
        // Triple RF fault: expect rf_fatal OR firmware trap
        passed = `RF.reg_file_fatal || saw_trap;
        write_csv_row(cat, "none", rfdesc,
                      fw_ok, saw_trap,
                      `RF.reg_file_single_err,
                      `RF.reg_file_double_err,
                      `RF.reg_file_fatal,
                      passed ? "PASS" : "FAIL");
        if (!passed)
            $display("[FAIL] %s %s  rf_fatal=%0d trap=%0d",
                     cat, rfdesc, `RF.reg_file_fatal, saw_trap);
    end
endtask

// =====================================================
// Main stimulus
// =====================================================
reg fw_ok, saw_trap;

initial begin
    test_id    = 0;
    total_pass = 0;
    total_fail = 0;

    fcsv = $fopen("combined_ecc_results.csv", "w");
    $fwrite(fcsv,
        "test_id,category,fetch_fault,rf_fault,fw_pass,trap_seen,"
        "fetch_single_cnt,fetch_double_cnt,fetch_fatal_cnt,"
        "rf_single_err,rf_double_err,rf_fatal,result\n");

    // =====================================================
    // CATEGORY A — Fetch-stage fault injection
    // =====================================================

    // --- A0: Golden run (no faults) ---
    $display("\n--- A0: Golden run (no fetch faults) ---");
    do_reset;
    run_firmware(fw_ok, saw_trap);
    write_csv_row("A0", "none", "none",
                  fw_ok, saw_trap,
                  `RF.reg_file_single_err, `RF.reg_file_double_err, `RF.reg_file_fatal,
                  (fw_ok && !saw_trap) ? "PASS" : "FAIL");

    // --- A1: Single-bit fetch faults (9 cases) ---
    // Test 3 instruction word indices × 3 bit positions each
    begin : a1_block
        integer wi, bi;
        integer word_indices[0:9];
        integer bit_positions[0:9];
        reg [31:0] wi_val;
        reg [5:0]  bi_val;
                word_indices[0] = 0;  word_indices[1] = 1;  word_indices[2] = 2;
        word_indices[3] = 3;  word_indices[4] = 4;  word_indices[5] = 5;
        word_indices[6] = 6;  word_indices[7] = 8;  word_indices[8] = 16;
        bit_positions[0] = 0; bit_positions[1] = 22; bit_positions[2] = 44;
        bit_positions[3] = 5; bit_positions[4] = 11; bit_positions[5] = 32;
        bit_positions[6] = 38; bit_positions[7] = 43; bit_positions[8] = 1; bit_positions[9] = 2;

        for (wi = 0; wi < 9; wi = wi + 1) begin
            for (bi = 0; bi < 10; bi = bi + 1) begin
                $display("--- A1: single-bit word=%0d bit=%0d ---",
                         word_indices[wi], bit_positions[bi]);
                do_reset;
                wi_val = word_indices[wi];
                bi_val = bit_positions[bi];
                fi_fetch_en       = 1;
                fi_fetch_double   = 0;
                fi_fetch_word_idx = wi_val;
                fi_fetch_pos1     = bi_val;
                run_firmware(fw_ok, saw_trap);
                fi_fetch_en = 0;
                eval_fetch_correctable("A1", "fetch_single", fw_ok, saw_trap);
            end
        end
    end

    // --- A2: Double-bit fetch faults (6 cases) ---
    begin : a2_block
        integer wi, pi;
        integer word_indices[0:9];
        integer pair_p1[0:5];
        integer pair_p2[0:5];
        reg [31:0] wi_val;
                word_indices[0] = 0;  word_indices[1] = 1;  word_indices[2] = 2;
        word_indices[3] = 3;  word_indices[4] = 4;  word_indices[5] = 5;
        word_indices[6] = 6;  word_indices[7] = 8;  word_indices[8] = 16;
                pair_p1[0] = 0;  pair_p2[0] = 22;
        pair_p1[1] = 5;  pair_p2[1] = 38;
        pair_p1[2] = 1;  pair_p2[2] = 44;
        pair_p1[3] = 12; pair_p2[3] = 30;
        pair_p1[4] = 20; pair_p2[4] = 43;
        pair_p1[5] = 11; pair_p2[5] = 32;

        for (wi = 0; wi < 10; wi = wi + 1) begin
            for (pi = 0; pi < 6; pi = pi + 1) begin
                $display("--- A2: double-bit word=%0d bits=(%0d,%0d) ---",
                         word_indices[wi], pair_p1[pi], pair_p2[pi]);
                do_reset;
                wi_val = word_indices[wi];
                fi_fetch_en       = 1;
                fi_fetch_double   = 1;
                fi_fetch_word_idx = wi_val;
                fi_fetch_pos1     = pair_p1[pi];
                fi_fetch_pos2     = pair_p2[pi];
                run_firmware(fw_ok, saw_trap);
                fi_fetch_en     = 0;
                fi_fetch_double = 0;
                eval_fetch_correctable("A2", "fetch_double", fw_ok, saw_trap);
            end
        end
    end

    // --- A3: Triple-bit fetch faults (3 cases, expect fatal/trap) ---
    begin : a3_block
        integer ti;
        integer t_w[0:29];
        integer t_p1[0:29], t_p2[0:29];
        reg [31:0] tw_val;
                t_w[0] = 0; t_p1[0] = 0; t_p2[0] = 22;
        t_w[1] = 3; t_p1[1] = 3; t_p2[1] = 18;
        t_w[2] = 6; t_p1[2] = 7; t_p2[2] = 33;
        t_w[3] = 9; t_p1[3] = 0; t_p2[3] = 22;
        t_w[4] = 12; t_p1[4] = 3; t_p2[4] = 18;
        t_w[5] = 15; t_p1[5] = 7; t_p2[5] = 33;
        t_w[6] = 1; t_p1[6] = 0; t_p2[6] = 22;
        t_w[7] = 4; t_p1[7] = 3; t_p2[7] = 18;
        t_w[8] = 7; t_p1[8] = 7; t_p2[8] = 33;
        t_w[9] = 10; t_p1[9] = 0; t_p2[9] = 22;
        t_w[10] = 13; t_p1[10] = 3; t_p2[10] = 18;
        t_w[11] = 16; t_p1[11] = 7; t_p2[11] = 33;
        t_w[12] = 2; t_p1[12] = 0; t_p2[12] = 22;
        t_w[13] = 5; t_p1[13] = 3; t_p2[13] = 18;
        t_w[14] = 8; t_p1[14] = 7; t_p2[14] = 33;
        t_w[15] = 11; t_p1[15] = 0; t_p2[15] = 22;
        t_w[16] = 14; t_p1[16] = 3; t_p2[16] = 18;
        t_w[17] = 0; t_p1[17] = 7; t_p2[17] = 33;
        t_w[18] = 3; t_p1[18] = 0; t_p2[18] = 22;
        t_w[19] = 6; t_p1[19] = 3; t_p2[19] = 18;
        t_w[20] = 9; t_p1[20] = 7; t_p2[20] = 33;
        t_w[21] = 12; t_p1[21] = 0; t_p2[21] = 22;
        t_w[22] = 15; t_p1[22] = 3; t_p2[22] = 18;
        t_w[23] = 1; t_p1[23] = 7; t_p2[23] = 33;
        t_w[24] = 4; t_p1[24] = 0; t_p2[24] = 22;
        t_w[25] = 7; t_p1[25] = 3; t_p2[25] = 18;
        t_w[26] = 10; t_p1[26] = 7; t_p2[26] = 33;
        t_w[27] = 13; t_p1[27] = 0; t_p2[27] = 22;
        t_w[28] = 16; t_p1[28] = 3; t_p2[28] = 18;
        t_w[29] = 2; t_p1[29] = 7; t_p2[29] = 33;

        // Use fi_fetch_pos1 ^ fi_fetch_pos2 ^ extra hardcoded bit (44)
        for (ti = 0; ti < 30; ti = ti + 1) begin
            $display("--- A3: triple-bit word=%0d bits=(%0d,%0d,44) ---",
                     t_w[ti], t_p1[ti], t_p2[ti]);
            do_reset;
            tw_val = t_w[ti];
            fi_fetch_en       = 1;
            fi_fetch_double   = 1;       // pos1 + pos2 already = 2 bits
            fi_fetch_word_idx = tw_val;
            fi_fetch_pos1     = t_p1[ti];
            fi_fetch_pos2     = t_p2[ti];
            // Force a third flip directly into the stored codeword (bit 44)
            force dut.fetch_ecc_mem.instr_mem_bch[tw_val][44] =
                ~dut.fetch_ecc_mem.instr_mem_bch[tw_val][44];
            run_firmware(fw_ok, saw_trap);
            release dut.fetch_ecc_mem.instr_mem_bch[tw_val][44];
            fi_fetch_en     = 0;
            fi_fetch_double = 0;
            eval_fetch_fatal("A3", "fetch_triple", fw_ok, saw_trap);
        end
    end

    // =====================================================
    // CATEGORY B — Register-file fault injection
    // =====================================================

    // --- B0: Golden run (no register faults) ---
    $display("\n--- B0: Golden run (no register faults) ---");
    do_reset;
    run_firmware(fw_ok, saw_trap);
    write_csv_row("B0", "none", "none",
                  fw_ok, saw_trap,
                  `RF.reg_file_single_err, `RF.reg_file_double_err, `RF.reg_file_fatal,
                  (fw_ok && !saw_trap) ? "PASS" : "FAIL");

    // --- B1: Single-bit register faults (5 cases) ---
    begin : b1_block
        integer ri, bi;
        integer reg_addrs[0:24];
        integer bit_pos;
        reg [5:0] ra_val;
                reg_addrs[0] = 1;
        reg_addrs[1] = 2;
        reg_addrs[2] = 3;
        reg_addrs[3] = 4;
        reg_addrs[4] = 5;
        reg_addrs[5] = 6;
        reg_addrs[6] = 7;
        reg_addrs[7] = 8;
        reg_addrs[8] = 9;
        reg_addrs[9] = 10;
        reg_addrs[10] = 11;
        reg_addrs[11] = 12;
        reg_addrs[12] = 13;
        reg_addrs[13] = 14;
        reg_addrs[14] = 15;
        reg_addrs[15] = 16;
        reg_addrs[16] = 17;
        reg_addrs[17] = 18;
        reg_addrs[18] = 19;
        reg_addrs[19] = 20;
        reg_addrs[20] = 21;
        reg_addrs[21] = 22;
        reg_addrs[22] = 23;
        reg_addrs[23] = 24;
        reg_addrs[24] = 25;
  // x28 (t3)

        for (ri = 0; ri < 25; ri = ri + 1) begin
            for (bi = 0; bi < 2; bi = bi + 1) begin
                bit_pos = (bi == 0) ? 22 : 5;
            $display("--- B1: single-bit RF fault x%0d bit=22 ---", reg_addrs[ri]);
            do_reset;
            ra_val = reg_addrs[ri];
            force `RF.inject_reg_fault  = 1'b1;
            force `RF.inject_reg_double = 1'b0;
            force `RF.fault_reg_addr    = ra_val;
            force `RF.fault_reg_bit1    = bit_pos;
            force `RF.fault_reg_bit2    = 6'd0;
            run_firmware(fw_ok, saw_trap);
            release `RF.inject_reg_fault;
            release `RF.inject_reg_double;
            release `RF.fault_reg_addr;
            release `RF.fault_reg_bit1;
            release `RF.fault_reg_bit2;
            eval_rf_single("B1", "rf_single", fw_ok, saw_trap);
            end
        end
    end

    // --- B2: Double-bit register faults (5 cases) ---
    begin : b2_block
        integer ri;
        integer reg_addrs[0:24];
        reg [5:0] ra_val;
                reg_addrs[0] = 1;
        reg_addrs[1] = 2;
        reg_addrs[2] = 3;
        reg_addrs[3] = 4;
        reg_addrs[4] = 5;
        reg_addrs[5] = 6;
        reg_addrs[6] = 7;
        reg_addrs[7] = 8;
        reg_addrs[8] = 9;
        reg_addrs[9] = 10;
        reg_addrs[10] = 11;
        reg_addrs[11] = 12;
        reg_addrs[12] = 13;
        reg_addrs[13] = 14;
        reg_addrs[14] = 15;
        reg_addrs[15] = 16;
        reg_addrs[16] = 17;
        reg_addrs[17] = 18;
        reg_addrs[18] = 19;
        reg_addrs[19] = 20;
        reg_addrs[20] = 21;
        reg_addrs[21] = 22;
        reg_addrs[22] = 23;
        reg_addrs[23] = 24;
        reg_addrs[24] = 25;


        for (ri = 0; ri < 25; ri = ri + 1) begin
            for (bi = 0; bi < 2; bi = bi + 1) begin
                bit_pos = (bi == 0) ? 22 : 5;
            $display("--- B2: double-bit RF fault x%0d bits=(5,38) ---",
                     reg_addrs[ri]);
            do_reset;
            ra_val = reg_addrs[ri];
            force `RF.inject_reg_fault  = 1'b1;
            force `RF.inject_reg_double = 1'b1;
            force `RF.fault_reg_addr    = ra_val;
            force `RF.fault_reg_bit1    = (pi_idx == 0) ? 6'd5 : 6'd12;
            force `RF.fault_reg_bit2    = (pi_idx == 0) ? 6'd38 : 6'd30;
            run_firmware(fw_ok, saw_trap);
            release `RF.inject_reg_fault;
            release `RF.inject_reg_double;
            release `RF.fault_reg_addr;
            release `RF.fault_reg_bit1;
            release `RF.fault_reg_bit2;
            eval_rf_double("B2", "rf_double", fw_ok, saw_trap);
            end
        end
    end

    // --- B3: Triple-bit register faults (3 cases, expect fatal) ---
    begin : b3_block
        integer ri;
        integer reg_addrs[0:29];
        reg [5:0] ra_val;
                reg_addrs[0] = 1;
        reg_addrs[1] = 2;
        reg_addrs[2] = 3;
        reg_addrs[3] = 4;
        reg_addrs[4] = 5;
        reg_addrs[5] = 6;
        reg_addrs[6] = 7;
        reg_addrs[7] = 8;
        reg_addrs[8] = 9;
        reg_addrs[9] = 10;
        reg_addrs[10] = 11;
        reg_addrs[11] = 12;
        reg_addrs[12] = 13;
        reg_addrs[13] = 14;
        reg_addrs[14] = 15;
        reg_addrs[15] = 16;
        reg_addrs[16] = 17;
        reg_addrs[17] = 18;
        reg_addrs[18] = 19;
        reg_addrs[19] = 20;
        reg_addrs[20] = 21;
        reg_addrs[21] = 22;
        reg_addrs[22] = 23;
        reg_addrs[23] = 24;
        reg_addrs[24] = 25;
        reg_addrs[25] = 26;
        reg_addrs[26] = 27;
        reg_addrs[27] = 28;
        reg_addrs[28] = 29;
        reg_addrs[29] = 30;


        for (ri = 0; ri < 30; ri = ri + 1) begin
            $display("--- B3: triple-bit RF fault x%0d bits=(0,22,44) ---",
                     reg_addrs[ri]);
            do_reset;
            ra_val = reg_addrs[ri];
            // Inject two bits via the fault-injection interface;
            // inject a third by directly flipping a bit in the stored codeword
            force `RF.inject_reg_fault  = 1'b1;
            force `RF.inject_reg_double = 1'b1;
            force `RF.fault_reg_addr    = ra_val;
            force `RF.fault_reg_bit1    = 6'd0;
            force `RF.fault_reg_bit2    = 6'd22;
            // Flip bit 44 (the pad bit) of the register's stored codeword
            force `RF.regfile_bch[ra_val][44] = ~`RF.regfile_bch[ra_val][44];
            run_firmware(fw_ok, saw_trap);
            release `RF.inject_reg_fault;
            release `RF.inject_reg_double;
            release `RF.fault_reg_addr;
            release `RF.fault_reg_bit1;
            release `RF.fault_reg_bit2;
            release `RF.regfile_bch[ra_val][44];
            eval_rf_fatal("B3", "rf_triple", fw_ok, saw_trap);
        end
    end

    // =====================================================
    // CATEGORY C — Concurrent dual-layer faults
    // =====================================================

    // --- C1: Fetch single + RF single simultaneously ---
    $display("\n--- C1: Concurrent fetch-single + RF-single fault ---");
    begin : c1_loop
        integer ci;
        for (ci = 0; ci < 10; ci = ci + 1) begin
            do_reset;
            fi_fetch_en       = 1;
            fi_fetch_double   = 0;
            fi_fetch_word_idx = 4 + ci;
            fi_fetch_pos1     = 12;
            force `RF.inject_reg_fault  = 1'b1;
            force `RF.inject_reg_double = 1'b0;
            force `RF.fault_reg_addr    = 10 + ci;
            force `RF.fault_reg_bit1    = 15;
            force `RF.fault_reg_bit2    = 6'd0;
            run_firmware(fw_ok, saw_trap);
            fi_fetch_en = 0;
            release `RF.inject_reg_fault;
            release `RF.inject_reg_double;
            release `RF.fault_reg_addr;
            release `RF.fault_reg_bit1;
            release `RF.fault_reg_bit2;
            begin
                reg passed;
                passed = fw_ok && !saw_trap &&
                         (fetch_single_err_cnt > 0) && `RF.reg_file_single_err;
                write_csv_row("C1", "fetch_single", "rf_single",
                              fw_ok, saw_trap,
                              `RF.reg_file_single_err, `RF.reg_file_double_err,
                              `RF.reg_file_fatal,
                              passed ? "PASS" : "FAIL");
            end
        end
    end

    // --- C2: Fetch double + RF double simultaneously ---
    $display("\n--- C2: Concurrent fetch-double + RF-double fault ---");
    begin : c2_loop
        integer ci;
        for (ci = 0; ci < 10; ci = ci + 1) begin
            do_reset;
            fi_fetch_en       = 1;
            fi_fetch_double   = 1;
            fi_fetch_word_idx = 2 + ci;
            fi_fetch_pos1     = 3;
            fi_fetch_pos2     = 30;
            force `RF.inject_reg_fault  = 1'b1;
            force `RF.inject_reg_double = 1'b1;
            force `RF.fault_reg_addr    = 5 + ci;
            force `RF.fault_reg_bit1    = 6'd8;
            force `RF.fault_reg_bit2    = 6'd40;
            run_firmware(fw_ok, saw_trap);
            fi_fetch_en     = 0;
            fi_fetch_double = 0;
            release `RF.inject_reg_fault;
            release `RF.inject_reg_double;
            release `RF.fault_reg_addr;
            release `RF.fault_reg_bit1;
            release `RF.fault_reg_bit2;
            begin
                reg passed;
                passed = fw_ok && !saw_trap &&
                         (fetch_double_err_cnt > 0) && `RF.reg_file_double_err;
                write_csv_row("C2", "fetch_double", "rf_double",
                              fw_ok, saw_trap,
                              `RF.reg_file_single_err, `RF.reg_file_double_err,
                              `RF.reg_file_fatal,
                              passed ? "PASS" : "FAIL");
            end
        end
    end

    // =====================================================
    // Summary
    // =====================================================
    $display("");
    $display("========================================================");
    $display("Combined BCH Error-Resilient RISC-V CPU — Test Summary");
    $display("  Total tests : %0d", test_id);
    $display("  PASS        : %0d", total_pass);
    $display("  FAIL        : %0d", total_fail);
    $display("========================================================");

    $fclose(fcsv);
    $finish;
end

// Watchdog
initial begin
    #50000000;  // 50 ms at 1 ns timescale
    $display("[WATCHDOG] Simulation timeout — check firmware or memory config.");
    $fclose(fcsv);
    $finish;
end

endmodule
