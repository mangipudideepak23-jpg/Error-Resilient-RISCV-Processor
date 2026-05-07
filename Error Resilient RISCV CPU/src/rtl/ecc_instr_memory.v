// =============================================================
// ecc_instr_memory.v
// (Friend's work — BCH(45,32,t=2) protection at the fetch stage)
//
// BCH(45,32,t=2) protected Instruction + Data Memory.
// Uses PicoRV32's native mem_valid/mem_ready/mem_instr interface.
//
// On instruction fetch (mem_instr=1):
//   - Reads BCH-encoded codeword from instr_mem_bch[]
//   - Applies optional fault-injection mask (for testbench use)
//   - Passes the (possibly corrupted) codeword to bch_decoder_prod
//     (defined in bch_ecc.v — 3-cycle pipeline)
//   - Asserts mem_ready ONLY after decoding completes
//   - Returns the corrected 32-bit instruction
//
// On data access (mem_instr=0):
//   - Reads/writes a plain raw memory (no BCH overhead)
//   - Asserts mem_ready in the next cycle
//
// Firmware is loaded from firmware/firmware.hex at elaboration.
// All instructions are BCH-encoded into instr_mem_bch[] at init time.
//
// Fault injection ports (driven by testbench):
//   fi_en        - enable fault injection
//   fi_double    - inject a second bit-flip (double-bit fault)
//   fi_word_idx  - which instruction word (word address) to corrupt
//   fi_pos1      - first bit position in 45-bit codeword to flip
//   fi_pos2      - second bit position (used when fi_double=1)
//
// ECC status outputs:
//   bch_fatal         - uncorrectable error flag (level, not sticky)
//   single_err_cnt    - count of corrected single-bit errors
//   double_err_cnt    - count of corrected double-bit errors
//   fatal_err_cnt     - count of uncorrectable error events
// =============================================================

`timescale 1ns/1ps

module ecc_instr_memory #(
    parameter MEM_SIZE = 128*1024   // bytes
) (
    input         clk,
    input         resetn,

    // Native PicoRV32 memory interface
    input         mem_valid,
    input         mem_instr,
    output reg    mem_ready,
    input  [31:0] mem_addr,
    input  [31:0] mem_wdata,
    input  [ 3:0] mem_wstrb,
    output reg [31:0] mem_rdata,

    // BCH status
    output        bch_fatal,

    // Fault injection (testbench use only)
    input         fi_en,
    input         fi_double,
    input  [31:0] fi_word_idx,
    input  [ 5:0] fi_pos1,
    input  [ 5:0] fi_pos2,

    // Error counters
    output reg [31:0] single_err_cnt,
    output reg [31:0] double_err_cnt,
    output reg [31:0] fatal_err_cnt
);

    localparam WORDS = MEM_SIZE / 4;

    // -------------------------------------------------------
    // Memory arrays
    // -------------------------------------------------------
    reg [31:0] memory       [0:WORDS-1]; // raw memory (data accesses)
    reg [44:0] instr_mem_bch[0:WORDS-1]; // BCH-encoded instruction memory

    integer i;

    // -------------------------------------------------------
    // BCH encode function — identical polynomial to bch_ecc.v
    // g(x) = x^12 + x^9 + x^8 + x^7 + x^4 + x^2 + 1
    // Codeword layout: {1'b0, data[31:0], parity[11:0]}
    // -------------------------------------------------------
    function [44:0] bch_encode_func;
        input [31:0] data_in;
        integer j;
        reg [43:0] divreg;
        reg [11:0] parity;
        begin
            divreg = {data_in, 12'b0};
            for (j = 43; j >= 12; j = j - 1) begin
                if (divreg[j]) begin
                    divreg[j]    = 1'b0;
                    divreg[j-2]  = divreg[j-2]  ^ 1'b1;
                    divreg[j-4]  = divreg[j-4]  ^ 1'b1;
                    divreg[j-7]  = divreg[j-7]  ^ 1'b1;
                    divreg[j-8]  = divreg[j-8]  ^ 1'b1;
                    divreg[j-9]  = divreg[j-9]  ^ 1'b1;
                    divreg[j-12] = divreg[j-12] ^ 1'b1;
                end
            end
            parity = divreg[11:0];
            bch_encode_func = {1'b0, data_in, parity};
        end
    endfunction

    // -------------------------------------------------------
    // Initialisation: load firmware, BCH-encode all words
    // -------------------------------------------------------
    initial begin
        for (i = 0; i < WORDS; i = i + 1)
            memory[i] = 32'h00000013;           // NOP

        $readmemh("firmware/firmware.hex", memory);

        for (i = 0; i < WORDS; i = i + 1)
            instr_mem_bch[i] = bch_encode_func(memory[i]);

        single_err_cnt = 32'd0;
        double_err_cnt = 32'd0;
        fatal_err_cnt  = 32'd0;
    end

    // -------------------------------------------------------
    // Fault-injection mask computation (combinational)
    // -------------------------------------------------------
    wire [44:0] fi_mask1 = (fi_pos1 < 6'd45) ? (45'd1 << fi_pos1) : 45'd0;
    wire [44:0] fi_mask2 = (fi_pos2 < 6'd45) ? (45'd1 << fi_pos2) : 45'd0;

    // -------------------------------------------------------
    // BCH Decoder wires
    // (bch_decoder_prod is defined in bch_ecc.v)
    // -------------------------------------------------------
    reg         bch_valid_in;
    reg  [44:0] bch_data_in;
    wire        bch_valid_out;
    wire [31:0] bch_data_out;
    wire        bch_fatal_wire;
    wire [44:0] bch_error_mask;
    wire [ 5:0] bch_sigma2;

    bch_decoder_prod bch_dec (
        .clk        (clk),
        .rst        (!resetn),
        .valid_in   (bch_valid_in),
        .data_in    (bch_data_in),
        .valid_out  (bch_valid_out),
        .data_out   (bch_data_out),
        .fatal      (bch_fatal_wire),
        .error_mask (bch_error_mask),
        .sigma2     (bch_sigma2)
    );

    assign bch_fatal = bch_fatal_wire;

    // -------------------------------------------------------
    // 5-state FSM
    //   S_IDLE       : waiting for mem_valid
    //   S_INSTR_DEC  : instruction fetch — waiting for BCH decoder
    //   S_DATA_RESP  : data read — respond next cycle
    //   S_WRITE_DONE : write — update memory then respond
    //   S_READY_HOLD : deassert mem_ready before returning to IDLE
    // -------------------------------------------------------
    localparam S_IDLE       = 3'd0;
    localparam S_INSTR_DEC  = 3'd1;
    localparam S_DATA_RESP  = 3'd2;
    localparam S_WRITE_DONE = 3'd3;
    localparam S_READY_HOLD = 3'd4;

    reg [2:0]  state;
    reg [31:0] latched_addr;
    reg [31:0] latched_wdata;
    reg [ 3:0] latched_wstrb;
    reg        bch_pending;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state         <= S_IDLE;
            mem_ready     <= 1'b0;
            mem_rdata     <= 32'd0;
            bch_valid_in  <= 1'b0;
            bch_data_in   <= 45'd0;
            bch_pending   <= 1'b0;
            latched_addr  <= 32'd0;
            latched_wdata <= 32'd0;
            latched_wstrb <= 4'd0;
            single_err_cnt <= 32'd0;
            double_err_cnt <= 32'd0;
            fatal_err_cnt  <= 32'd0;
        end else begin
            mem_ready    <= 1'b0;
            bch_valid_in <= 1'b0;

            case (state)

                // -----------------------------------------------
                S_IDLE: begin
                    if (mem_valid) begin
                        latched_addr  <= mem_addr;
                        latched_wdata <= mem_wdata;
                        latched_wstrb <= mem_wstrb;

                        if (mem_addr >= MEM_SIZE) begin
                            // Out-of-bounds: return NOP immediately
                            $display("[ECC_MEM] OOB access addr=%08x", mem_addr);
                            mem_rdata <= 32'h0000_0013;
                            mem_ready <= 1'b1;
                            // stay in IDLE

                        end else if (|mem_wstrb) begin
                            // Write: update raw + BCH arrays next cycle
                            state <= S_WRITE_DONE;

                        end else if (mem_instr) begin
                            // Instruction fetch: read codeword, apply fault mask, kick decoder
                            bch_data_in  <= (fi_en && ((mem_addr >> 2) == fi_word_idx))
                                            ? (instr_mem_bch[mem_addr >> 2] ^ fi_mask1
                                               ^ (fi_double ? fi_mask2 : 45'd0))
                                            : instr_mem_bch[mem_addr >> 2];
                            bch_valid_in <= 1'b1;
                            bch_pending  <= 1'b1;
                            state        <= S_INSTR_DEC;

                        end else begin
                            // Data read: return raw word next cycle
                            mem_rdata <= memory[mem_addr >> 2];
                            state     <= S_DATA_RESP;
                        end
                    end
                end

                // -----------------------------------------------
                S_INSTR_DEC: begin
                    if (bch_valid_out && bch_pending) begin
                        bch_pending <= 1'b0;
                        mem_rdata   <= bch_data_out;
                        mem_ready   <= 1'b1;

                        // Update error counters
                        if (bch_fatal_wire) begin
                            fatal_err_cnt <= fatal_err_cnt + 1'b1;
                            $display("[BCH FETCH] FATAL uncorrectable error at addr=%08x",
                                     latched_addr);
                        end else if (bch_error_mask != 45'd0) begin
                            if (bch_sigma2 == 6'd0) begin
                                single_err_cnt <= single_err_cnt + 1'b1;
                                $display("[BCH FETCH] Corrected 1-bit error at addr=%08x",
                                         latched_addr);
                            end else begin
                                double_err_cnt <= double_err_cnt + 1'b1;
                                $display("[BCH FETCH] Corrected 2-bit error at addr=%08x",
                                         latched_addr);
                            end
                        end

                        state <= S_READY_HOLD;
                    end
                end

                // -----------------------------------------------
                S_READY_HOLD: begin
                    mem_ready <= 1'b0;
                    state     <= S_IDLE;
                end

                // -----------------------------------------------
                S_DATA_RESP: begin
                    mem_ready <= 1'b1;
                    state     <= S_READY_HOLD;
                end

                // -----------------------------------------------
                S_WRITE_DONE: begin
                    // Build the updated 32-bit word with byte-lane masking,
                    // then write both the raw array and the re-encoded BCH copy.
                    // Using blocking reads of memory[] here so the new value is
                    // passed to bch_encode_func in the same delta cycle.
                    begin : write_word
                        reg [31:0] new_word;
                        new_word = memory[latched_addr >> 2];
                        if (latched_wstrb[0]) new_word[ 7: 0] = latched_wdata[ 7: 0];
                        if (latched_wstrb[1]) new_word[15: 8] = latched_wdata[15: 8];
                        if (latched_wstrb[2]) new_word[23:16] = latched_wdata[23:16];
                        if (latched_wstrb[3]) new_word[31:24] = latched_wdata[31:24];
                        memory       [latched_addr >> 2] <= new_word;
                        instr_mem_bch[latched_addr >> 2] <= bch_encode_func(new_word);
                    end

                    mem_ready <= 1'b1;
                    state     <= S_READY_HOLD;
                end

            endcase
        end
    end

endmodule
