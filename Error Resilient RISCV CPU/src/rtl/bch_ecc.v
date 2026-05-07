`timescale 1ns/1ps

module bch_encoder_final (
    input  [31:0] data_in,
    output reg [44:0] codeword
);
    integer i;
    reg [43:0] divreg;
    reg [11:0] parity;

    always @(*) begin
        divreg = {data_in, 12'b0};

        for (i = 43; i >= 12; i = i - 1) begin
            if (divreg[i]) begin
                divreg[i]    = 1'b0;
                divreg[i-2]  = divreg[i-2]  ^ 1'b1;
                divreg[i-4]  = divreg[i-4]  ^ 1'b1;
                divreg[i-7]  = divreg[i-7]  ^ 1'b1;
                divreg[i-8]  = divreg[i-8]  ^ 1'b1;
                divreg[i-9]  = divreg[i-9]  ^ 1'b1;
                divreg[i-12] = divreg[i-12] ^ 1'b1;
            end
        end

        parity   = divreg[11:0];
        codeword = {1'b0, data_in, parity};
    end
endmodule

module bch_encoder (
    input  [31:0] data_in,
    output [44:0] encoded_out
);
    bch_encoder_final encoder_i (
        .data_in(data_in),
        .codeword(encoded_out)
    );
endmodule

module bch_decoder_prod (
    input clk,
    input rst,
    input valid_in,
    input [44:0] data_in,
    output reg valid_out,
    output reg [31:0] data_out,
    output reg fatal,
    output reg [44:0] error_mask,
    output reg [5:0] sigma2
);
    function [5:0] gf_mul;
        input [5:0] a, b;
        reg [10:0] p;
        begin
            p = 0;
            if (b[0]) p = p ^ a;
            if (b[1]) p = p ^ (a << 1);
            if (b[2]) p = p ^ (a << 2);
            if (b[3]) p = p ^ (a << 3);
            if (b[4]) p = p ^ (a << 4);
            if (b[5]) p = p ^ (a << 5);

            gf_mul[0] = p[0] ^ p[6];
            gf_mul[1] = p[1] ^ p[6] ^ p[7];
            gf_mul[2] = p[2] ^ p[7] ^ p[8];
            gf_mul[3] = p[3] ^ p[8] ^ p[9];
            gf_mul[4] = p[4] ^ p[9] ^ p[10];
            gf_mul[5] = p[5] ^ p[10];
        end
    endfunction

    function [5:0] gf_inv;
        input [5:0] a;
        begin
            case(a)
                6'd0:  gf_inv = 6'd0;
                6'd1:  gf_inv = 6'd1;
                6'd2:  gf_inv = 6'd33;
                6'd3:  gf_inv = 6'd62;
                6'd4:  gf_inv = 6'd49;
                6'd5:  gf_inv = 6'd43;
                6'd6:  gf_inv = 6'd31;
                6'd7:  gf_inv = 6'd44;
                6'd8:  gf_inv = 6'd57;
                6'd9:  gf_inv = 6'd37;
                6'd10: gf_inv = 6'd52;
                6'd11: gf_inv = 6'd28;
                6'd12: gf_inv = 6'd46;
                6'd13: gf_inv = 6'd40;
                6'd14: gf_inv = 6'd22;
                6'd15: gf_inv = 6'd25;
                6'd16: gf_inv = 6'd61;
                6'd17: gf_inv = 6'd54;
                6'd18: gf_inv = 6'd51;
                6'd19: gf_inv = 6'd39;
                6'd20: gf_inv = 6'd26;
                6'd21: gf_inv = 6'd35;
                6'd22: gf_inv = 6'd14;
                6'd23: gf_inv = 6'd24;
                6'd24: gf_inv = 6'd23;
                6'd25: gf_inv = 6'd15;
                6'd26: gf_inv = 6'd20;
                6'd27: gf_inv = 6'd34;
                6'd28: gf_inv = 6'd11;
                6'd29: gf_inv = 6'd53;
                6'd30: gf_inv = 6'd45;
                6'd31: gf_inv = 6'd6;
                6'd32: gf_inv = 6'd63;
                6'd33: gf_inv = 6'd2;
                6'd34: gf_inv = 6'd27;
                6'd35: gf_inv = 6'd21;
                6'd36: gf_inv = 6'd56;
                6'd37: gf_inv = 6'd9;
                6'd38: gf_inv = 6'd50;
                6'd39: gf_inv = 6'd19;
                6'd40: gf_inv = 6'd13;
                6'd41: gf_inv = 6'd47;
                6'd42: gf_inv = 6'd48;
                6'd43: gf_inv = 6'd5;
                6'd44: gf_inv = 6'd7;
                6'd45: gf_inv = 6'd30;
                6'd46: gf_inv = 6'd12;
                6'd47: gf_inv = 6'd41;
                6'd48: gf_inv = 6'd42;
                6'd49: gf_inv = 6'd4;
                6'd50: gf_inv = 6'd38;
                6'd51: gf_inv = 6'd18;
                6'd52: gf_inv = 6'd10;
                6'd53: gf_inv = 6'd29;
                6'd54: gf_inv = 6'd17;
                6'd55: gf_inv = 6'd60;
                6'd56: gf_inv = 6'd36;
                6'd57: gf_inv = 6'd8;
                6'd58: gf_inv = 6'd59;
                6'd59: gf_inv = 6'd58;
                6'd60: gf_inv = 6'd55;
                6'd61: gf_inv = 6'd16;
                6'd62: gf_inv = 6'd3;
                6'd63: gf_inv = 6'd32;
                default: gf_inv = 6'd0;
            endcase
        end
    endfunction

    function [5:0] alpha_b;
        input [6:0] e;
        begin
            case(e)
                7'd0:  alpha_b = 6'd1;
                7'd1:  alpha_b = 6'd2;
                7'd2:  alpha_b = 6'd4;
                7'd3:  alpha_b = 6'd8;
                7'd4:  alpha_b = 6'd16;
                7'd5:  alpha_b = 6'd32;
                7'd6:  alpha_b = 6'd3;
                7'd7:  alpha_b = 6'd6;
                7'd8:  alpha_b = 6'd12;
                7'd9:  alpha_b = 6'd24;
                7'd10: alpha_b = 6'd48;
                7'd11: alpha_b = 6'd35;
                7'd12: alpha_b = 6'd5;
                7'd13: alpha_b = 6'd10;
                7'd14: alpha_b = 6'd20;
                7'd15: alpha_b = 6'd40;
                7'd16: alpha_b = 6'd19;
                7'd17: alpha_b = 6'd38;
                7'd18: alpha_b = 6'd15;
                7'd19: alpha_b = 6'd30;
                7'd20: alpha_b = 6'd60;
                7'd21: alpha_b = 6'd59;
                7'd22: alpha_b = 6'd53;
                7'd23: alpha_b = 6'd41;
                7'd24: alpha_b = 6'd17;
                7'd25: alpha_b = 6'd34;
                7'd26: alpha_b = 6'd7;
                7'd27: alpha_b = 6'd14;
                7'd28: alpha_b = 6'd28;
                7'd29: alpha_b = 6'd56;
                7'd30: alpha_b = 6'd51;
                7'd31: alpha_b = 6'd37;
                7'd32: alpha_b = 6'd9;
                7'd33: alpha_b = 6'd18;
                7'd34: alpha_b = 6'd36;
                7'd35: alpha_b = 6'd11;
                7'd36: alpha_b = 6'd22;
                7'd37: alpha_b = 6'd44;
                7'd38: alpha_b = 6'd27;
                7'd39: alpha_b = 6'd54;
                7'd40: alpha_b = 6'd47;
                7'd41: alpha_b = 6'd29;
                7'd42: alpha_b = 6'd58;
                7'd43: alpha_b = 6'd55;
                7'd44: alpha_b = 6'd45;
                7'd45: alpha_b = 6'd25;
                7'd46: alpha_b = 6'd50;
                7'd47: alpha_b = 6'd39;
                7'd48: alpha_b = 6'd13;
                7'd49: alpha_b = 6'd26;
                7'd50: alpha_b = 6'd52;
                7'd51: alpha_b = 6'd43;
                7'd52: alpha_b = 6'd21;
                7'd53: alpha_b = 6'd42;
                7'd54: alpha_b = 6'd23;
                7'd55: alpha_b = 6'd46;
                7'd56: alpha_b = 6'd31;
                7'd57: alpha_b = 6'd62;
                7'd58: alpha_b = 6'd63;
                7'd59: alpha_b = 6'd61;
                7'd60: alpha_b = 6'd57;
                7'd61: alpha_b = 6'd49;
                7'd62: alpha_b = 6'd33;
                default: alpha_b = 6'd0;
            endcase
        end
    endfunction

    function [6:0] mod63_mul3;
        input [5:0] i6;
        reg [7:0] t;
        begin
            t = {2'b00, i6} + {1'b0, i6, 1'b0};
            if (t >= 8'd126)
                mod63_mul3 = t - 8'd126;
            else if (t >= 8'd63)
                mod63_mul3 = t - 8'd63;
            else
                mod63_mul3 = t[6:0];
        end
    endfunction

    function [6:0] mod63_neg;
        input [5:0] k6;
        begin
            if (k6 == 6'd0)
                mod63_neg = 7'd0;
            else
                mod63_neg = 7'd63 - {1'b0, k6};
        end
    endfunction

    function [6:0] mod63_mul2;
        input [6:0] x;
        reg [7:0] t;
        begin
            t = {1'b0, x, 1'b0};
            if (t >= 8'd63)
                mod63_mul2 = t - 8'd63;
            else
                mod63_mul2 = t[6:0];
        end
    endfunction

    reg [5:0] S1, S3;
    reg [44:0] d1;
    reg v1;

    integer i;
    reg [5:0] next_S1, next_S3;
    reg [6:0] exp_s1, exp_s3;

    always @(*) begin
        next_S1 = 6'd0;
        next_S3 = 6'd0;

        for (i = 0; i < 45; i = i + 1) begin
            if (data_in[i]) begin
                exp_s1 = i[6:0];
                exp_s3 = mod63_mul3(i[5:0]);
                next_S1 = next_S1 ^ alpha_b(exp_s1);
                next_S3 = next_S3 ^ alpha_b(exp_s3);
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            S1 <= 6'd0;
            S3 <= 6'd0;
            d1 <= 45'd0;
            v1 <= 1'b0;
        end else begin
            S1 <= next_S1;
            S3 <= next_S3;
            d1 <= data_in;
            v1 <= valid_in;
        end
    end

    reg [5:0] s1, s2;
    reg [44:0] d2;
    reg v2;

    always @(posedge clk) begin
        if (rst) begin
            s1 <= 6'd0;
            s2 <= 6'd0;
            d2 <= 45'd0;
            v2 <= 1'b0;
        end else begin
            s1 <= S1;
            if (S1 == 6'd0)
                s2 <= 6'd0;
            else
                s2 <= gf_mul(gf_mul(gf_mul(S1, S1), S1) ^ S3, gf_inv(S1));
            d2 <= d1;
            v2 <= v1;
        end
    end

    reg [44:0] mask;
    integer k;
    reg [6:0] chien_e1, chien_e2;
    wire [44:0] corrected_cw = d2 ^ mask;

    always @(*) begin
        mask = 45'd0;

        for (k = 0; k < 45; k = k + 1) begin
            chien_e1 = mod63_neg(k[5:0]);
            chien_e2 = mod63_mul2(chien_e1);

            if ((6'd1 ^
                 gf_mul(s1, alpha_b(chien_e1)) ^
                 gf_mul(s2, alpha_b(chien_e2))) == 6'd0)
                mask[k] = 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            valid_out  <= 1'b0;
            data_out   <= 32'd0;
            fatal      <= 1'b0;
            error_mask <= 45'd0;
            sigma2     <= 6'd0;
        end else begin
            valid_out  <= v2;
            data_out   <= corrected_cw[43:12];
            fatal      <= (s1 != 6'd0) && (mask == 45'd0);
            error_mask <= mask;
            sigma2     <= s2;
        end
    end
endmodule
