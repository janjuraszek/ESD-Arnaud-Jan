module sobelCi #(
    parameter [7:0]  CUSTOM_ID = 8'd11,
    parameter integer MAX_WIDTH = 640,
    parameter [7:0]  THRESHOLD  = 8'd128
) (
    input  wire        clock,
    input  wire        reset,
    input  wire        start,
    input  wire [31:0] valueA,
    input  wire [31:0] valueB,
    input  wire [7:0]  ciN,
    output wire        done,
    output wire [31:0] result
);

    // =========================================================
    // Opcodes
    // =========================================================
    localparam ROW_ADDR_BITS = 10;  // 1024 entries >= 640

    wire selected = (ciN == CUSTOM_ID);
    wire fire     = start && selected;

    wire op = valueA[0];
    localparam OP_FEED  = 1'b0;
    localparam OP_RESET = 1'b1;

    // =========================================================
    // Position tracking
    // =========================================================
    reg [15:0] x_pos;
    reg [15:0] y_pos;

    // =========================================================
    // Line buffers
    // line_a stores even-numbered rows, line_b stores odd-numbered rows.
    // prev2_in_a = 1: line_a holds y-2 (to overwrite), line_b holds y-1
    // prev2_in_a = 0: line_b holds y-2 (to overwrite), line_a holds y-1
    // =========================================================
    wire prev2_in_a = ~y_pos[0];

    reg  [ROW_ADDR_BITS-1:0] la_addr, lb_addr;
    reg                      la_we,   lb_we;
    reg  [7:0]               la_din,  lb_din;
    wire [7:0]               la_dout, lb_dout;

    dualPortSSRAM #(.bitwidth(8), .nrOfEntries(1<<ROW_ADDR_BITS)) line_a (
        .clockA(clock),       .clockB(clock),
        .writeEnableA(la_we), .writeEnableB(1'b0),
        .addressA(la_addr),   .addressB({ROW_ADDR_BITS{1'b0}}),
        .dataInA(la_din),     .dataInB(8'b0),
        .dataOutA(la_dout),   .dataOutB());

    dualPortSSRAM #(.bitwidth(8), .nrOfEntries(1<<ROW_ADDR_BITS)) line_b (
        .clockA(clock),       .clockB(clock),
        .writeEnableA(lb_we), .writeEnableB(1'b0),
        .addressA(lb_addr),   .addressB({ROW_ADDR_BITS{1'b0}}),
        .dataInA(lb_din),     .dataInB(8'b0),
        .dataOutA(lb_dout),   .dataOutB());

    // =========================================================
    // Carry registers (current row only)
    // Hold the last 2 pixels fed in the current row.
    // These provide the left edge of the 3x3 window since the
    // current row is not yet stored in any line buffer.
    // =========================================================
    reg [7:0] carry_cur_0, carry_cur_1;

    // =========================================================
    // Pipeline registers
    // =========================================================
    reg        active;
    reg [3:0]  phase;
    reg [31:0] feed_word;

    // The 4 input pixels unpacked from feed_word
    wire [7:0] in_pix [0:3];
    assign in_pix[0] = feed_word[7:0];
    assign in_pix[1] = feed_word[15:8];
    assign in_pix[2] = feed_word[23:16];
    assign in_pix[3] = feed_word[31:24];

    // Storage for 6 BRAM reads per stored row:
    // cols x-2, x-1, x, x+1, x+2, x+3 from rows y-2 and y-1
    reg [7:0] rd_prev2 [0:5];
    reg [7:0] rd_prev1 [0:5];

    // =========================================================
    // Sobel magnitude function
    // Computes |Gx| + |Gy| for a 3x3 window.
    // Rows: a = y-2 (top), b = y-1 (mid), c = y (bottom/current)
    // =========================================================
    function automatic [11:0] sobel_mag;
        input [7:0] a0, a1, a2;
        input [7:0] b0, b1, b2;
        input [7:0] c0, c1, c2;
        reg signed [10:0] gx, gy;
        reg        [10:0] gx_abs, gy_abs;
        begin
            gx = - $signed({3'b0, a0}) + $signed({3'b0, a2})
                 - ($signed({3'b0, b0}) <<< 1) + ($signed({3'b0, b2}) <<< 1)
                 - $signed({3'b0, c0}) + $signed({3'b0, c2});
            gy =   $signed({3'b0, a0}) + ($signed({3'b0, a1}) <<< 1) + $signed({3'b0, a2})
                 - $signed({3'b0, c0}) - ($signed({3'b0, c1}) <<< 1) - $signed({3'b0, c2});
            gx_abs = gx[10] ? (~gx + 11'd1) : gx;
            gy_abs = gy[10] ? (~gy + 11'd1) : gy;
            sobel_mag = gx_abs + gy_abs;
        end
    endfunction

    // =========================================================
    // Result register
    // =========================================================
    reg [3:0] sobel_result;
    
	assign result = selected ? {28'b0, sobel_result} : 32'b0;
    // =========================================================
    // Done signal
    // =========================================================
    assign done = (active && phase == 4'd13)
               || (fire && op == OP_RESET);

    // =========================================================
    // Main FSM
    // =========================================================
    always @(posedge clock) begin
        if (reset) begin
            x_pos        <= 16'd0;
            y_pos        <= 16'd0;
            carry_cur_0  <= 8'd0;
            carry_cur_1  <= 8'd0;
            active       <= 1'b0;
            phase        <= 4'd0;
            la_we        <= 1'b0;
            lb_we        <= 1'b0;
            sobel_result <= 4'd0;
        end else begin
            la_we <= 1'b0;
            lb_we <= 1'b0;

            if (active) begin
                case (phase)

                // -------------------------------------------------
                // Phases 0-5: present read addresses for cols x-2..x+3
                // Both BRAMs addressed identically each cycle.
                // Results captured one cycle after address presentation.
                // Wraparound on x-2/x-1 at row start is harmless since
                // the C code ignores the leftmost output pixel of each row.
                // -------------------------------------------------
                4'd0: begin
                    la_addr <= (x_pos - 16'd2) & {ROW_ADDR_BITS{1'b1}};
                    lb_addr <= (x_pos - 16'd2) & {ROW_ADDR_BITS{1'b1}};
                    phase   <= 4'd1;
                end
                4'd1: begin
                    rd_prev2[0] <= prev2_in_a ? la_dout : lb_dout;
                    rd_prev1[0] <= prev2_in_a ? lb_dout : la_dout;
                    la_addr <= (x_pos - 16'd1) & {ROW_ADDR_BITS{1'b1}};
                    lb_addr <= (x_pos - 16'd1) & {ROW_ADDR_BITS{1'b1}};
                    phase   <= 4'd2;
                end
                4'd2: begin
                    rd_prev2[1] <= prev2_in_a ? la_dout : lb_dout;
                    rd_prev1[1] <= prev2_in_a ? lb_dout : la_dout;
                    la_addr <= x_pos[ROW_ADDR_BITS-1:0];
                    lb_addr <= x_pos[ROW_ADDR_BITS-1:0];
                    phase   <= 4'd3;
                end
                4'd3: begin
                    rd_prev2[2] <= prev2_in_a ? la_dout : lb_dout;
                    rd_prev1[2] <= prev2_in_a ? lb_dout : la_dout;
                    la_addr <= (x_pos + 16'd1) & {ROW_ADDR_BITS{1'b1}};
                    lb_addr <= (x_pos + 16'd1) & {ROW_ADDR_BITS{1'b1}};
                    phase   <= 4'd4;
                end
                4'd4: begin
                    rd_prev2[3] <= prev2_in_a ? la_dout : lb_dout;
                    rd_prev1[3] <= prev2_in_a ? lb_dout : la_dout;
                    la_addr <= (x_pos + 16'd2) & {ROW_ADDR_BITS{1'b1}};
                    lb_addr <= (x_pos + 16'd2) & {ROW_ADDR_BITS{1'b1}};
                    phase   <= 4'd5;
                end
                4'd5: begin
                    rd_prev2[4] <= prev2_in_a ? la_dout : lb_dout;
                    rd_prev1[4] <= prev2_in_a ? lb_dout : la_dout;
                    la_addr <= (x_pos + 16'd3) & {ROW_ADDR_BITS{1'b1}};
                    lb_addr <= (x_pos + 16'd3) & {ROW_ADDR_BITS{1'b1}};
                    phase   <= 4'd6;
                end
                4'd6: begin
                    rd_prev2[5] <= prev2_in_a ? la_dout : lb_dout;
                    rd_prev1[5] <= prev2_in_a ? lb_dout : la_dout;
                    phase <= 4'd7;
                end

                // -------------------------------------------------
                // Phase 7: compute 4 Sobel values
                // Outputs are for row y-1, cols x-1, x, x+1, x+2:
                //   out0 center (x-1): window cols x-2, x-1, x
                //   out1 center (x):   window cols x-1, x,   x+1
                //   out2 center (x+1): window cols x,   x+1, x+2
                //   out3 center (x+2): window cols x+1, x+2, x+3
                // -------------------------------------------------
                4'd7: begin
                    begin : compute
                        reg [11:0] m0, m1, m2, m3;
                        m0 = sobel_mag(
                            rd_prev2[0], rd_prev2[1], rd_prev2[2],
                            rd_prev1[0], rd_prev1[1], rd_prev1[2],
                            carry_cur_0, carry_cur_1, in_pix[0]);
                        m1 = sobel_mag(
                            rd_prev2[1], rd_prev2[2], rd_prev2[3],
                            rd_prev1[1], rd_prev1[2], rd_prev1[3],
                            carry_cur_1, in_pix[0],   in_pix[1]);
                        m2 = sobel_mag(
                            rd_prev2[2], rd_prev2[3], rd_prev2[4],
                            rd_prev1[2], rd_prev1[3], rd_prev1[4],
                            in_pix[0],   in_pix[1],   in_pix[2]);
                        m3 = sobel_mag(
                            rd_prev2[3], rd_prev2[4], rd_prev2[5],
                            rd_prev1[3], rd_prev1[4], rd_prev1[5],
                            in_pix[1],   in_pix[2],   in_pix[3]);
                        sobel_result <= {
                            (m3 > {4'b0, THRESHOLD}),
                            (m2 > {4'b0, THRESHOLD}),
                            (m1 > {4'b0, THRESHOLD}),
                            (m0 > {4'b0, THRESHOLD})};
                    end

                    carry_cur_0 <= in_pix[2];
                    carry_cur_1 <= in_pix[3];
                    phase <= 4'd8;
                end

                // -------------------------------------------------
                // Phases 8-11: write in_pix[0..3] into the old line
                // buffer, overwriting row y-2 with current row y.
                // -------------------------------------------------
                4'd8: begin
                    if (prev2_in_a) begin la_we <= 1'b1; la_addr <= x_pos[ROW_ADDR_BITS-1:0];       la_din <= in_pix[0]; end
                    else            begin lb_we <= 1'b1; lb_addr <= x_pos[ROW_ADDR_BITS-1:0];       lb_din <= in_pix[0]; end
                    phase <= 4'd9;
                end
                4'd9: begin
                    if (prev2_in_a) begin la_we <= 1'b1; la_addr <= x_pos[ROW_ADDR_BITS-1:0] + 1;  la_din <= in_pix[1]; end
                    else            begin lb_we <= 1'b1; lb_addr <= x_pos[ROW_ADDR_BITS-1:0] + 1;  lb_din <= in_pix[1]; end
                    phase <= 4'd10;
                end
                4'd10: begin
                    if (prev2_in_a) begin la_we <= 1'b1; la_addr <= x_pos[ROW_ADDR_BITS-1:0] + 2;  la_din <= in_pix[2]; end
                    else            begin lb_we <= 1'b1; lb_addr <= x_pos[ROW_ADDR_BITS-1:0] + 2;  lb_din <= in_pix[2]; end
                    phase <= 4'd11;
                end
                4'd11: begin
                    if (prev2_in_a) begin la_we <= 1'b1; la_addr <= x_pos[ROW_ADDR_BITS-1:0] + 3;  la_din <= in_pix[3]; end
                    else            begin lb_we <= 1'b1; lb_addr <= x_pos[ROW_ADDR_BITS-1:0] + 3;  lb_din <= in_pix[3]; end
                    phase <= 4'd12;
                end

                // -------------------------------------------------
                // Phase 12: advance position, reset carries at end of row
                // -------------------------------------------------
                4'd12: begin
                    if (x_pos + 16'd4 >= MAX_WIDTH) begin
                        x_pos       <= 16'd0;
                        y_pos       <= y_pos + 16'd1;
                        carry_cur_0 <= 8'd0;
                        carry_cur_1 <= 8'd0;
                    end else begin
                        x_pos <= x_pos + 16'd4;
                    end
                    phase <= 4'd13;
                end

                // -------------------------------------------------
                // Phase 13: done fires, return to idle
                // -------------------------------------------------
                4'd13: begin
                    active <= 1'b0;
                    phase  <= 4'd0;
                end

                default: phase <= 4'd0;
                endcase

            end else if (fire) begin
                case (op)
                    OP_FEED: begin
                        feed_word <= valueB;
                        active    <= 1'b1;
                        phase     <= 4'd0;
                    end
                    OP_RESET: begin
                        x_pos       <= 16'd0;
                        y_pos       <= 16'd0;
                        carry_cur_0 <= 8'd0;
                        carry_cur_1 <= 8'd0;
                        active      <= 1'b0;
                        phase       <= 4'd0;
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule
