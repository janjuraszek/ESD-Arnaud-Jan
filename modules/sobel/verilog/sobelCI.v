/* sobelCi.v - BRAM-friendly streaming Sobel custom instruction (CI ID = 11)
 *
 * Uses dualPortSSRAM for all line buffers and the output buffer.
 * Reads are registered (1-cycle latency); pipeline is structured accordingly.
 *
 * Operations (valueA[2:0] = op):
 *   OP=0 FEED      : feed 4 packed grayscale pixels (valueB = {p3,p2,p1,p0}).
 *                    Takes 8 cycles. done asserts on last cycle.
 *   OP=1 READ_OUT  : read 4 packed sobel bytes. valueB = word index [0..159].
 *                    Takes 2 cycles (BRAM read latency). done asserts cycle 2.
 *   OP=2 STATUS    : read status. bit[0]=out_row_ready.
 *   OP=3 RESET     : reset streaming state for a new frame.
 *   OP=4 SET_THR   : set threshold = valueB[7:0].
 *   OP=5 SET_WIDTH : set frame width in pixels = valueB[15:0]. Multiple of 4.
 *   OP=6 ACK_ROW   : clear out_row_ready.
 */

module sobelCi #(
    parameter [7:0] CUSTOM_ID = 8'd11,
    parameter integer MAX_WIDTH = 640
) (
    input  wire        clock,
    input  wire        reset,         // ACTIVE HIGH, like other CIs
    input  wire        start,
    input  wire [31:0] valueA,
    input  wire [31:0] valueB,
    input  wire [7:0]  ciN,
    output wire        done,
    output wire [31:0] result
);

    localparam ROW_ADDR_BITS = 10;            // 1024 covers 640
    localparam OUT_WORDS     = MAX_WIDTH / 4; // 160
    localparam OUT_ADDR_BITS = 8;             // 256 covers 160

    wire selected = (ciN == CUSTOM_ID);
    wire fire     = start && selected;

    wire [2:0] op = valueA[2:0];
    localparam OP_FEED      = 3'd0;
    localparam OP_READ_OUT  = 3'd1;
    localparam OP_STATUS    = 3'd2;
    localparam OP_RESET     = 3'd3;
    localparam OP_SET_THR   = 3'd4;
    localparam OP_SET_WIDTH = 3'd5;
    localparam OP_ACK_ROW   = 3'd6;

    // ---- Config ----
    reg [7:0]  threshold_r;
    reg [15:0] width_r;

    // ---- Streaming state ----
    reg [15:0] x_pos;       // input column 0..width-1
    reg [15:0] y_pos;       // input row
    reg        out_row_ready;

    // ---- FEED state machine ----
    // For each of 4 input pixels, we run a 2-cycle micro-sequence:
    //   phase 0: present read addr x_pos to both line buffers
    //   phase 1: capture BRAM read data, shift registers, write new pixel,
    //            compute sobel, write to out_buf, advance x_pos
    // feed_pix in {0..3}, feed_phase in {0..1}
    reg       feed_active;
    reg [1:0] feed_pix;
    reg       feed_phase;
    reg [31:0] feed_word;

    // The pixel currently being processed
    wire [7:0] in_pix = (feed_pix == 2'd0) ? feed_word[7:0]
                     : (feed_pix == 2'd1) ? feed_word[15:8]
                     : (feed_pix == 2'd2) ? feed_word[23:16]
                                           : feed_word[31:24];

    // Which buffer holds row y-2 (the one we read OLD data from then overwrite)?
    wire prev2_in_a = ~y_pos[0];

    // ---- 3-column shift registers (the 3x3 window) ----
    reg [7:0] prev2_p0, prev2_p1, prev2_p2;
    reg [7:0] prev1_p0, prev1_p1, prev1_p2;
    reg [7:0] cur_p0,   cur_p1,   cur_p2;

    // ---- Line buffer BRAMs ----
    // Port A: used for the streaming pipeline (read + write at x_pos)
    // Port B: unused for now (could be used for parallel readout later)
    reg [ROW_ADDR_BITS-1:0] la_addrA, lb_addrA;
    reg                     la_weA,   lb_weA;
    reg [7:0]               la_dinA,  lb_dinA;
    wire [7:0]              la_doutA, lb_doutA;

    dualPortSSRAM #(.bitwidth(8), .nrOfEntries(1<<ROW_ADDR_BITS)) line_a (
        .clockA(clock), .clockB(clock),
        .writeEnableA(la_weA), .writeEnableB(1'b0),
        .addressA(la_addrA),   .addressB({ROW_ADDR_BITS{1'b0}}),
        .dataInA(la_dinA),     .dataInB(8'b0),
        .dataOutA(la_doutA),   .dataOutB());

    dualPortSSRAM #(.bitwidth(8), .nrOfEntries(1<<ROW_ADDR_BITS)) line_b (
        .clockA(clock), .clockB(clock),
        .writeEnableA(lb_weA), .writeEnableB(1'b0),
        .addressA(lb_addrA),   .addressB({ROW_ADDR_BITS{1'b0}}),
        .dataInA(lb_dinA),     .dataInB(8'b0),
        .dataOutA(lb_doutA),   .dataOutB());

    // ---- Output buffer BRAM (32-bit wide, word-addressed) ----
    // Port A: write side from streaming pipeline (we accumulate 4 bytes into
    //         a 32-bit word in a small staging register, then write).
    // Port B: read side for host READ_OUT.
    reg [OUT_ADDR_BITS-1:0]  ob_addrA;
    reg                      ob_weA;
    reg [31:0]               ob_dinA;
    wire [31:0]              ob_doutA;       // unused on write port

    reg [OUT_ADDR_BITS-1:0]  ob_addrB;
    wire [31:0]              ob_doutB;

    dualPortSSRAM #(.bitwidth(32), .nrOfEntries(1<<OUT_ADDR_BITS)) out_mem (
        .clockA(clock), .clockB(clock),
        .writeEnableA(ob_weA), .writeEnableB(1'b0),
        .addressA(ob_addrA),   .addressB(ob_addrB),
        .dataInA(ob_dinA),     .dataInB(32'b0),
        .dataOutA(ob_doutA),   .dataOutB(ob_doutB));

    // ---- Staging register: pack 4 sobel bytes into one 32-bit word ----
    reg [7:0] out_b0, out_b1, out_b2;       // bytes 0..2 of the word being assembled

    // ---- Sobel arithmetic ----
    wire signed [10:0] gx =
        - $signed({3'b0, prev2_p0}) + $signed({3'b0, prev2_p2})
        - ($signed({3'b0, prev1_p0}) <<< 1) + ($signed({3'b0, prev1_p2}) <<< 1)
        - $signed({3'b0, cur_p0})   + $signed({3'b0, cur_p2});

    wire signed [10:0] gy =
        $signed({3'b0, prev2_p0}) + ($signed({3'b0, prev2_p1}) <<< 1) + $signed({3'b0, prev2_p2})
      - $signed({3'b0, cur_p0})   - ($signed({3'b0, cur_p1})   <<< 1) - $signed({3'b0, cur_p2});

    wire [10:0] gx_abs = gx[10] ? (~gx + 11'd1) : gx;
    wire [10:0] gy_abs = gy[10] ? (~gy + 11'd1) : gy;
    wire [11:0] mag    = gx_abs + gy_abs;
    wire [7:0]  sobel_pix = (mag > {4'b0, threshold_r}) ? 8'hFF : 8'h00;

    // ---- READ_OUT sequencing ----
    // Phase 0: present address, no data yet.
    // Phase 1: data appears in ob_doutB, return it, assert done.
    reg       read_active;
    reg       read_phase;
    reg [OUT_ADDR_BITS-1:0] read_addr_latched;

    // ---- Result mux ----
    reg [31:0] result_r;
    always @(*) begin
        case (op)
            OP_STATUS:   result_r = {31'b0, out_row_ready};
            OP_READ_OUT: result_r = ob_doutB;
            default:     result_r = 32'b0;
        endcase
    end
    assign result = result_r;

    // ---- done signal ----
    //   Non-multicycle ops (RESET, SET_THR, SET_WIDTH, ACK_ROW, STATUS): done same cycle.
    //   FEED: done on last cycle (pixel 3, phase 1).
    //   READ_OUT: done on phase 1.
    wire feed_last = feed_active && (feed_pix == 2'd3) && (feed_phase == 1'b1);
    wire read_last = read_active && (read_phase == 1'b1);
    wire simple_op = fire && (op != OP_FEED) && (op != OP_READ_OUT);

    assign done = simple_op | feed_last | read_last;

    // ---- Main FSM ----
    integer i;
    always @(posedge clock) begin
        if (reset) begin
            threshold_r   <= 8'd128;
            width_r       <= 16'd640;
            x_pos         <= 16'd0;
            y_pos         <= 16'd0;
            out_row_ready <= 1'b0;
            feed_active   <= 1'b0;
            feed_pix      <= 2'd0;
            feed_phase    <= 1'b0;
            read_active   <= 1'b0;
            read_phase    <= 1'b0;
            prev2_p0 <= 0; prev2_p1 <= 0; prev2_p2 <= 0;
            prev1_p0 <= 0; prev1_p1 <= 0; prev1_p2 <= 0;
            cur_p0   <= 0; cur_p1   <= 0; cur_p2   <= 0;
            out_b0 <= 0; out_b1 <= 0; out_b2 <= 0;
            la_weA <= 1'b0; lb_weA <= 1'b0;
            ob_weA <= 1'b0;
        end else begin
            // Default: deassert all writes; will be re-asserted by FSMs below
            la_weA <= 1'b0;
            lb_weA <= 1'b0;
            ob_weA <= 1'b0;

            // ============= FEED state machine =============
            if (feed_active) begin
                if (feed_phase == 1'b0) begin
                    // Phase 0: just present read addresses (they're already
                    // assigned combinationally below). Advance to phase 1.
                    feed_phase <= 1'b1;
                end else begin
                    // Phase 1: BRAM read data is now valid on doutA of both banks.
                    // - prev2 row's pixel at x_pos: from whichever bank holds row y-2
                    // - prev1 row's pixel at x_pos: from the OTHER bank
                    // Shift the column registers and add the new column.
                    prev2_p0 <= prev2_p1;
                    prev2_p1 <= prev2_p2;
                    prev2_p2 <= prev2_in_a ? la_doutA : lb_doutA;

                    prev1_p0 <= prev1_p1;
                    prev1_p1 <= prev1_p2;
                    prev1_p2 <= prev2_in_a ? lb_doutA : la_doutA;

                    cur_p0 <= cur_p1;
                    cur_p1 <= cur_p2;
                    cur_p2 <= in_pix;

                    // Overwrite "row y-2"'s buffer (which becomes the storage
                    // for the current row y) at x_pos with the new pixel.
                    if (prev2_in_a) begin
                        la_weA  <= 1'b1;
                        la_dinA <= in_pix;
                    end else begin
                        lb_weA  <= 1'b1;
                        lb_dinA <= in_pix;
                    end

                    // Stage the sobel byte for output. The output pixel
                    // corresponds to column (x_pos - 1), row (y_pos - 1).
                    // We assemble 4 bytes into a 32-bit word and write to
                    // out_mem when complete. Use x_pos[1:0] to decide which
                    // byte slot we're filling. Note: x_pos has just been
                    // incremented in the previous pixel, so the "current
                    // output column" is x_pos - 1, whose [1:0] equals
                    // (x_pos + 3) [1:0] = x_pos[1:0] XOR 11 — easier to
                    // track explicitly with a local position.
                    //
                    // Simpler: track the *output* column independently.
                    // But we already advance x_pos here. So compute the
                    // output position from x_pos: out_col = x_pos - 1 when
                    // x_pos >= 1, else (wraps from previous row, ignore).
                    //
                    // For first cut: write the staging bytes into a 4-byte
                    // accumulator and dump to out_mem every 4 valid outputs.

                    // We only actually emit sobel output if stencil is valid
                    // AND we are past the first pixel of the row (need x_pos>=1
                    // for an output column to exist).
                    // We'll buffer the byte in out_b0/1/2 based on x_pos[1:0].
                    // When x_pos[1:0] == 2'b11 after the write, we have 4 bytes
                    // ready for the output word at addr (x_pos-3)/4 in row (y_pos-1).

                    // Use the simpler approach: track output column via x_pos.
                    // The "just-computed" sobel_pix is for output column
                    // OUT_COL = x_pos - 1.
                    // OUT_COL[1:0] tells us the byte position in the word.
                    // OUT_COL/4 is the word address.
                    // We need y_pos >= 2 for output to be valid.

                    if (y_pos >= 16'd2 && x_pos >= 16'd1) begin
                        case ((x_pos - 16'd1) & 16'd3)
                            16'd0: out_b0 <= sobel_pix;
                            16'd1: out_b1 <= sobel_pix;
                            16'd2: out_b2 <= sobel_pix;
                            16'd3: begin
                                // Write the assembled word to out_mem.
                                // Word address = (out_col - 3) / 4 = (x_pos - 4) / 4
                                ob_addrA <= (x_pos - 16'd4) >> 2;
                                ob_dinA  <= {sobel_pix, out_b2, out_b1, out_b0};
                                ob_weA   <= 1'b1;
                            end
                        endcase
                    end

                    // Advance position
                    if (x_pos == width_r - 16'd1) begin
                        x_pos <= 16'd0;
                        y_pos <= y_pos + 16'd1;
                        if (y_pos >= 16'd1) out_row_ready <= 1'b1;
                    end else begin
                        x_pos <= x_pos + 16'd1;
                    end

                    // Next pixel or end of FEED?
                    if (feed_pix == 2'd3) begin
                        feed_active <= 1'b0;
                        feed_pix    <= 2'd0;
                    end else begin
                        feed_pix <= feed_pix + 2'd1;
                    end
                    feed_phase <= 1'b0;
                end
            end

            // ============= READ_OUT state machine =============
            else if (read_active) begin
                if (read_phase == 1'b0) begin
                    // Address was presented last cycle; data is now valid.
                    read_phase  <= 1'b1;
                end else begin
                    // done was asserted; deactivate.
                    read_active <= 1'b0;
                    read_phase  <= 1'b0;
                end
            end

            // ============= New CI invocation =============
            else if (fire) begin
                case (op)
                OP_FEED: begin
                    feed_word   <= valueB;
                    feed_pix    <= 2'd0;
                    feed_phase  <= 1'b0;
                    feed_active <= 1'b1;
                end
                OP_READ_OUT: begin
                    ob_addrB     <= valueB[OUT_ADDR_BITS-1:0];
                    read_active  <= 1'b1;
                    read_phase   <= 1'b0;
                end
                OP_RESET: begin
                    x_pos <= 0; y_pos <= 0; out_row_ready <= 1'b0;
                    feed_active <= 1'b0; feed_pix <= 0; feed_phase <= 0;
                    read_active <= 1'b0; read_phase <= 0;
                    prev2_p0 <= 0; prev2_p1 <= 0; prev2_p2 <= 0;
                    prev1_p0 <= 0; prev1_p1 <= 0; prev1_p2 <= 0;
                    cur_p0   <= 0; cur_p1   <= 0; cur_p2   <= 0;
                    out_b0 <= 0; out_b1 <= 0; out_b2 <= 0;
                end
                OP_SET_THR:   threshold_r <= valueB[7:0];
                OP_SET_WIDTH: width_r     <= valueB[15:0];
                OP_ACK_ROW:   out_row_ready <= 1'b0;
                default: /* STATUS: no side effect */ ;
                endcase
            end
        end
    end

    // ---- Combinational address selection for line buffers ----
    // We present the read address during feed_phase 0; the data appears on doutA in phase 1.
    // Both BRAMs see the same address (x_pos) — we don't know yet which holds prev2 vs prev1.
    always @(*) begin
        la_addrA = x_pos[ROW_ADDR_BITS-1:0];
        lb_addrA = x_pos[ROW_ADDR_BITS-1:0];
    end

endmodule