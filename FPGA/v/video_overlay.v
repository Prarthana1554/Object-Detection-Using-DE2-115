// -----------------------------------------------------------------------------
// video_overlay.v
// Draws YOLO results on top of the live 800x600 camera image:
//   * a 3-pixel bounding rectangle per detection, one colour per class
//   * a filled label bar above the box with black text "PERSON 92%"
//     (project label, then confidence percent), 5x7 font
//   * only the 10 project classes are drawn: PERSON BOTTLE MOBILE NOTEBOOK CUP
//     MOUSE KEYBOARD LAPTOP POUCH BAG (see coco_label_rom.v)
//     scaled 2x -> 12x16 pixel character cells.
// Up to 8 detections. Slot 0 is drawn on top of slot 1, and so on
// (the Windows program sorts by confidence, highest first).
//
// Fully pipelined in the VGA pixel clock domain: 6 register stages. The camera
// pixel AND the HS / VS / BLANK / SYNC signals are delayed by the same 6
// clocks, so the timing relationship that the Terasic VGA_Controller produced
// is preserved exactly (the whole picture is just output 6 pixel clocks later).
//
// Detection snapshot handling:
//   det_pulse (from detection_cdc) stores a new snapshot as "pending". The
//   pending snapshot becomes visible at the start of the next video frame
//   (VS falling edge) so a box never tears halfway down the screen.
//   If no new result arrives for TIMEOUT_FRAMES video frames (~1.5 s), all
//   boxes are removed: nothing stale is left on screen if Python stops.
// -----------------------------------------------------------------------------
module video_overlay #(
    parameter [7:0] TIMEOUT_FRAMES = 8'd90
)(
    input  wire         clk,
    input  wire         reset_n,
    input  wire         enable,          // 1 = draw overlay

    // from VGA_Controller (all registered there, mutually aligned)
    input  wire         in_active,
    input  wire [9:0]   in_x,
    input  wire [9:0]   in_y,
    input  wire [9:0]   in_r,
    input  wire [9:0]   in_g,
    input  wire [9:0]   in_b,
    input  wire         in_hs,
    input  wire         in_vs,
    input  wire         in_blank,
    input  wire         in_sync,

    // detection snapshot (VGA clock domain)
    input  wire [651:0] det_bus,         // {frame_id[7:0], count[3:0], records[639:0]}
    input  wire         det_pulse,

    output reg  [9:0]   out_r,
    output reg  [9:0]   out_g,
    output reg  [9:0]   out_b,
    output reg          out_hs,
    output reg          out_vs,
    output reg          out_blank,
    output reg          out_sync,
    output wire         dets_visible
);
    localparam N = 8;

    // ------------------------------------------------------------------
    // Snapshot latching at frame start + stale timeout
    // ------------------------------------------------------------------
    reg  [639:0] pend_rec, act_rec;
    reg  [3:0]   pend_cnt, act_cnt;
    reg          pend_valid;
    reg          vs_d;
    reg  [7:0]   stale;
    wire         frame_start = vs_d & ~in_vs;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pend_rec   <= 640'd0;
            pend_cnt   <= 4'd0;
            pend_valid <= 1'b0;
            act_rec    <= 640'd0;
            act_cnt    <= 4'd0;
            vs_d       <= 1'b1;
            stale      <= 8'd0;
        end else begin
            vs_d <= in_vs;
            if (det_pulse) begin
                pend_rec   <= det_bus[639:0];
                pend_cnt   <= det_bus[643:640];
                pend_valid <= 1'b1;
            end
            if (frame_start) begin
                if (pend_valid && !det_pulse) begin
                    act_rec    <= pend_rec;
                    act_cnt    <= pend_cnt;
                    pend_valid <= 1'b0;
                    stale      <= 8'd0;
                end else if (stale >= TIMEOUT_FRAMES) begin
                    act_cnt    <= 4'd0;
                end else begin
                    stale      <= stale + 8'd1;
                end
            end
        end
    end

    assign dets_visible = enable && (act_cnt != 4'd0);

    // ------------------------------------------------------------------
    // Per-slot geometry (registered; changes only when act_* changes)
    // ------------------------------------------------------------------
    reg [N-1:0]     g_valid;
    reg [N*10-1:0]  g_x1, g_y1, g_x2, g_y2, g_ix1, g_iy1, g_ix2, g_iy2;
    reg [N*10-1:0]  g_lx1, g_ly1, g_lx2, g_ly2;
    reg [N*7-1:0]   g_cls;
    reg [N*4-1:0]   g_len, g_tens, g_ones;
    reg [N*30-1:0]  g_col;

    function [29:0] palette;               // 10-bit R,G,B, one colour per project class
        input [7:0] k;                     // COCO class id
        reg [23:0] c;
        begin
            case (k)
                8'd0:  c = 24'hFF3838;   // PERSON   red
                8'd39: c = 24'h00C2FF;   // BOTTLE   sky blue
                8'd67: c = 24'hFFB21D;   // MOBILE   amber
                8'd73: c = 24'hCB38FF;   // NOTEBOOK violet
                8'd41: c = 24'h48F90A;   // CUP      green
                8'd64: c = 24'h00D4BB;   // MOUSE    teal
                8'd66: c = 24'hFF701F;   // KEYBOARD orange
                8'd63: c = 24'hFF37C7;   // LAPTOP   pink
                8'd26: c = 24'hFFFF40;   // POUCH    yellow
                default: c = 24'hFFFFFF; // BAG      white
            endcase
            palette = {c[23:16], c[23:22], c[15:8], c[15:14], c[7:0], c[7:6]};
        end
    endfunction

    genvar gi;
    generate
        for (gi = 0; gi < N; gi = gi + 1) begin : g_slot
            wire [79:0] rec  = act_rec[639 - 80*gi -: 80];
            wire [7:0]  cls  = rec[79:72];
            wire [7:0]  conf = rec[71:64];
            wire [15:0] x1   = rec[63:48];
            wire [15:0] y1   = rec[47:32];
            wire [15:0] x2   = rec[31:16];
            wire [15:0] y2   = rec[15:0];
            wire [3:0]  nlen;
            coco_name_len u_len (.class_id(cls), .len(nlen));

            // Only the 10 project classes have a label (nlen != 0); anything
            // else is ignored even if it were received.
            wire        ok   = (gi < act_cnt) && (cls < 8'd80) && (nlen != 4'd0) &&
                               (x2 > x1) && (y2 > y1) &&
                               (x2 < 16'd800) && (y2 < 16'd600);
            wire [6:0]  c99  = (conf > 8'd99) ? 7'd99 : conf[6:0];
            wire [9:0]  lw   = ({6'd0, nlen} + 10'd4) * 10'd12 + 10'd4;  // label width
            wire [9:0]  lx   = ((x1[9:0] + lw) > 10'd800) ? (10'd800 - lw) : x1[9:0];
            wire [9:0]  ly   = (y1[9:0] >= 10'd20) ? (y1[9:0] - 10'd20) : (y1[9:0] + 10'd3);

            always @(posedge clk or negedge reset_n) begin
                if (!reset_n) begin
                    g_valid[gi]          <= 1'b0;
                    g_x1[gi*10 +: 10]    <= 10'd0;
                    g_y1[gi*10 +: 10]    <= 10'd0;
                    g_x2[gi*10 +: 10]    <= 10'd0;
                    g_y2[gi*10 +: 10]    <= 10'd0;
                    g_ix1[gi*10 +: 10]   <= 10'd0;
                    g_iy1[gi*10 +: 10]   <= 10'd0;
                    g_ix2[gi*10 +: 10]   <= 10'd0;
                    g_iy2[gi*10 +: 10]   <= 10'd0;
                    g_lx1[gi*10 +: 10]   <= 10'd0;
                    g_ly1[gi*10 +: 10]   <= 10'd0;
                    g_lx2[gi*10 +: 10]   <= 10'd0;
                    g_ly2[gi*10 +: 10]   <= 10'd0;
                    g_cls[gi*7 +: 7]     <= 7'd0;
                    g_len[gi*4 +: 4]     <= 4'd0;
                    g_tens[gi*4 +: 4]    <= 4'd0;
                    g_ones[gi*4 +: 4]    <= 4'd0;
                    g_col[gi*30 +: 30]   <= 30'd0;
                end else begin
                    g_valid[gi]          <= ok && enable;
                    g_x1[gi*10 +: 10]    <= x1[9:0];
                    g_y1[gi*10 +: 10]    <= y1[9:0];
                    g_x2[gi*10 +: 10]    <= x2[9:0];
                    g_y2[gi*10 +: 10]    <= y2[9:0];
                    g_ix1[gi*10 +: 10]   <= x1[9:0] + 10'd3;
                    g_iy1[gi*10 +: 10]   <= y1[9:0] + 10'd3;
                    g_ix2[gi*10 +: 10]   <= (x2[9:0] >= 10'd3) ? (x2[9:0] - 10'd3) : 10'd0;
                    g_iy2[gi*10 +: 10]   <= (y2[9:0] >= 10'd3) ? (y2[9:0] - 10'd3) : 10'd0;
                    g_lx1[gi*10 +: 10]   <= lx;
                    g_ly1[gi*10 +: 10]   <= ly;
                    g_lx2[gi*10 +: 10]   <= lx + lw;
                    g_ly2[gi*10 +: 10]   <= ly + 10'd20;
                    g_cls[gi*7 +: 7]     <= cls[6:0];
                    g_len[gi*4 +: 4]     <= nlen;
                    g_tens[gi*4 +: 4]    <= c99 / 7'd10;
                    g_ones[gi*4 +: 4]    <= c99 % 7'd10;
                    g_col[gi*30 +: 30]   <= palette(cls);
                end
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Pass-through pipeline (camera pixel + sync), 5 stages + output reg
    // ------------------------------------------------------------------
    reg [34:0] p1, p2, p3, p4, p5;   // {active, hs, vs, blank, sync, r, g, b}
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            p1 <= 35'd0; p2 <= 35'd0; p3 <= 35'd0; p4 <= 35'd0; p5 <= 35'd0;
        end else begin
            p1 <= {in_active, in_hs, in_vs, in_blank, in_sync, in_r, in_g, in_b};
            p2 <= p1;
            p3 <= p2;
            p4 <= p3;
            p5 <= p4;
        end
    end

    // ------------------------------------------------------------------
    // Stage 1: per-slot hit tests
    // ------------------------------------------------------------------
    reg [N-1:0]   s1_box, s1_lab;
    reg [N*8-1:0] s1_lx;
    reg [N*5-1:0] s1_ly;

    generate
        for (gi = 0; gi < N; gi = gi + 1) begin : g_hit
            wire [9:0] x1  = g_x1[gi*10 +: 10];
            wire [9:0] y1  = g_y1[gi*10 +: 10];
            wire [9:0] x2  = g_x2[gi*10 +: 10];
            wire [9:0] y2  = g_y2[gi*10 +: 10];
            wire [9:0] ix1 = g_ix1[gi*10 +: 10];
            wire [9:0] iy1 = g_iy1[gi*10 +: 10];
            wire [9:0] ix2 = g_ix2[gi*10 +: 10];
            wire [9:0] iy2 = g_iy2[gi*10 +: 10];
            wire [9:0] lx1 = g_lx1[gi*10 +: 10];
            wire [9:0] ly1 = g_ly1[gi*10 +: 10];
            wire [9:0] lx2 = g_lx2[gi*10 +: 10];
            wire [9:0] ly2 = g_ly2[gi*10 +: 10];

            wire outer = (in_x >= x1) && (in_x <= x2) && (in_y >= y1) && (in_y <= y2);
            wire inner = (in_x >= ix1) && (in_x <= ix2) && (in_y >= iy1) && (in_y <= iy2);
            wire lab   = (in_x >= lx1) && (in_x <  lx2) && (in_y >= ly1) && (in_y <  ly2);
            wire [9:0] dx = in_x - lx1;
            wire [9:0] dy = in_y - ly1;

            always @(posedge clk or negedge reset_n) begin
                if (!reset_n) begin
                    s1_box[gi]         <= 1'b0;
                    s1_lab[gi]         <= 1'b0;
                    s1_lx[gi*8 +: 8]   <= 8'd0;
                    s1_ly[gi*5 +: 5]   <= 5'd0;
                end else begin
                    s1_box[gi]         <= g_valid[gi] && in_active && outer && !inner;
                    s1_lab[gi]         <= g_valid[gi] && in_active && lab;
                    s1_lx[gi*8 +: 8]   <= dx[7:0];
                    s1_ly[gi*5 +: 5]   <= dy[4:0];
                end
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Stage 2: priority select (lowest slot index wins; labels over boxes)
    // ------------------------------------------------------------------
    reg        s2_lab, s2_box;
    reg [29:0] s2_lcol, s2_bcol;
    reg [7:0]  s2_lx;
    reg [4:0]  s2_ly;
    reg [6:0]  s2_cls;
    reg [3:0]  s2_len, s2_tens, s2_ones;

    reg [2:0]  c_lsel, c_bsel;
    reg        c_lany, c_bany;
    integer    k;
    always @* begin
        c_lsel = 3'd0; c_bsel = 3'd0; c_lany = 1'b0; c_bany = 1'b0;
        for (k = N-1; k >= 0; k = k - 1) begin
            if (s1_lab[k]) begin c_lsel = k[2:0]; c_lany = 1'b1; end
            if (s1_box[k]) begin c_bsel = k[2:0]; c_bany = 1'b1; end
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            s2_lab <= 1'b0; s2_box <= 1'b0;
            s2_lcol <= 30'd0; s2_bcol <= 30'd0;
            s2_lx <= 8'd0; s2_ly <= 5'd0; s2_cls <= 7'd0;
            s2_len <= 4'd0; s2_tens <= 4'd0; s2_ones <= 4'd0;
        end else begin
            s2_lab  <= c_lany;
            s2_box  <= c_bany;
            s2_lcol <= g_col[c_lsel*30 +: 30];
            s2_bcol <= g_col[c_bsel*30 +: 30];
            s2_lx   <= s1_lx[c_lsel*8 +: 8];
            s2_ly   <= s1_ly[c_lsel*5 +: 5];
            s2_cls  <= g_cls[c_lsel*7 +: 7];
            s2_len  <= g_len[c_lsel*4 +: 4];
            s2_tens <= g_tens[c_lsel*4 +: 4];
            s2_ones <= g_ones[c_lsel*4 +: 4];
        end
    end

    // ------------------------------------------------------------------
    // Stage 3: character cell / glyph pixel coordinates inside the label
    //   label = 2 px padding, then (len+4) cells of 12x16 px, 2 px padding
    // ------------------------------------------------------------------
    localparam K_SPACE = 3'd0, K_NAME = 3'd1, K_TENS = 3'd2, K_ONES = 3'd3, K_PCT = 3'd4;

    wire [7:0] t_x   = s2_lx - 8'd2;
    wire [4:0] t_y   = s2_ly - 5'd2;
    wire       t_in  = (s2_lx >= 8'd2) && (s2_ly >= 5'd2) && (s2_ly < 5'd18);
    wire [4:0] t_ci  = t_x / 8'd12;                       // character index
    wire [3:0] t_pc  = t_x - t_ci * 8'd12;                // pixel inside the 12-px cell
    wire [4:0] t_len = {1'b0, s2_len};

    reg [2:0]  t_kind;
    always @* begin
        if (t_ci < t_len)                  t_kind = K_NAME;
        else if (t_ci == t_len + 5'd1)     t_kind = K_TENS;
        else if (t_ci == t_len + 5'd2)     t_kind = K_ONES;
        else if (t_ci == t_len + 5'd3)     t_kind = K_PCT;
        else                               t_kind = K_SPACE;
    end

    reg        s3_lab, s3_box, s3_in;
    reg [29:0] s3_lcol, s3_bcol;
    reg [2:0]  s3_kind;
    reg [5:0]  s3_digit;
    reg [2:0]  s3_col, s3_row;
    reg [6:0]  s3_cls;
    reg [3:0]  s3_ci;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            s3_lab <= 1'b0; s3_box <= 1'b0; s3_in <= 1'b0;
            s3_lcol <= 30'd0; s3_bcol <= 30'd0;
            s3_kind <= K_SPACE; s3_digit <= 6'd0;
            s3_col <= 3'd0; s3_row <= 3'd0; s3_cls <= 7'd0; s3_ci <= 4'd0;
        end else begin
            s3_lab   <= s2_lab;
            s3_box   <= s2_box;
            s3_in    <= t_in;
            s3_lcol  <= s2_lcol;
            s3_bcol  <= s2_bcol;
            s3_kind  <= t_kind;
            s3_digit <= 6'd27 + {2'd0, (t_kind == K_TENS) ? s2_tens : s2_ones};
            s3_col   <= t_pc[3:1];       // 0..5 (glyph columns 0..4, column 5 = gap)
            s3_row   <= t_y[3:1];        // 0..7 (glyph rows 0..6, row 7 = gap)
            s3_cls   <= s2_cls;
            s3_ci    <= t_ci[3:0];
        end
    end

    // ------------------------------------------------------------------
    // Stage 4: class-name character ROM (synchronous, M9K-capable)
    // ------------------------------------------------------------------
    wire [5:0] name_code;
    coco_label_rom u_label_rom (.clk(clk), .class_id(s3_cls), .idx(s3_ci), .q(name_code));

    reg        s4_lab, s4_box, s4_in;
    reg [29:0] s4_lcol, s4_bcol;
    reg [2:0]  s4_kind;
    reg [5:0]  s4_digit;
    reg [2:0]  s4_col, s4_row;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            s4_lab <= 1'b0; s4_box <= 1'b0; s4_in <= 1'b0;
            s4_lcol <= 30'd0; s4_bcol <= 30'd0;
            s4_kind <= K_SPACE; s4_digit <= 6'd0; s4_col <= 3'd0; s4_row <= 3'd0;
        end else begin
            s4_lab <= s3_lab;  s4_box <= s3_box;  s4_in <= s3_in;
            s4_lcol <= s3_lcol; s4_bcol <= s3_bcol;
            s4_kind <= s3_kind; s4_digit <= s3_digit;
            s4_col <= s3_col;  s4_row <= s3_row;
        end
    end

    // ------------------------------------------------------------------
    // Stage 5: font ROM (synchronous)
    // ------------------------------------------------------------------
    reg [5:0] char_code;
    always @* begin
        case (s4_kind)
            K_NAME:         char_code = name_code;
            K_TENS, K_ONES: char_code = s4_digit;
            K_PCT:          char_code = 6'd37;
            default:        char_code = 6'd0;
        endcase
    end

    wire [4:0] glyph_row;
    font5x7_rom u_font (.clk(clk), .code(char_code), .row(s4_row), .q(glyph_row));

    reg        s5_lab, s5_box, s5_in;
    reg [29:0] s5_lcol, s5_bcol;
    reg [2:0]  s5_col, s5_row;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            s5_lab <= 1'b0; s5_box <= 1'b0; s5_in <= 1'b0;
            s5_lcol <= 30'd0; s5_bcol <= 30'd0; s5_col <= 3'd0; s5_row <= 3'd0;
        end else begin
            s5_lab <= s4_lab;  s5_box <= s4_box;  s5_in <= s4_in;
            s5_lcol <= s4_lcol; s5_bcol <= s4_bcol;
            s5_col <= s4_col;  s5_row <= s4_row;
        end
    end

    // ------------------------------------------------------------------
    // Stage 6: final pixel + delayed sync outputs
    // ------------------------------------------------------------------
    wire text_px = s5_in && (s5_row != 3'd7) && (s5_col < 3'd5) &&
                   glyph_row[3'd4 - s5_col];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            out_r <= 10'd0; out_g <= 10'd0; out_b <= 10'd0;
            out_hs <= 1'b0; out_vs <= 1'b0; out_blank <= 1'b0; out_sync <= 1'b0;
        end else begin
            out_hs    <= p5[33];
            out_vs    <= p5[32];
            out_blank <= p5[31];
            out_sync  <= p5[30];
            if (p5[34] && s5_lab) begin
                if (text_px) begin
                    out_r <= 10'd0; out_g <= 10'd0; out_b <= 10'd0;           // black text
                end else begin
                    out_r <= s5_lcol[29:20]; out_g <= s5_lcol[19:10]; out_b <= s5_lcol[9:0];
                end
            end else if (p5[34] && s5_box) begin
                out_r <= s5_bcol[29:20]; out_g <= s5_bcol[19:10]; out_b <= s5_bcol[9:0];
            end else begin
                out_r <= p5[29:20]; out_g <= p5[19:10]; out_b <= p5[9:0];     // camera
            end
        end
    end
endmodule
