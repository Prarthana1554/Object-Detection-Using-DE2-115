// -----------------------------------------------------------------------------
// hw_object_detector.v
// Object recognition done entirely in FPGA hardware (no PC, no neural network).
//
// Method: background subtraction + feature signature + nearest-neighbour match.
//
//  1. BACKGROUND: in learn mode, KEY2 stores the empty scene. Every 4th pixel
//     of every 4th line (200x150 samples) is saved in on-chip RAM (M9K),
//     6 bits per colour.
//  2. FOREGROUND: for every sample of every frame,
//        diff = |R-Rbg| + |G-Gbg| + |B-Bbg|   (6-bit colours)
//     diff > threshold  ->  the sample belongs to the object.
//  3. FEATURES of the object, measured over one frame:
//        mean R, mean G, mean B                       (3 values)
//        colour histogram, 8 colour classes           (8 values, share of pixels)
//        size, bounding-box width, height, fill ratio (4 values)
//     = a 15-byte "signature".
//  4. LEARN: in learn mode, KEY1 stores the current signature in memory slot
//     {SW4, SW3..SW0}: 10 objects x 2 samples = 20 slots.
//  5. DETECT: every frame the live signature is compared with all stored
//     signatures (weighted sum of absolute differences, "L1 distance").
//     The closest one wins if it is close enough; the box and label are sent
//     to video_overlay in its normal detection-record format.
//
// All per-pixel work happens while the picture is drawn; the divisions and
// the comparison run during the vertical blanking interval (a few hundred
// clock cycles of the ~28,000 available).
// -----------------------------------------------------------------------------
module hw_object_detector #(
    parameter [15:0] MIN_PIXELS = 16'd40,     // fewer foreground samples = no object
    parameter [13:0] REJECT     = 14'd700,    // distance above this = unknown object
    parameter [3:0]  SCORE_SHIFT = 4'd3       // score = 99 - distance/8
)(
    input  wire        clk,            // VGA_CTRL_CLK (40 MHz)
    input  wire        reset_n,

    // live picture from VGA_Controller
    input  wire        active,
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire [9:0]  r,
    input  wire [9:0]  g,
    input  wire [9:0]  b,

    // user controls (raw board signals)
    input  wire        learn_mode,     // SW14: 1 = learn, 0 = detect
    input  wire [3:0]  sel_class,      // SW3..SW0: object number 0..9
    input  wire        sel_sample,     // SW4: sample A / B
    input  wire [1:0]  thr_sel,        // SW13..SW12: foreground threshold
    input  wire        key_bg_n,       // KEY2 (active low): store background
    input  wire        key_learn_n,    // KEY1 (active low): store signature

    // result for video_overlay (same format as the old Ethernet receiver)
    output reg  [651:0] det_bus,       // {frame_id, count, 8 x 80-bit records}
    output reg          det_pulse,

    // status for LEDs / HEX
    output reg          bg_valid,
    output reg          obj_present,
    output reg          saved_pulse,
    output reg          match_valid,
    output reg  [3:0]   best_class,
    output reg  [6:0]   score,
    output wire [9:0]   learned_mask
);
    // ------------------------------------------------------------------
    // Object number -> COCO class id used by the label ROM in video_overlay
    // ------------------------------------------------------------------
    function [7:0] coco_id;
        input [3:0] c;
        begin
            case (c)
                4'd0: coco_id = 8'd0;    // PERSON
                4'd1: coco_id = 8'd39;   // BOTTLE
                4'd2: coco_id = 8'd67;   // MOBILE
                4'd3: coco_id = 8'd73;   // NOTEBOOK
                4'd4: coco_id = 8'd41;   // CUP
                4'd5: coco_id = 8'd64;   // MOUSE
                4'd6: coco_id = 8'd66;   // KEYBOARD
                4'd7: coco_id = 8'd63;   // LAPTOP
                4'd8: coco_id = 8'd26;   // POUCH
                default: coco_id = 8'd24;// BAG
            endcase
        end
    endfunction

    wire [3:0] cls_sel = (sel_class > 4'd9) ? 4'd9 : sel_class;

    // ------------------------------------------------------------------
    // Key synchronisers and press detection (keys read 0 when pressed)
    // ------------------------------------------------------------------
    reg [2:0] kbg, klr;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin kbg <= 3'b111; klr <= 3'b111; end
        else begin kbg <= {kbg[1:0], key_bg_n}; klr <= {klr[1:0], key_learn_n}; end
    end
    wire bg_press    = kbg[2] & ~kbg[1];   // 1 -> 0 edge
    wire learn_press = klr[2] & ~klr[1];

    // ------------------------------------------------------------------
    // Stage 0: pick every 4th pixel of every 4th line (200 x 150 samples)
    // ------------------------------------------------------------------
    wire       samp   = active && (x < 10'd800) && (y < 10'd600) &&
                        (x[1:0] == 2'b00) && (y[1:0] == 2'b00);
    wire [7:0] sx     = x[9:2];                    // 0..199
    wire [7:0] sy     = y[9:2];                    // 0..149
    wire [14:0] sy_ext = {7'd0, sy};
    wire [14:0] saddr = (sy_ext << 7) + (sy_ext << 6) + (sy_ext << 3) + {7'd0, sx};
    wire       s_first = samp && (sx == 8'd0)   && (sy == 8'd0);
    wire       s_last  = samp && (sx == 8'd199) && (sy == 8'd149);

    // ------------------------------------------------------------------
    // Background memory: 32768 x 18 bit (6 bits per colour) in M9K blocks
    // ------------------------------------------------------------------
    (* ramstyle = "M9K" *) reg [17:0] bg_mem [0:32767];
    reg  [17:0] bg_q;
    reg         bg_req, cap_active;
    wire        bg_we = samp && (cap_active || (s_first && bg_req));

    always @(posedge clk) begin
        if (bg_we)
            bg_mem[saddr] <= {r[9:4], g[9:4], b[9:4]};
        bg_q <= bg_mem[saddr];                  // read, valid next clock
    end

    // ------------------------------------------------------------------
    // Stage A: register the sample while the background is being read
    // ------------------------------------------------------------------
    reg       a_v, a_last;
    reg [7:0] a_x, a_y, a_r, a_g, a_b;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            a_v <= 1'b0; a_last <= 1'b0;
            a_x <= 8'd0; a_y <= 8'd0; a_r <= 8'd0; a_g <= 8'd0; a_b <= 8'd0;
        end else begin
            a_v    <= samp;
            a_last <= s_last;
            a_x <= sx;  a_y <= sy;
            a_r <= r[9:2]; a_g <= g[9:2]; a_b <= b[9:2];
        end
    end

    // ------------------------------------------------------------------
    // Stage B: foreground test + colour class (combinational)
    // ------------------------------------------------------------------
    wire [5:0] c_r = a_r[7:2], c_g = a_g[7:2], c_b = a_b[7:2];
    wire [5:0] q_r = bg_q[17:12], q_g = bg_q[11:6], q_b = bg_q[5:0];
    wire [5:0] d_r = (c_r > q_r) ? (c_r - q_r) : (q_r - c_r);
    wire [5:0] d_g = (c_g > q_g) ? (c_g - q_g) : (q_g - c_g);
    wire [5:0] d_b = (c_b > q_b) ? (c_b - q_b) : (q_b - c_b);
    wire [7:0] diff = {2'b0, d_r} + {2'b0, d_g} + {2'b0, d_b};

    reg [7:0] thr;
    always @* begin
        case (thr_sel)
            2'd0: thr = 8'd12;
            2'd1: thr = 8'd18;
            2'd2: thr = 8'd26;
            default: thr = 8'd36;
        endcase
    end
    wire fg = a_v && bg_valid && (diff > thr);

    // colour class of the pixel (8 classes, comparisons only)
    wire [7:0] mx  = (a_r >= a_g && a_r >= a_b) ? a_r : ((a_g >= a_b) ? a_g : a_b);
    wire [7:0] mn  = (a_r <= a_g && a_r <= a_b) ? a_r : ((a_g <= a_b) ? a_g : a_b);
    wire [7:0] sat = mx - mn;
    wire [7:0] hs  = {1'b0, sat[7:1]};          // half saturation
    reg  [2:0] cbin;
    always @* begin
        if (mx < 8'd50)                     cbin = 3'd0;   // black / very dark
        else if (sat < 8'd40)               cbin = (mx > 8'd170) ? 3'd2 : 3'd1; // white : grey
        else if (a_r == mx) begin
            if (a_b > a_g + hs)             cbin = 3'd7;   // pink / purple
            else if (a_g > a_r - hs)        cbin = 3'd4;   // yellow / orange
            else                            cbin = 3'd3;   // red
        end else if (a_g == mx) begin
            if (a_b > a_r + hs)             cbin = 3'd6;   // cyan -> blue class
            else if (a_r > a_g - hs)        cbin = 3'd4;   // yellow
            else                            cbin = 3'd5;   // green
        end else begin
            if (a_r > a_g + hs)             cbin = 3'd7;   // purple
            else                            cbin = 3'd6;   // blue
        end
    end

    // noise filter for the box: two foreground samples side by side
    reg       prev_fg;
    reg [7:0] prev_x;
    wire      fg2 = fg && prev_fg && (a_x == prev_x + 8'd1);

    // ------------------------------------------------------------------
    // Per-frame accumulators
    // ------------------------------------------------------------------
    reg [15:0] acc_n;
    reg [23:0] acc_r, acc_g, acc_b;
    reg [15:0] acc_h [0:7];
    reg [7:0]  acc_x0, acc_x1, acc_y0, acc_y1;
    reg        acc_box;
    reg        latch_req;
    integer    i;

    // snapshot used by the compute machine
    reg [15:0] s_n;
    reg [23:0] s_r, s_g, s_b;
    reg [15:0] s_h [0:7];
    reg [7:0]  s_x0, s_x1, s_y0, s_y1;
    reg        s_box;

    // ------------------------------------------------------------------
    // Signature memory: 20 slots x 15 features x 8 bits
    // ------------------------------------------------------------------
    reg [119:0] sig [0:19];
    reg [19:0]  sig_valid;
    genvar gm;
    generate
        for (gm = 0; gm < 10; gm = gm + 1) begin : g_mask
            assign learned_mask[gm] = sig_valid[gm] | sig_valid[gm + 10];
        end
    endgenerate

    // ------------------------------------------------------------------
    // Compute state machine (runs during vertical blanking)
    //   C_START  : decide whether an object is present
    //   C_LOAD   : load one division job (12 jobs: 3 means, 8 histogram
    //              shares, fill ratio)
    //   C_RUN    : restoring division, one quotient bit per clock (24 clocks)
    //   C_SAVE   : store the 8-bit result as a feature
    //   C_STORE  : in learn mode, save the signature in the selected slot
    //   C_MATCH  : distance to all 20 slots, one feature per clock
    //   C_OUT    : build the detection record for the overlay
    // ------------------------------------------------------------------
    localparam C_IDLE = 4'd0, C_START = 4'd1, C_LOAD = 4'd2, C_RUN = 4'd3,
               C_SAVE = 4'd4, C_STORE = 4'd5, C_MATCH = 4'd6, C_OUT = 4'd7;
    reg [3:0]   cst;
    reg [3:0]   job;
    reg [4:0]   dbit;
    reg [23:0]  dvd, quo;
    reg [16:0]  rem;
    reg [15:0]  dvs;
    reg [119:0] feat;            // f0 = feat[119:112] ... f14 = feat[7:0]
    reg [7:0]   w, h;
    reg [15:0]  area;
    reg [4:0]   slot;
    reg [3:0]   fidx;
    reg [13:0]  dacc, best_dist;
    reg [4:0]   best_slot;
    reg         best_ok;
    reg         learn_req;
    reg [7:0]   frame_id;

    // operands of each division job
    reg [23:0] job_dvd;
    reg [15:0] job_dvs;
    always @* begin
        job_dvs = s_n;
        case (job)
            4'd0:  job_dvd = s_r;                              // mean red
            4'd1:  job_dvd = s_g;                              // mean green
            4'd2:  job_dvd = s_b;                              // mean blue
            4'd11: begin job_dvd = {s_n, 8'd0}; job_dvs = area; end // fill ratio x256
            default: job_dvd = {s_h[job - 4'd3], 8'd0};        // histogram share x256
        endcase
    end

    // one restoring-division step
    wire [16:0] rem_sh = {rem[15:0], dvd[dbit]};
    wire        ge     = (rem_sh >= {1'b0, dvs});
    wire [7:0]  q8     = (quo[23:8] != 16'd0) ? 8'hFF : quo[7:0];   // saturate to 255
    wire [3:0]  fslot  = (job == 4'd11) ? 4'd14 : job;               // feature index of a job

    // feature fidx of the live vector and of the slot being compared
    wire [7:0]   fa  = feat[119 - 8*fidx -: 8];
    wire [119:0] sv  = sig[slot];
    wire [7:0]   fb  = sv[119 - 8*fidx -: 8];
    wire [7:0]   ad  = (fa > fb) ? (fa - fb) : (fb - fa);
    // weights: the 8 colour-histogram features count double
    wire [9:0]   wad = (fidx >= 4'd3 && fidx <= 4'd10) ? {1'b0, ad, 1'b0} : {2'b0, ad};
    wire [13:0]  dsum = dacc + {4'd0, wad};

    wire [4:0]  sv_slot   = sel_sample ? ({1'b0, cls_sel} + 5'd10) : {1'b0, cls_sel};
    wire [3:0]  best_cls  = (best_slot >= 5'd10) ? (best_slot - 5'd10) : best_slot[3:0];
    wire [13:0] sd        = best_dist >> SCORE_SHIFT;
    wire        show      = learn_mode ? obj_present
                                       : (obj_present && best_ok && (best_dist <= REJECT));
    wire [3:0]  ocls      = learn_mode ? cls_sel : best_cls;
    wire [7:0]  oconf     = learn_mode ? 8'd0 : ((sd >= 14'd99) ? 8'd0 : (8'd99 - sd[7:0]));

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            bg_req <= 1'b0; cap_active <= 1'b0; bg_valid <= 1'b0;
            prev_fg <= 1'b0; prev_x <= 8'd0;
            acc_n <= 16'd0; acc_r <= 24'd0; acc_g <= 24'd0; acc_b <= 24'd0;
            for (i = 0; i < 8; i = i + 1) begin acc_h[i] <= 16'd0; s_h[i] <= 16'd0; end
            acc_x0 <= 8'hFF; acc_x1 <= 8'd0; acc_y0 <= 8'hFF; acc_y1 <= 8'd0; acc_box <= 1'b0;
            latch_req <= 1'b0;
            s_n <= 16'd0; s_r <= 24'd0; s_g <= 24'd0; s_b <= 24'd0;
            s_x0 <= 8'd0; s_x1 <= 8'd0; s_y0 <= 8'd0; s_y1 <= 8'd0; s_box <= 1'b0;
            sig_valid <= 20'd0;
            for (i = 0; i < 20; i = i + 1) sig[i] <= 120'd0;
            cst <= C_IDLE; job <= 4'd0; dbit <= 5'd0; dvd <= 24'd0; quo <= 24'd0;
            rem <= 17'd0; dvs <= 16'd1; feat <= 120'd0; w <= 8'd0; h <= 8'd0; area <= 16'd1;
            slot <= 5'd0; fidx <= 4'd0; dacc <= 14'd0; best_dist <= 14'h3FFF;
            best_slot <= 5'd0; best_ok <= 1'b0; learn_req <= 1'b0; frame_id <= 8'd0;
            det_bus <= 652'd0; det_pulse <= 1'b0;
            obj_present <= 1'b0; saved_pulse <= 1'b0; match_valid <= 1'b0;
            best_class <= 4'hF; score <= 7'd0;
        end else begin
            det_pulse   <= 1'b0;
            saved_pulse <= 1'b0;

            // ---------- user requests (learn mode only) ----------
            if (learn_mode && bg_press)    bg_req    <= 1'b1;
            if (learn_mode && learn_press) learn_req <= 1'b1;

            // ---------- background capture over one whole frame ----------
            if (s_first && bg_req) begin cap_active <= 1'b1; bg_req <= 1'b0; end
            if (s_last && cap_active) begin cap_active <= 1'b0; bg_valid <= 1'b1; end

            // ---------- per-sample accumulation (stage B) ----------
            if (a_v) begin
                prev_fg <= fg;
                prev_x  <= a_x;
                if (fg) begin
                    acc_n <= acc_n + 16'd1;
                    acc_r <= acc_r + {16'd0, a_r};
                    acc_g <= acc_g + {16'd0, a_g};
                    acc_b <= acc_b + {16'd0, a_b};
                    acc_h[cbin] <= acc_h[cbin] + 16'd1;
                end
                if (fg2) begin
                    acc_box <= 1'b1;
                    if (prev_x < acc_x0) acc_x0 <= prev_x;
                    if (a_x    > acc_x1) acc_x1 <= a_x;
                    if (a_y    < acc_y0) acc_y0 <= a_y;
                    if (a_y    > acc_y1) acc_y1 <= a_y;
                end
                if (a_last) latch_req <= 1'b1;
            end

            // ---------- end of frame: snapshot, clear, start computing ----------
            if (latch_req) begin
                latch_req <= 1'b0;
                s_n <= acc_n; s_r <= acc_r; s_g <= acc_g; s_b <= acc_b;
                for (i = 0; i < 8; i = i + 1) begin s_h[i] <= acc_h[i]; acc_h[i] <= 16'd0; end
                s_x0 <= acc_x0; s_x1 <= acc_x1; s_y0 <= acc_y0; s_y1 <= acc_y1; s_box <= acc_box;
                acc_n <= 16'd0; acc_r <= 24'd0; acc_g <= 24'd0; acc_b <= 24'd0;
                acc_x0 <= 8'hFF; acc_x1 <= 8'd0; acc_y0 <= 8'hFF; acc_y1 <= 8'd0; acc_box <= 1'b0;
                prev_fg  <= 1'b0;
                frame_id <= frame_id + 8'd1;
                cst      <= C_START;
            end else begin
                case (cst)
                    C_START: begin
                        w    <= s_x1 - s_x0 + 8'd1;
                        h    <= s_y1 - s_y0 + 8'd1;
                        area <= {8'd0, s_x1 - s_x0 + 8'd1} * {8'd0, s_y1 - s_y0 + 8'd1};
                        job  <= 4'd0;
                        if (s_box && s_n >= MIN_PIXELS) begin
                            obj_present <= 1'b1;
                            cst <= C_LOAD;
                        end else begin
                            obj_present <= 1'b0;
                            cst <= C_STORE;          // nothing to learn, no match
                        end
                    end

                    C_LOAD: begin
                        dvd  <= job_dvd;
                        dvs  <= (job_dvs == 16'd0) ? 16'd1 : job_dvs;
                        rem  <= 17'd0;
                        quo  <= 24'd0;
                        dbit <= 5'd23;
                        cst  <= C_RUN;
                    end

                    C_RUN: begin
                        rem       <= ge ? (rem_sh - {1'b0, dvs}) : rem_sh;
                        quo[dbit] <= ge;
                        if (dbit == 5'd0) cst <= C_SAVE;
                        else              dbit <= dbit - 5'd1;
                    end

                    C_SAVE: begin
                        feat[119 - 8*fslot -: 8] <= q8;
                        if (job == 4'd11) begin
                            feat[31:24] <= (s_n[15:7] > 9'd255) ? 8'hFF : s_n[14:7]; // f11 size
                            feat[23:16] <= w;                                        // f12 width
                            feat[15:8]  <= h;                                        // f13 height
                            cst <= C_STORE;
                        end else begin
                            job <= job + 4'd1;
                            cst <= C_LOAD;
                        end
                    end

                    C_STORE: begin
                        if (learn_req) begin
                            if (learn_mode && obj_present) begin
                                sig[sv_slot]       <= feat;
                                sig_valid[sv_slot] <= 1'b1;
                                saved_pulse        <= 1'b1;
                            end
                            learn_req <= 1'b0;
                        end
                        slot      <= 5'd0;
                        fidx      <= 4'd0;
                        dacc      <= 14'd0;
                        best_dist <= 14'h3FFF;
                        best_ok   <= 1'b0;
                        cst       <= obj_present ? C_MATCH : C_OUT;
                    end

                    C_MATCH: begin
                        if (fidx == 4'd14) begin
                            if (sig_valid[slot] && dsum < best_dist) begin
                                best_dist <= dsum;
                                best_slot <= slot;
                                best_ok   <= 1'b1;
                            end
                            dacc <= 14'd0;
                            fidx <= 4'd0;
                            if (slot == 5'd19) cst <= C_OUT;
                            else               slot <= slot + 5'd1;
                        end else begin
                            dacc <= dsum;
                            fidx <= fidx + 4'd1;
                        end
                    end

                    C_OUT: begin
                        match_valid <= show && !learn_mode;
                        best_class  <= show ? ocls : 4'hF;
                        score       <= show ? oconf[6:0] : 7'd0;
                        det_bus <= {frame_id, (show ? 4'd1 : 4'd0),
                                    coco_id(ocls), oconf,
                                    6'd0, s_x0, 2'b00,  6'd0, s_y0, 2'b00,
                                    6'd0, s_x1, 2'b11,  6'd0, s_y1, 2'b11,
                                    560'd0};
                        det_pulse <= 1'b1;
                        cst <= C_IDLE;
                    end

                    default: cst <= C_IDLE;
                endcase
            end
        end
    end
endmodule
