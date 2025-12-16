`default_nettype none
// ============================================================================
// nms3x3_event_pix_dense
// - Event-style 3x3 NMS with BRAM-based line buffers (II=1).
// - Parametric tie-break, (-1,-1), clamp, timing-friendly comparator tree.
// - TLAST can be per-row (last center of each row) or per-frame.
// - Verilog-2001 compliant; no mixed blocking/nonblocking in same always.
// ============================================================================
module nms3x3_event_pix_dense #(
    parameter integer MAX_W          = 2048,   // maximum frame width (for BRAM depth)
    parameter integer SCORE_W        = 8,      // score bitwidth
    parameter integer MIN_SCORE      = 1,      // minimal score to keep
    parameter integer STRICT_GREATER = 0,      // 1: center > neigh_max ; 0: allow tie by TIE_MODE
    parameter integer TIE_MODE       = 1,      // 0: strict only ; 1: even-even ; 2: xor phase
    parameter integer APPLY_NEG1     = 1,      // 1: apply (x-1,y-1) after NMS keep
    parameter integer CLAMP_MAX      = 1,      // 1: clamp to [0..W-1]/[0..H-1]
    parameter integer TLAST_EACH_ROW = 1       // 1: row-mode TLAST; 0: frame-mode TLAST
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // stream in (per-pixel)
    input  wire                     s_valid,
    output wire                     s_ready,
    input  wire [15:0]              s_x,
    input  wire [15:0]              s_y,
    input  wire [SCORE_W-1:0]       s_score,
    input  wire                     s_is_strong,
    input  wire                     s_sof,     // start of frame pulse at first pixel of frame
    input  wire [15:0]              frm_w,     // frame width  (must be <= MAX_W)
    input  wire [15:0]              frm_h,     // frame height

    // stream out (kept corners, one per clk at most)
    output reg                      m_valid,
    input  wire                     m_ready,
    output reg  [15:0]              m_x,
    output reg  [15:0]              m_y,
    output reg                      m_is_strong,
    output reg  [SCORE_W-1:0]       m_score,
    output reg                      m_tlast    // asserted on last center (per-row or per-frame)
);
    localparam integer PACK_W = SCORE_W + 1;  // {strong, score}

    // coords & 1-cycle align for center
    reg  [15:0] x_cur, y_cur, x_d1, y_d1;
    wire center_v = (x_d1 >= 16'd1) && (x_d1 + 16'd1 < frm_w) &&
                    (y_d1 >= 16'd1) && (y_d1 + 16'd1 < frm_h);

    // output skid space -> s_ready
    wire out_can_take = (~m_valid) || (m_valid && m_ready);
    assign s_ready    = out_can_take;

    // === XY tracking ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cur <= 16'd0; y_cur <= 16'd0; x_d1 <= 16'd0; y_d1 <= 16'd0;
        end else if (s_valid && s_ready) begin
            x_cur <= s_x; y_cur <= s_y; x_d1 <= s_x; y_d1 <= s_y;
        end
    end

    // === Line buffers (BRAM, ping-pong) ===
    (* ram_style = "block" *) reg [PACK_W-1:0] lb1 [0:MAX_W-1];
    (* ram_style = "block" *) reg [PACK_W-1:0] lb2 [0:MAX_W-1];
    reg  lb_sel_cur_is_lb1; // 1: current row writes LB1; 0: writes LB2
    reg [15:0] last_y;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_y <= 16'd0; lb_sel_cur_is_lb1 <= 1'b1;
        end else if (s_valid && s_ready) begin
            if (s_sof) begin
                last_y <= s_y; lb_sel_cur_is_lb1 <= 1'b1;
            end else if (s_y != last_y) begin
                last_y <= s_y; lb_sel_cur_is_lb1 <= ~lb_sel_cur_is_lb1;
            end
        end
    end

    reg [PACK_W-1:0] lb1_rd, lb2_rd;
    wire [PACK_W-1:0] cur_pack = {s_is_strong, s_score};

    // read
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lb1_rd <= {PACK_W{1'b0}}; lb2_rd <= {PACK_W{1'b0}};
        end else if (s_valid && s_ready) begin
            if (lb_sel_cur_is_lb1) begin
                lb1_rd <= lb2[s_x]; // y-1
                lb2_rd <= lb1[s_x]; // y-2
            end else begin
                lb1_rd <= lb1[s_x]; // y-1
                lb2_rd <= lb2[s_x]; // y-2
            end
        end
    end

    // write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
        end else if (s_valid && s_ready) begin
            if (lb_sel_cur_is_lb1) lb1[s_x] <= cur_pack;
            else                   lb2[s_x] <= cur_pack;
        end
    end

    // === 3x3 shift registers ===
    reg [PACK_W-1:0] r2_l, r2_c, r2_r;
    reg [PACK_W-1:0] r1_l, r1_c, r1_r;
    reg [PACK_W-1:0] r0_l, r0_c, r0_r;
    wire [PACK_W-1:0] r0_in = cur_pack;
    wire [PACK_W-1:0] r1_in = lb1_rd;
    wire [PACK_W-1:0] r2_in = lb2_rd;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r0_l<=0; r0_c<=0; r0_r<=0; r1_l<=0; r1_c<=0; r1_r<=0; r2_l<=0; r2_c<=0; r2_r<=0;
        end else if (s_valid && s_ready) begin
            r0_l<=r0_c; r0_c<=r0_r; r0_r<=r0_in;
            r1_l<=r1_c; r1_c<=r1_r; r1_r<=r1_in;
            r2_l<=r2_c; r2_c<=r2_r; r2_r<=r2_in;
        end
    end

    // unpack center & neighbors
    wire               c_s_str = r1_c[PACK_W-1];
    wire [SCORE_W-1:0] c_s     = r1_c[SCORE_W-1:0];

    wire [SCORE_W-1:0] n00 = r2_l[SCORE_W-1:0];
    wire [SCORE_W-1:0] n01 = r2_c[SCORE_W-1:0];
    wire [SCORE_W-1:0] n02 = r2_r[SCORE_W-1:0];
    wire [SCORE_W-1:0] n10 = r1_l[SCORE_W-1:0];
    wire [SCORE_W-1:0] n12 = r1_r[SCORE_W-1:0];
    wire [SCORE_W-1:0] n20 = r0_l[SCORE_W-1:0];
    wire [SCORE_W-1:0] n21 = r0_c[SCORE_W-1:0];
    wire [SCORE_W-1:0] n22 = r0_r[SCORE_W-1:0];

    // row max3
    reg [SCORE_W-1:0] row0_max, row1_max, row2_max;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_max<=0; row1_max<=0; row2_max<=0;
        end else if (s_valid && s_ready) begin
            row0_max <= ((n20>n21)?n20:n21) > n22 ? ((n20>n21)?n20:n21) : n22;
            row1_max <= (n10>n12) ? n10 : n12;
            row2_max <= ((n00>n01)?n00:n01) > n02 ? ((n00>n01)?n00:n01) : n02;
        end
    end

    // global max (exclude center) + any_eq
    reg [SCORE_W-1:0] neigh_max_d1; reg any_eq_d1, center_v_d1;
    reg  [15:0]       cx_d1, cy_d1; reg c_s_str_d1; reg [SCORE_W-1:0] c_s_d1;
    wire [SCORE_W-1:0] r02m = (row0_max > row2_max) ? row0_max : row2_max;
    wire [SCORE_W-1:0] gmax = (r02m > row1_max) ? r02m : row1_max;
    wire any_eq_w = (n00==c_s)|(n01==c_s)|(n02==c_s)|(n10==c_s)|(n12==c_s)|(n20==c_s)|(n21==c_s)|(n22==c_s);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            neigh_max_d1<=0; any_eq_d1<=0; center_v_d1<=0;
            cx_d1<=0; cy_d1<=0; c_s_str_d1<=0; c_s_d1<=0;
        end else if (s_valid && s_ready) begin
            neigh_max_d1<=gmax; any_eq_d1<=any_eq_w; center_v_d1<=center_v;
            cx_d1<=x_d1; cy_d1<=y_d1; c_s_str_d1<=c_s_str; c_s_d1<=c_s;
        end
    end

    // keep decision
    reg pass_core;
    always @(*) begin
        pass_core = (c_s_d1 > neigh_max_d1);
        if (!STRICT_GREATER) begin
            case (TIE_MODE)
              1: pass_core = (c_s_d1 > neigh_max_d1) ||
                             ((c_s_d1 == neigh_max_d1) && any_eq_d1 && (~cx_d1[0] & ~cy_d1[0]));
              2: pass_core = (c_s_d1 > neigh_max_d1) ||
                             ((c_s_d1 == neigh_max_d1) && any_eq_d1 && ( cx_d1[0] ^  cy_d1[0]));
              default: pass_core = (c_s_d1 > neigh_max_d1);
            endcase
        end
    end
    wire keep = center_v_d1 && (c_s_d1 >= MIN_SCORE) && pass_core;

    // (-1,-1) + clamp
    wire [15:0] x_m1 = (cx_d1==16'd0) ? 16'd0 : (cx_d1 - 16'd1);
    wire [15:0] y_m1 = (cy_d1==16'd0) ? 16'd0 : (cy_d1 - 16'd1);
    wire [15:0] x_k  = APPLY_NEG1 ? x_m1 : cx_d1;
    wire [15:0] y_k  = APPLY_NEG1 ? y_m1 : cy_d1;
    wire [15:0] x_c  = (CLAMP_MAX && (x_k >= frm_w)) ? (frm_w - 16'd1) : x_k;
    wire [15:0] y_c  = (CLAMP_MAX && (y_k >= frm_h)) ? (frm_h - 16'd1) : y_k;

    // TLAST: per-row or per-frame on last center
    wire last_center_row   = center_v_d1 && (cx_d1 == (frm_w - 16'd2));
    wire last_center_frame = center_v_d1 && (cx_d1 == (frm_w - 16'd2)) && (cy_d1 == (frm_h - 16'd2));
    wire nms_tlast = TLAST_EACH_ROW ? last_center_row : last_center_frame;

    // output skid
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_valid<=1'b0; m_x<=0; m_y<=0; m_is_strong<=1'b0; m_score<=0; m_tlast<=1'b0;
        end else begin
            if (m_valid && m_ready) m_valid <= 1'b0;
            if (keep && ((~m_valid) || m_ready)) begin
                m_valid<=1'b1; m_x<=x_c; m_y<=y_c; m_is_strong<=c_s_str_d1; m_score<=c_s_d1; m_tlast<=nms_tlast;
            end else if (((~m_valid) || m_ready) && nms_tlast && !keep) begin
                m_valid<=1'b1; m_x<=x_c; m_y<=y_c; m_is_strong<=1'b0; m_score<=0; m_tlast<=1'b1;
            end
        end
    end
endmodule
`default_nettype wire
