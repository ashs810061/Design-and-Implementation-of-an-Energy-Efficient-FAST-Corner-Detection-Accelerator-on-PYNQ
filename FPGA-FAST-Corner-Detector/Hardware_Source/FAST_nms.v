`default_nettype none
// ============================================================================
// top_fast_nms_to_dma.v  (64b in/out, VDMA-like)
// - 依輸入 AXIS 產生 TLAST：row-mode (H-2 次) 或 frame-mode (1 次)
// - NMS 專心做角點；TLAST 由 top 注入，避免稀疏流下 NMS 無法判行界
// ============================================================================
module top_fast_nms_to_dma #
(
    parameter integer MAX_W           = 1024,
    parameter integer MAX_H           = 768,

    // ★ TLAST 模式：1=每列（H-2），0=整幀一次
    parameter integer TLAST_EACH_ROW  = 1,

    // FAST params
    parameter [8:0]  FAST_INI_TH      = 9'd20,
    parameter [8:0]  FAST_MIN_TH      = 9'd7,
    parameter integer FAST_MIN_CONTIG = 12,

    // NMS params
    parameter integer NMS_MIN_SCORE   = 1,
    parameter integer NMS_STRICT_GT   = 0,
    parameter integer NMS_TIE_MODE    = 1,
    parameter integer NMS_APPLY_NEG1  = 1,
    parameter integer NMS_CLAMP_MAX   = 1
)
(
    input  wire               aclk,
    input  wire               aresetn,

    // AXIS Video IN (64b)
    input  wire [63:0]        s_axis_tdata,
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,
    input  wire               s_axis_tlast,
    input  wire               s_axis_tuser,   // SOF

    // AXIS OUT (64b)
    output wire [63:0]        m_axis_tdata,
    output wire               m_axis_tvalid,
    input  wire               m_axis_tready,
    output wire               m_axis_tlast,   // 依 TLAST_EACH_ROW 注入
    output wire               m_axis_tuser,   // 幀內第一筆
    output wire [7:0]         m_axis_tkeep    // 8'hFF
);

    // --------------------------------------------------------------------
    // Clock/Reset
    // --------------------------------------------------------------------
    wire clk    = aclk;
    wire resetn = aresetn;

    // --------------------------------------------------------------------
    // 解析輸入（SOF header: W/H） + x/y 計數，專供 TLAST 注入使用
    // --------------------------------------------------------------------
    wire        acc_in   = s_axis_tvalid & s_axis_tready;
    wire [7:0]  in_gray  = s_axis_tdata[7:0];
    wire [15:0] in_cfg_h = s_axis_tdata[23:8];
    wire [15:0] in_cfg_w = s_axis_tdata[39:24];

    reg [15:0] frm_w, frm_h;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            frm_w <= 16'd0; frm_h <= 16'd0;
        end else if (acc_in && s_axis_tuser) begin
            frm_w <= (in_cfg_w > MAX_W[15:0]) ? MAX_W[15:0] : in_cfg_w;
            frm_h <= (in_cfg_h > MAX_H[15:0]) ? MAX_H[15:0] : in_cfg_h;
        end
    end

    // 輸入 x/y 計數
    reg [15:0] in_x, in_y;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            in_x <= 0; in_y <= 0;
        end else if (acc_in) begin
            if (s_axis_tuser) begin
                in_x <= 0; in_y <= 0;
            end else if (s_axis_tlast) begin
                in_x <= 0;
                in_y <= in_y + 16'd1;
            end else begin
                in_x <= in_x + 16'd1;
            end
        end
    end

    // --------------------------------------------------------------------
    // FAST core（事件輸出；稀疏）
    // --------------------------------------------------------------------
    wire        fast_s_tready;
    assign s_axis_tready = fast_s_tready;

    wire        fast_m_valid;
    wire        fast_m_ready;
    wire [15:0] fast_m_x;
    wire [15:0] fast_m_y;
    wire        fast_m_is_strong;
    wire [9:0]  fast_m_score;

    fast9_dualth_event_pix_dyn #(
        .MAX_W          (MAX_W),
        .MAX_H          (MAX_H),
        .INI_TH         (FAST_INI_TH),
        .MIN_TH         (FAST_MIN_TH),
        .MIN_CONTIGUOUS (FAST_MIN_CONTIG)
    ) u_fast (
        .clk            (clk),
        .rst_n          (resetn),
        .cfg_w          (s_axis_tuser ? in_cfg_w : frm_w),
        .cfg_h          (s_axis_tuser ? in_cfg_h : frm_h),
        .s_axis_tdata   (in_gray),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (fast_s_tready),
        .s_axis_tlast   (s_axis_tlast),
        .s_axis_tuser   (s_axis_tuser),
        .m_valid        (fast_m_valid),
        .m_ready        (fast_m_ready),
        .m_x            (fast_m_x),
        .m_y            (fast_m_y),
        .m_is_strong    (fast_m_is_strong),
        .m_score        (fast_m_score)
    );

    // --------------------------------------------------------------------
    // NMS（只做角點；不產生 TLAST）
    // --------------------------------------------------------------------
    wire        nms_s_ready;
    wire        nms_m_valid;
    wire [15:0] nms_m_x, nms_m_y;
    wire        nms_m_is_strong;
    wire [9:0]  nms_m_score;

    assign fast_m_ready = nms_s_ready;

    nms3x3_event_pix_dense #(
        .MAX_W          (MAX_W),
        .SCORE_W        (10),
        .MIN_SCORE      (NMS_MIN_SCORE),
        .STRICT_GREATER (NMS_STRICT_GT),
        .TIE_MODE       (NMS_TIE_MODE),
        .APPLY_NEG1     (NMS_APPLY_NEG1),
        .CLAMP_MAX      (NMS_CLAMP_MAX),
        .TLAST_EACH_ROW (0)              // ★ 關掉；由 top 注入
    ) u_nms (
        .clk        (clk),
        .rst_n      (resetn),
        .s_valid    (fast_m_valid),
        .s_ready    (nms_s_ready),
        .s_x        (fast_m_x),
        .s_y        (fast_m_y),
        .s_score    (fast_m_score),
        .s_is_strong(fast_m_is_strong),
        .s_sof      (s_axis_tuser),
        .frm_w      (frm_w),
        .frm_h      (frm_h),
        .m_valid    (nms_m_valid),
        .m_ready    (1'b1),              // 接到內部仲裁
        .m_x        (nms_m_x),
        .m_y        (nms_m_y),
        .m_is_strong(nms_m_is_strong),
        .m_score    (nms_m_score),
        .m_tlast    ()                   // 忽略
    );

    // --------------------------------------------------------------------
    // TLAST 注入器
    // - row-mode：只在 1..H-2 行尾產生 TLAST（H>=3 時共 H-2 次）
    // - frame-mode：只在最後一列（y==H-1）行尾產生 1 次 TLAST
    // - 若當拍有 NMS 事件，優先送 NMS；TLAST 延後一拍以保持「角點先於行尾」
    // --------------------------------------------------------------------
    reg pending_row_tlast, pending_frame_tlast;

    wire line_end_acc = acc_in & s_axis_tlast;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            pending_row_tlast   <= 1'b0;
            pending_frame_tlast <= 1'b0;
        end else begin
            // 生成
            if (line_end_acc) begin
                if (TLAST_EACH_ROW) begin
                    if ((in_y >= 16'd1) && (in_y + 16'd1 < frm_h))
                        pending_row_tlast <= 1'b1;
                end else begin
                    if (in_y == (frm_h - 16'd1))
                        pending_frame_tlast <= 1'b1;
                end
            end

            // 被送出後清掉（見下方仲裁）
            if (m_axis_tvalid && m_axis_tready && m_axis_tlast && !nms_m_valid) begin
                if (TLAST_EACH_ROW) pending_row_tlast <= 1'b0;
                else                pending_frame_tlast <= 1'b0;
            end

            // 幀起清零
            if (acc_in && s_axis_tuser) begin
                pending_row_tlast   <= 1'b0;
                pending_frame_tlast <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------------------
    // 輸出仲裁：先 NMS，再 TLAST-only
    // --------------------------------------------------------------------
    reg         out_vld_q, out_last_q, out_user_q;
    reg  [63:0] out_data_q;

    // frame user：幀的第一筆輸出
    reg seen_frame_out;
    wire next_user_if_push_nms = (!seen_frame_out) && nms_m_valid;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            seen_frame_out <= 1'b0;
        end else if (acc_in && s_axis_tuser) begin
            seen_frame_out <= 1'b0;
        end else if (m_axis_tvalid && m_axis_tready) begin
            if (!seen_frame_out) seen_frame_out <= 1'b1;
        end
    end

    wire want_tlast_only = TLAST_EACH_ROW ? pending_row_tlast : pending_frame_tlast;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            out_vld_q  <= 1'b0;
            out_last_q <= 1'b0;
            out_user_q <= 1'b0;
            out_data_q <= 64'd0;
        end else begin
            // pop
            if (out_vld_q && m_axis_tready) out_vld_q <= 1'b0;

            // push NMS
            if (!out_vld_q && nms_m_valid) begin
                out_vld_q  <= 1'b1;
                out_last_q <= 1'b0; // NMS 不負責 TLAST
                out_user_q <= next_user_if_push_nms;
                out_data_q <= { nms_m_y, nms_m_x, 21'd0, nms_m_is_strong, nms_m_score };
            end
            // push TLAST-only
            else if (!out_vld_q && want_tlast_only && !nms_m_valid) begin
                out_vld_q  <= 1'b1;
                out_last_q <= 1'b1;
                out_user_q <= 1'b0;
                out_data_q <= 64'd0; // score=0 占位
            end
        end
    end

    assign m_axis_tdata  = out_data_q;
    assign m_axis_tvalid = out_vld_q;
    assign m_axis_tlast  = out_last_q;
    assign m_axis_tuser  = out_user_q;
    assign m_axis_tkeep  = 8'hFF;

endmodule
`default_nettype wire
