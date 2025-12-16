// ===============================================================
// fast9_dualth_event_pix_dyn.v  (Verilog-2001, 3-stage, BRAM line buffers)
// 修正：
//  - S2 無論 window_ok 與否都要「消耗」token，避免卡死
//  - S1 similar 計數改用 blocking 暫存累加，最後一次性 <=
//  - AXIS tready 採 bubble-free hand-shake：~pend_v && (~s0_v || s1_take)
// ===============================================================
module fast9_dualth_event_pix_dyn #
(
    parameter integer MAX_W          = 1024,
    parameter integer MAX_H          = 768,
    parameter [8:0]  INI_TH          = 9'd20,
    parameter [8:0]  MIN_TH          = 9'd7,
    parameter integer MIN_CONTIGUOUS = 12
)
(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] cfg_w,
    input  wire [15:0] cfg_h,

    // AXI4-Stream Video in
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tuser,

    // every-pixel out (ready/valid)
    output reg         m_valid,
    input  wire        m_ready,
    output reg  [15:0] m_x,
    output reg  [15:0] m_y,
    output reg         m_is_strong,
    output reg  [9:0]  m_score
);

    // ---------------- constants / helpers ----------------
    localparam integer RADIUS = 3;
    localparam integer HDELAY = 3;

    function integer CLOG2; input integer v; integer i; begin
        i=0; v=v-1; for (i=0; v>0; i=i+1) v=v>>1; CLOG2=i;
    end endfunction
    localparam integer COL_W = CLOG2(MAX_W);
    localparam integer ROW_W = CLOG2(MAX_H);

    function [15:0] zext16_col; input [COL_W-1:0] v; begin
        if (16>COL_W) zext16_col = { {(16-COL_W){1'b0}}, v }; else zext16_col = v[15:0];
    end endfunction
    function [15:0] zext16_row; input [ROW_W-1:0] v; begin
        if (16>ROW_W) zext16_row = { {(16-ROW_W){1'b0}}, v }; else zext16_row = v[15:0];
    end endfunction

    // ========================= 幀/座標/狀態 =========================
    reg [15:0] frm_w, frm_h;
    reg [COL_W-1:0] col;
    reg [ROW_W-1:0] row;

    // output skid
    reg        pend_v;
    reg [15:0] pend_x, pend_y;
    reg        pend_is_strong;
    reg [9:0]  pend_score;

    // pipeline valid
    reg s0_v, s1_v, s2_v;

    // hand-shake between stages
    wire s1_take = s0_v && !s1_v;      // S1 takes S0 this cycle
    wire s2_take = s1_v && !s2_v;      // S2 takes S1 this cycle
    wire out_take = s2_v && !pend_v;   // output stage takes S2 this cycle

    // tready：S0 空或同拍被 S1 取走，且沒有 pending
    assign s_axis_tready = ~pend_v && (~s0_v || s1_take);

    wire acc = s_axis_tvalid & s_axis_tready;

    // 被接受的 TLAST / SOF 脈衝
    reg eol_acc, sof_acc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin eol_acc <= 1'b0; sof_acc <= 1'b0; end
        else begin
            eol_acc <= acc & s_axis_tlast;
            sof_acc <= acc & s_axis_tuser;
        end
    end

    // ========================= 7-bank BRAM ring buffer =========================
    localparam integer AW = COL_W;

    reg  [2:0]    bank_wr;
    reg  [AW-1:0] rd_addr, wr_addr;

    wire [7:0] bram_dout0, bram_dout1, bram_dout2, bram_dout3, bram_dout4, bram_dout5, bram_dout6;

    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b0 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout0), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd0)) );
    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b1 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout1), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd1)) );
    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b2 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout2), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd2)) );
    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b3 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout3), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd3)) );
    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b4 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout4), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd4)) );
    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b5 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout5), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd5)) );
    bram_1r1w_sdpram #(.DATA_WIDTH(8), .ADDR_WIDTH(AW), .DEPTH(MAX_W)) u_b6 ( .clk(clk), .raddr(rd_addr), .dout(bram_dout6), .waddr(wr_addr), .din(s_axis_tdata), .we(acc & (bank_wr==3'd6)) );

    // 把 7 個 bank 旋轉成 y, y-1, ... , y-6
    reg [7:0] rdata0, rdata1, rdata2, rdata3, rdata4, rdata5, rdata6;
    always @* begin
        case (bank_wr)
          3'd0: begin rdata0=bram_dout0; rdata1=bram_dout6; rdata2=bram_dout5; rdata3=bram_dout4; rdata4=bram_dout3; rdata5=bram_dout2; rdata6=bram_dout1; end
          3'd1: begin rdata0=bram_dout1; rdata1=bram_dout0; rdata2=bram_dout6; rdata3=bram_dout5; rdata4=bram_dout4; rdata5=bram_dout3; rdata6=bram_dout2; end
          3'd2: begin rdata0=bram_dout2; rdata1=bram_dout1; rdata2=bram_dout0; rdata3=bram_dout6; rdata4=bram_dout5; rdata5=bram_dout4; rdata6=bram_dout3; end
          3'd3: begin rdata0=bram_dout3; rdata1=bram_dout2; rdata2=bram_dout1; rdata3=bram_dout0; rdata4=bram_dout6; rdata5=bram_dout5; rdata6=bram_dout4; end
          3'd4: begin rdata0=bram_dout4; rdata1=bram_dout3; rdata2=bram_dout2; rdata3=bram_dout1; rdata4=bram_dout0; rdata5=bram_dout6; rdata6=bram_dout5; end
          3'd5: begin rdata0=bram_dout5; rdata1=bram_dout4; rdata2=bram_dout3; rdata3=bram_dout2; rdata4=bram_dout1; rdata5=bram_dout0; rdata6=bram_dout6; end
          default: begin rdata0=bram_dout6; rdata1=bram_dout5; rdata2=bram_dout4; rdata3=bram_dout3; rdata4=bram_dout2; rdata5=bram_dout1; rdata6=bram_dout0; end
        endcase
    end

    // 座標/位址/列輪轉（僅在 acc 時推進）
    wire [15:0] cfg_w_clip = (cfg_w > MAX_W) ? MAX_W[15:0] : cfg_w;
    wire [15:0] cfg_h_clip = (cfg_h > MAX_H) ? MAX_H[15:0] : cfg_h;
    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        frm_w<=0; frm_h<=0; col<={COL_W{1'b0}}; row<={ROW_W{1'b0}};
        bank_wr<=3'd0; rd_addr<={AW{1'b0}}; wr_addr<={AW{1'b0}};
      end else if (acc) begin
        if (s_axis_tuser) begin
          frm_w <= (cfg_w_clip < (RADIUS+HDELAY+1)) ? (RADIUS+HDELAY+1) : cfg_w_clip;
          frm_h <= (cfg_h_clip < (2*RADIUS+1))     ? (2*RADIUS+1)     : cfg_h_clip;
          col   <= {COL_W{1'b0}};
          row   <= {ROW_W{1'b0}};
          rd_addr <= {AW{1'b0}};
          wr_addr <= {AW{1'b0}};
          bank_wr <= 3'd0;
        end else begin
          if (col != frm_w[COL_W-1:0]-1) begin
            col     <= col + 1'b1;
            rd_addr <= rd_addr + 1'b1;
            wr_addr <= wr_addr + 1'b1;
          end else begin
            col     <= {COL_W{1'b0}};
            rd_addr <= {AW{1'b0}};
            wr_addr <= {AW{1'b0}};
            if (row != frm_h[ROW_W-1:0]-1) row <= row + 1'b1;
            bank_wr <= (bank_wr==3'd6) ? 3'd0 : (bank_wr + 3'd1);
          end
        end
      end
    end

    // ========================= 7x7 window via shift rows =========================
    reg [7:0] win0 [0:6], win1 [0:6], win2 [0:6], win3 [0:6], win4 [0:6], win5 [0:6], win6 [0:6];
    integer c;
    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        for (c=0;c<7;c=c+1) begin
          win0[c]<=0; win1[c]<=0; win2[c]<=0; win3[c]<=0; win4[c]<=0; win5[c]<=0; win6[c]<=0;
        end
      end else if (acc) begin
        win0[0]<=win0[1]; win0[1]<=win0[2]; win0[2]<=win0[3]; win0[3]<=win0[4]; win0[4]<=win0[5]; win0[5]<=win0[6]; win0[6]<=rdata0;
        win1[0]<=win1[1]; win1[1]<=win1[2]; win1[2]<=win1[3]; win1[3]<=win1[4]; win1[4]<=win1[5]; win1[5]<=win1[6]; win1[6]<=rdata1;
        win2[0]<=win2[1]; win2[1]<=win2[2]; win2[2]<=win2[3]; win2[3]<=win2[4]; win2[4]<=win2[5]; win2[5]<=win2[6]; win2[6]<=rdata2;
        win3[0]<=win3[1]; win3[1]<=win3[2]; win3[2]<=win3[3]; win3[3]<=win3[4]; win3[4]<=win3[5]; win3[5]<=win3[6]; win3[6]<=rdata3;
        win4[0]<=win4[1]; win4[1]<=win4[2]; win4[2]<=win4[3]; win4[3]<=win4[4]; win4[4]<=win4[5]; win4[5]<=win4[6]; win4[6]<=rdata4;
        win5[0]<=win5[1]; win5[1]<=win5[2]; win5[2]<=win5[3]; win5[3]<=win5[4]; win5[4]<=win5[5]; win5[5]<=win5[6]; win5[6]<=rdata5;
        win6[0]<=win6[1]; win6[1]<=win6[2]; win6[2]<=win6[3]; win6[3]<=win6[4]; win6[4]<=win6[5]; win6[5]<=win6[6]; win6[6]<=rdata6;
      end
    end

    function [7:0] tap; input integer dx; input integer dy; reg [7:0] v; begin
      case (dy)
        -3: v = win6[dx+3];
        -2: v = win5[dx+3];
        -1: v = win4[dx+3];
         0: v = win3[dx+3];
         1: v = win2[dx+3];
         2: v = win1[dx+3];
         3: v = win0[dx+3];
        default: v = 8'd0;
      endcase
      tap = v;
    end endfunction

    function integer dx_lut; input integer i; begin
      case (i)
         0:dx_lut= 0;  1:dx_lut= 1;  2:dx_lut= 2;  3:dx_lut= 3;
         4:dx_lut= 3;  5:dx_lut= 3;  6:dx_lut= 2;  7:dx_lut= 1;
         8:dx_lut= 0;  9:dx_lut=-1; 10:dx_lut=-2; 11:dx_lut=-3;
        12:dx_lut=-3; 13:dx_lut=-3; 14:dx_lut=-2; 15:dx_lut=-1;
        default: dx_lut=0;
      endcase
    end endfunction
    function integer dy_lut; input integer i; begin
      case (i)
         0:dy_lut=-3;  1:dy_lut=-3;  2:dy_lut=-2;  3:dy_lut=-1;
         4:dy_lut= 0;  5:dy_lut= 1;  6:dy_lut= 2;  7:dy_lut= 3;
         8:dy_lut= 3;  9:dy_lut= 3; 10:dy_lut= 2; 11:dy_lut= 1;
        12:dy_lut= 0; 13:dy_lut=-1; 14:dy_lut=-2; 15:dy_lut=-3;
        default: dy_lut=0;
      endcase
    end endfunction

    // window 可用
    wire window_ok_now =
      (frm_w >= (RADIUS+HDELAY+1)) && (frm_h >= (2*RADIUS+1)) &&
      (row    >= (2*RADIUS)) &&
      (col    >= (HDELAY + RADIUS)) &&
      (col    <  frm_w);

    // ========================= S0 =========================
    reg [7:0]  s0_Ic;
    reg [7:0]  s0_P [0:15];
    reg [15:0] s0_x, s0_y;
    reg        s0_window_ok;
    integer k;

    localparam [COL_W-1:0] HDELAY_W = HDELAY;
    localparam [ROW_W-1:0] RADIUS_W = RADIUS;
    wire [COL_W-1:0] col_m_h = col - HDELAY_W;
    wire [ROW_W-1:0] row_m_r = row - RADIUS_W;

    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        s0_v <= 1'b0; s0_Ic<=0; s0_x<=0; s0_y<=0; s0_window_ok<=1'b0;
        for (k=0;k<16;k=k+1) s0_P[k] <= 8'd0;
      end else begin
        if (sof_acc) s0_v <= 1'b0; // 新幀清管
        if (acc) begin
          s0_x <= zext16_col(col_m_h);
          s0_y <= zext16_row(row_m_r);
          s0_window_ok <= window_ok_now;
          if (window_ok_now) begin
            s0_Ic <= tap(0,0);
            for (k=0;k<16;k=k+1) s0_P[k] <= tap(dx_lut(k), dy_lut(k));
          end else begin
            s0_Ic <= 8'd0;
            for (k=0;k<16;k=k+1) s0_P[k] <= 8'd0;
          end
          s0_v <= 1'b1;
        end else if (s1_take) begin
          s0_v <= 1'b0;
        end
      end
    end

    // ========================= S1 =========================
    reg [7:0]  s1_D [0:15];
    reg [15:0] s1_mb_s, s1_md_s, s1_mb_w, s1_md_w;
    reg [4:0]  s1_sim_s, s1_sim_w;
    reg [15:0] s1_x, s1_y;
    reg        s1_window_ok;

    wire [9:0] TH_STR = {1'b0, INI_TH};
    wire [9:0] TH_WEAK= {1'b0, MIN_TH};
    reg  [9:0] Ic10, P10;

    integer sim_s_n, sim_w_n;
    reg [15:0] mb_s_n, md_s_n, mb_w_n, md_w_n;

    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        s1_v<=1'b0; s1_x<=0; s1_y<=0; s1_window_ok<=1'b0;
        s1_mb_s<=0; s1_md_s<=0; s1_mb_w<=0; s1_md_w<=0; s1_sim_s<=5'd0; s1_sim_w<=5'd0;
        for (k=0;k<16;k=k+1) s1_D[k]<=0;
      end else begin
        if (sof_acc) s1_v <= 1'b0;

        if (s1_take) begin
          // |P-Ic|
          for (k=0;k<16;k=k+1)
            s1_D[k] <= (s0_P[k] >= s0_Ic) ? (s0_P[k]-s0_Ic) : (s0_Ic-s0_P[k]);

          // masks + similar（用 blocking 累加）
          mb_s_n = 16'd0; md_s_n = 16'd0; mb_w_n = 16'd0; md_w_n = 16'd0;
          sim_s_n = 0;     sim_w_n = 0;
          Ic10   = {2'b00, s0_Ic};
          for (k=0;k<16;k=k+1) begin
            P10 = {2'b00, s0_P[k]};
            if      (P10 >= Ic10 + TH_STR) mb_s_n[k] = 1'b1;
            else if (P10 + TH_STR <= Ic10) md_s_n[k] = 1'b1;
            else                           sim_s_n = sim_s_n + 1;

            if      (P10 >= Ic10 + TH_WEAK) mb_w_n[k] = 1'b1;
            else if (P10 + TH_WEAK <= Ic10) md_w_n[k] = 1'b1;
            else                            sim_w_n = sim_w_n + 1;
          end
          s1_mb_s  <= mb_s_n;  s1_md_s <= md_s_n;
          s1_mb_w  <= mb_w_n;  s1_md_w <= md_w_n;
          s1_sim_s <= sim_s_n[4:0];
          s1_sim_w <= sim_w_n[4:0];

          s1_x <= s0_x; s1_y <= s0_y; s1_window_ok <= s0_window_ok;
          s1_v <= 1'b1;
        end else if (s2_take) begin
          s1_v <= 1'b0;
        end
      end
    end

    // ========================= S2 + 出口 =========================
// 平衡 AND，把 >=N 命中算出（Verilog-2001 版，無 +=、&=）
function [31:0] hits_from_mask_balanced;
  input [15:0] m; input integer N;
  reg [31:0] v, a0, a1, a2, a3, a4, res;
  integer shift;
  reg [4:0] nb;
begin
  v  = {m,m};
  a0 = v;                   // >=1
  a1 = a0 & (a0 << 1);      // >=2
  a2 = a1 & (a1 << 2);      // >=4
  a3 = a2 & (a2 << 4);      // >=8
  a4 = a3 & (a3 << 8);      // >=16

  nb    = N[4:0];
  res   = 32'hFFFF_FFFF;
  shift = 0;

  if (nb[0]) begin
    res   = res & (a0 << shift);
    shift = shift + 1;
  end
  if (nb[1]) begin
    res   = res & (a1 << shift);
    shift = shift + 2;
  end
  if (nb[2]) begin
    res   = res & (a2 << shift);
    shift = shift + 4;
  end
  if (nb[3]) begin
    res   = res & (a3 << shift);
    shift = shift + 8;
  end
  if (nb[4]) begin
    res   = res & (a4 << shift);
    shift = shift + 16;
  end

  hits_from_mask_balanced = res;
end
endfunction


    // S2 暫存
    reg [15:0] s2_x, s2_y;
    reg        s2_window_ok;
    reg [7:0]  s2_score_s8, s2_score_w8;

    integer j;
    reg [31:0] hit_s_b32, hit_s_d32, hit_w_b32, hit_w_d32;
    reg [15:0] hit_s_b,  hit_s_d,  hit_w_b,  hit_w_d;
    reg [7:0]  best_b_s, best_d_s, minrun_s, score_s8;
    reg [7:0]  best_b_w, best_d_w, minrun_w, score_w8;

    // 12 個數最小值（balanced）
    function [7:0] min12;
      input [7:0] d0,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11;
      reg [7:0] a01,a23,a45,a67,a89,a1011,m4_0,m4_1,m8,m12;
      begin
        a01=(d0<d1)?d0:d1; a23=(d2<d3)?d2:d3; a45=(d4<d5)?d4:d5; a67=(d6<d7)?d6:d7;
        a89=(d8<d9)?d8:d9; a1011=(d10<d11)?d10:d11;
        m4_0=(a01<a23)?a01:a23; m4_1=(a45<a67)?a45:a67;
        m8=(m4_0<m4_1)?m4_0:m4_1;
        m12=(m8<(a89<a1011?a89:a1011))?m8:(a89<a1011?a89:a1011);
        min12=m12;
      end
    endfunction

    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        s2_v<=1'b0; s2_x<=0; s2_y<=0; s2_window_ok<=1'b0; s2_score_s8<=0; s2_score_w8<=0;
        m_valid<=0; m_x<=0; m_y<=0; m_is_strong<=0; m_score<=0;
        pend_v<=0; pend_x<=0; pend_y<=0; pend_is_strong<=0; pend_score<=0;
      end else begin
        if (m_ready) m_valid <= 1'b0;
        if (sof_acc) begin s2_v<=1'b0; pend_v<=1'b0; end

        // 接收 S1 → 做 hits/score
        if (s2_take) begin
          hit_s_b32 = hits_from_mask_balanced(s1_mb_s, MIN_CONTIGUOUS);
          hit_s_d32 = hits_from_mask_balanced(s1_md_s, MIN_CONTIGUOUS);
          hit_w_b32 = hits_from_mask_balanced(s1_mb_w, MIN_CONTIGUOUS);
          hit_w_d32 = hits_from_mask_balanced(s1_md_w, MIN_CONTIGUOUS);
          hit_s_b   = hit_s_b32[15:0]; hit_s_d = hit_s_d32[15:0];
          hit_w_b   = hit_w_b32[15:0]; hit_w_d = hit_w_d32[15:0];

          // strong
          best_b_s=0; best_d_s=0; score_s8=0;
          if (s1_window_ok && (s1_sim_s<=5'd4) && (|hit_s_b || |hit_s_d)) begin
            if (|hit_s_b) for (j=0;j<16;j=j+1) if (hit_s_b[j]) begin
              minrun_s = min12(
                s1_D[(j    )&15], s1_D[(j+ 1)&15], s1_D[(j+ 2)&15], s1_D[(j+ 3)&15],
                s1_D[(j+ 4)&15], s1_D[(j+ 5)&15], s1_D[(j+ 6)&15], s1_D[(j+ 7)&15],
                s1_D[(j+ 8)&15], s1_D[(j+ 9)&15], s1_D[(j+10)&15], s1_D[(j+11)&15]
              );
              if (minrun_s > best_b_s) best_b_s = minrun_s;
            end
            if (|hit_s_d) for (j=0;j<16;j=j+1) if (hit_s_d[j]) begin
              minrun_s = min12(
                s1_D[(j    )&15], s1_D[(j+ 1)&15], s1_D[(j+ 2)&15], s1_D[(j+ 3)&15],
                s1_D[(j+ 4)&15], s1_D[(j+ 5)&15], s1_D[(j+ 6)&15], s1_D[(j+ 7)&15],
                s1_D[(j+ 8)&15], s1_D[(j+ 9)&15], s1_D[(j+10)&15], s1_D[(j+11)&15]
              );
              if (minrun_s > best_d_s) best_d_s = minrun_s;
            end
            score_s8 = (best_b_s > best_d_s) ? best_b_s : best_d_s;
          end

          // weak
          best_b_w=0; best_d_w=0; score_w8=0;
          if (s1_window_ok && (s1_sim_w<=5'd4) && (|hit_w_b || |hit_w_d)) begin
            if (|hit_w_b) for (j=0;j<16;j=j+1) if (hit_w_b[j]) begin
              minrun_w = min12(
                s1_D[(j    )&15], s1_D[(j+ 1)&15], s1_D[(j+ 2)&15], s1_D[(j+ 3)&15],
                s1_D[(j+ 4)&15], s1_D[(j+ 5)&15], s1_D[(j+ 6)&15], s1_D[(j+ 7)&15],
                s1_D[(j+ 8)&15], s1_D[(j+ 9)&15], s1_D[(j+10)&15], s1_D[(j+11)&15]
              );
              if (minrun_w > best_b_w) best_b_w = minrun_w;
            end
            if (|hit_w_d) for (j=0;j<16;j=j+1) if (hit_w_d[j]) begin
              minrun_w = min12(
                s1_D[(j    )&15], s1_D[(j+ 1)&15], s1_D[(j+ 2)&15], s1_D[(j+ 3)&15],
                s1_D[(j+ 4)&15], s1_D[(j+ 5)&15], s1_D[(j+ 6)&15], s1_D[(j+ 7)&15],
                s1_D[(j+ 8)&15], s1_D[(j+ 9)&15], s1_D[(j+10)&15], s1_D[(j+11)&15]
              );
              if (minrun_w > best_d_w) best_d_w = minrun_w;
            end
            score_w8 = (best_b_w > best_d_w) ? best_b_w : best_d_w;
          end

          // 寫回 S2 暫存
          s2_x <= s1_x; s2_y <= s1_y;
          s2_window_ok <= s1_window_ok;
          s2_score_s8 <= score_s8; s2_score_w8 <= score_w8;
          s2_v <= 1'b1;
        end

        // 出/掛起：**無論 window_ok 與否都要消耗 token**
        if (out_take) begin
          if (s2_window_ok) begin
            if (m_ready) begin
              m_valid <= 1'b1; m_x <= s2_x; m_y <= s2_y;
              if (s2_score_s8!=0) begin m_is_strong<=1'b1; m_score<={2'b00,s2_score_s8}; end
              else                 begin m_is_strong<=1'b0; m_score<={2'b00,s2_score_w8}; end
            end else begin
              pend_v  <= 1'b1; pend_x <= s2_x; pend_y <= s2_y;
              if (s2_score_s8!=0) begin pend_is_strong<=1'b1; pend_score<={2'b00,s2_score_s8}; end
              else                 begin pend_is_strong<=1'b0; pend_score<={2'b00,s2_score_w8}; end
            end
          end
          s2_v <= 1'b0; // consume anyway
        end

        // 吐 pending
        if (pend_v && m_ready) begin
          m_valid     <= 1'b1;
          m_x         <= pend_x;
          m_y         <= pend_y;
          m_is_strong <= pend_is_strong;
          m_score     <= pend_score;
          pend_v      <= 1'b0;
        end
      end
    end

endmodule

// ===============================================================
// Inferred BRAM: 1R1W Simple Dual-Port, synchronous read (1-cycle)
// ===============================================================
module bram_1r1w_sdpram #(
  parameter integer DATA_WIDTH = 8,
  parameter integer ADDR_WIDTH = 10,
  parameter integer DEPTH      = 1024
)(
  input  wire                     clk,
  input  wire [ADDR_WIDTH-1:0]    raddr,
  output reg  [DATA_WIDTH-1:0]    dout,
  input  wire [ADDR_WIDTH-1:0]    waddr,
  input  wire [DATA_WIDTH-1:0]    din,
  input  wire                     we
);
  (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
  always @(posedge clk) begin
    if (we) mem[waddr] <= din;
    dout <= mem[raddr];   // 同步讀，1-cycle latency
  end
endmodule
