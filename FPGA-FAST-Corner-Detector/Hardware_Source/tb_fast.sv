// ===============================================================
// tb_fast_dyn_twoframes.sv
// - 連續餵兩幀：L0(640x428) → L1(320x214)
// - 適用 DUT: fast9_dualth_event_pix_dyn (SOF 鎖 cfg_w/cfg_h)
// - 檔案路徑用正斜線 '/'，避免 Windows 反斜線跳脫問題
// - 輸出即時寫檔，不會漏掉 feeding 期間的 m_valid
// ===============================================================
module tb_fast_dyn_twoframes;

  // ---------- 檔案路徑（改成你本機實際安裝路徑） ----------
  string IN_L0  = "D:/ic_design/vivado_practice/project_new/fast/txt/pyramid_txt_small/pyr_l0_640x428.txt";
  string IN_L1  = "D:/ic_design/vivado_practice/project_new/fast/txt/pyramid_txt_small/pyr_l1_320x214.txt";
  string OUT_L0 = "D:/ic_design/vivado_practice/project_new/fast/txt/pyramid_txt_small/pyr_l0_kp.txt";
  string OUT_L1 = "D:/ic_design/vivado_practice/project_new/fast/txt/pyramid_txt_small/pyr_l1_kp.txt";

  // ---------- 時脈/重設 ----------
  localparam real CLK_NS   = 10.0;    // 100 MHz
  localparam int  RST_CYC  = 10;
  localparam int  DRAIN_CYC= 1024;    // 幀尾等管線排空
  localparam int  FRAME_GAP= 64;      // 幀與幀之間留點空檔（可 0）

  reg clk=0, rst_n=0;
  always #(CLK_NS/2.0) clk = ~clk;
  initial begin
    repeat (RST_CYC) @(posedge clk);
    rst_n = 1;
  end

  // ---------- DUT 介面 ----------
  reg  [15:0] cfg_w, cfg_h;

  reg  [7:0]  s_axis_tdata;
  reg         s_axis_tvalid, s_axis_tlast, s_axis_tuser;
  wire        s_axis_tready;

  wire        m_valid;
  reg         m_ready = 1'b1;
  wire [15:0] m_x, m_y;
  wire        m_is_strong;
  wire [9:0]  m_score;

  // ---------- 實例化 DUT ----------
  // MAX_W/MAX_H 至少要覆蓋 L0 尺寸
  fast9_dualth_event_pix_dyn #(
    .MAX_W(640), .MAX_H(428),
    .INI_TH(20), .MIN_TH(7), .MIN_CONTIGUOUS(12)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .cfg_w(cfg_w), .cfg_h(cfg_h),
    .s_axis_tdata(s_axis_tdata),
    .s_axis_tvalid(s_axis_tvalid),
    .s_axis_tready(s_axis_tready),
    .s_axis_tlast(s_axis_tlast),
    .s_axis_tuser(s_axis_tuser),
    .m_valid(m_valid),
    .m_ready(m_ready),
    .m_x(m_x), .m_y(m_y),
    .m_is_strong(m_is_strong),
    .m_score(m_score)
  );

  // ---------- 目前輸出檔案（即時寫入） ----------
  integer fout = 0;
  task open_out(input string path);
    begin
      if (fout != 0) $fclose(fout);
      fout = $fopen(path, "w");
      if (fout == 0) begin
        $display("[%0t] ERROR: cannot open output %s", $time, path);
        $finish;
      end
      $fwrite(fout, "# x y strong score\n");
    end
  endtask
  task close_out; begin if (fout!=0) begin $fclose(fout); fout=0; end end endtask

  // 串流輸出即時寫檔（不漏拍）
  always @(posedge clk) begin
    if (m_valid && m_ready && fout!=0)
      $fwrite(fout, "%0d %0d %0d %0d\n", m_x, m_y, m_is_strong, m_score);
  end

  // ---------- 餵一幀（讀 txt → AXI-S） ----------
  task feed_frame(input string in_path, input string out_path);
    integer fin, W, H, rv, pix, x, y, accepted, gap;
    begin
      // 讀入
      fin = $fopen(in_path, "r");
      if (fin==0) begin
        $display("[%0t] ERROR: cannot open input %s", $time, in_path);
        $finish;
      end
      rv = $fscanf(fin, "%d %d\n", W, H);
      if (rv!=2) begin
        $display("[%0t] ERROR: bad header in %s", $time, in_path);
        $finish;
      end
      if (W>640 || H>428) begin
        $display("[%0t] ERROR: frame %0dx%0d exceeds MAX 640x428", $time, W, H);
        $finish;
      end

      // 開啟對應輸出檔
      open_out(out_path);

      // SOF：同拍設定 cfg_w/h 與 tuser=1
      cfg_w <= W; cfg_h <= H;
      s_axis_tuser  <= 1'b1;
      s_axis_tvalid <= 1'b0;
      s_axis_tlast  <= 1'b0;
      @(posedge clk);

      // 逐像素餵
      for (y=0; y<H; y=y+1) begin
        for (x=0; x<W; x=x+1) begin
          rv = $fscanf(fin, "%d", pix);
          if (rv!=1) begin
            $display("[%0t] ERROR: EOF at (%0d,%0d) in %s", $time, x, y, in_path);
            $finish;
          end
          s_axis_tdata  <= pix[7:0];
          s_axis_tvalid <= 1'b1;
          s_axis_tlast  <= (x==W-1);

          accepted = 0;
          // 嚴格握手，直到 tvalid&&tready
          while (!accepted) begin
            @(posedge clk);
            if (s_axis_tvalid && s_axis_tready) begin
              // 第一顆像素送出後拉回 tuser=0
              s_axis_tuser <= 1'b0;
              accepted = 1;
            end
          end
        end
        // 行尾拉低 valid/last 一拍
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
        @(posedge clk);
      end
      $fclose(fin);

      // 幀尾：等待管線排空
      gap = 0;
      while (gap < DRAIN_CYC) begin
        @(posedge clk);
        if (m_valid) gap = 0; else gap = gap + 1;
      end
      // 關檔
      close_out();

      // 幀與幀之間留一點空檔（避免切檔瞬間交錯）
      repeat (FRAME_GAP) @(posedge clk);
    end
  endtask

  // ---------- 主流程 ----------
  initial begin
    s_axis_tdata  = 8'd0;
    s_axis_tvalid = 1'b0;
    s_axis_tlast  = 1'b0;
    s_axis_tuser  = 1'b0;
    cfg_w = 0; cfg_h = 0;

    @(posedge rst_n);
    repeat (5) @(posedge clk);

    $display("[%0t] Feed L0 ...", $time);
    feed_frame(IN_L0, OUT_L0);

    $display("[%0t] Feed L1 ...", $time);
    feed_frame(IN_L1, OUT_L1);

    $display("[%0t] TB done. OUT0=%s, OUT1=%s", $time, OUT_L0, OUT_L1);
    $finish;
  end

endmodule
