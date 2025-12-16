module tb_top_fast_nms_to_dma;

  // =========================
  // Clock / Reset
  // =========================
  localparam real CLK_PERIOD = 10.0; // 100 MHz
  reg clk = 0;
  always #(CLK_PERIOD/2.0) clk = ~clk;

  reg rst_n = 0;
  initial begin
    rst_n = 0;
    repeat (20) @(posedge clk);
    rst_n = 1;
  end

  // =========================
  // TB knobs (plusargs)
  // =========================
  string IN_PATH  = "D:/ic_design/vivado_practice/project_new/fast_nms/txt/pyramid_txt_small/pyr_l0_640x428.txt";
  string OUT_PATH = "D:/ic_design/vivado_practice/project_new/fast_nms/txt/pyramid_txt_small/pyr_l0_kp.txt";
  int    OUT_TLAST_MODE = 1;   // 0: frame; 1: center-rows(H-2); 2: rows(H)
  int    SINK_STALL_PCT = 0;   // 0..100
  int    SINK_STALL_MIN = 2;
  int    SINK_STALL_MAX = 8;
  int    SRC_IDLE_PCT   = 0;   // 0..100 (MM2S 間歇 idle 機率)

  initial begin
    void'($value$plusargs("IN=%s",  IN_PATH));
    void'($value$plusargs("OUT=%s", OUT_PATH));
    void'($value$plusargs("OUT_TLAST_MODE=%d", OUT_TLAST_MODE));
    void'($value$plusargs("SINK_STALL_PCT=%d", SINK_STALL_PCT));
    void'($value$plusargs("SRC_IDLE_PCT=%d",  SRC_IDLE_PCT));
    $display("[%0t] IN=%s OUT=%s OUT_TLAST_MODE=%0d SINK_STALL_PCT=%0d SRC_IDLE_PCT=%0d",
             $time, IN_PATH, OUT_PATH, OUT_TLAST_MODE, SINK_STALL_PCT, SRC_IDLE_PCT);
  end

  // =========================
  // DUT AXIS ports (64-bit in/out)
  // =========================
  reg  [63:0] s_axis_tdata;
  reg         s_axis_tvalid;
  wire        s_axis_tready;
  reg         s_axis_tlast;
  reg         s_axis_tuser; // SOF

  wire [63:0] m_axis_tdata;
  wire        m_axis_tvalid;
  reg         m_axis_tready;
  wire        m_axis_tlast;
  wire        m_axis_tuser; // SOF (若 DUT 有輸出)
  wire [7:0]  m_axis_tkeep; // 應為 8'hFF

  // =========================
  // Instantiate DUT
  // =========================
  top_fast_nms_to_dma #(
    .MAX_W(1024), .MAX_H(768),
    .TLAST_EACH_ROW(1),
    .FAST_INI_TH(9'd20), .FAST_MIN_TH(9'd7), .FAST_MIN_CONTIG(12),
    .NMS_MIN_SCORE(1),
    .NMS_STRICT_GT(0),       // 建議 tie 模式
    .NMS_TIE_MODE(1),        // even-even
    .NMS_APPLY_NEG1(1),      // NMS 後做 (-1,-1)
    .NMS_CLAMP_MAX(1)
  ) dut (
    .aclk           (clk),
    .aresetn        (rst_n),
    // AXIS in
    .s_axis_tdata   (s_axis_tdata),
    .s_axis_tvalid  (s_axis_tvalid),
    .s_axis_tready  (s_axis_tready),
    .s_axis_tlast   (s_axis_tlast),
    .s_axis_tuser   (s_axis_tuser),
    // AXIS out
    .m_axis_tdata   (m_axis_tdata),
    .m_axis_tvalid  (m_axis_tvalid),
    .m_axis_tready  (m_axis_tready),
    .m_axis_tlast   (m_axis_tlast),
    .m_axis_tuser   (m_axis_tuser),
    .m_axis_tkeep   (m_axis_tkeep)
  );

  // =========================
  // S2MM Sink（VDMA 接收）
  // =========================
  int cyc, stall_rem;
  initial begin
    m_axis_tready = 0;
    cyc = 0; stall_rem = 0;
    wait(rst_n);
    @(posedge clk);
    m_axis_tready = 1;
  end

  function automatic bit rand_hit(input int pct);
    rand_hit = ($urandom_range(0,99) < pct);
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      cyc <= 0; stall_rem <= 0; m_axis_tready <= 0;
    end else begin
      cyc <= cyc + 1;
      if (stall_rem > 0) begin
        stall_rem   <= stall_rem - 1;
        m_axis_tready <= 0;
      end else if (rand_hit(SINK_STALL_PCT)) begin
        stall_rem   <= $urandom_range(SINK_STALL_MIN, SINK_STALL_MAX);
        m_axis_tready <= 0;
      end else begin
        m_axis_tready <= 1;
      end
    end
  end

  // =========================
  // Output monitor / file dump
  // =========================
  integer fout;
  integer out_total, out_keep, out_rows, out_frames;

  // 解包 64b：{ y[63:48], x[47:32], 21b 0, is_strong[10], score[9:0] }
  wire [15:0] o_y     = m_axis_tdata[63:48];
  wire [15:0] o_x     = m_axis_tdata[47:32];
  wire        o_str   = m_axis_tdata[10];
  wire [9:0]  o_score = m_axis_tdata[9:0];

  // tkeep 必須 0xFF
  always @(posedge clk) begin
    if (rst_n && m_axis_tvalid && m_axis_tready) begin
      if (m_axis_tkeep !== 8'hFF) begin
        $error("[%0t] m_axis_tkeep != 8'hFF (got %02x)", $time, m_axis_tkeep);
        $stop;
      end
    end
  end

  // 統計輸出
  always @(posedge clk) begin
    if (!rst_n) begin
      out_total <= 0; out_keep <= 0; out_rows <= 0; out_frames <= 0;
    end else if (m_axis_tvalid && m_axis_tready) begin
      if (fout) $fwrite(fout, "%0d %0d %0d %0d\n", o_x, o_y, o_str, o_score);
      out_total <= out_total + 1;
      if (o_score != 0) out_keep <= out_keep + 1; // 0 分數是「未 keep 但打 TLAST」的佔位
      if (m_axis_tlast) out_rows <= out_rows + 1;
      if (m_axis_tuser) out_frames <= out_frames + 1;
    end
  end

  // =========================
  // MM2S Source（VDMA 傳輸）
  // =========================
  task automatic axis_idle();
    begin
      s_axis_tvalid <= 1'b0;
      s_axis_tlast  <= 1'b0;
      s_axis_tuser  <= 1'b0;
      s_axis_tdata  <= 64'd0;
      @(posedge clk);
    end
  endtask

  // 單拍送資料（阻塞至握手）
  task automatic axis_send_beat(input [63:0] tdata,
                                input        tlast,
                                input        tuser);
    begin
      // 模擬 MM2S 可能的 idle 插空
      if (rand_hit(SRC_IDLE_PCT)) repeat ($urandom_range(1,3)) @(posedge clk);

      s_axis_tdata  <= tdata;
      s_axis_tlast  <= tlast;
      s_axis_tuser  <= tuser;
      s_axis_tvalid <= 1'b1;

      @(posedge clk);
      while (!(s_axis_tvalid && s_axis_tready)) @(posedge clk);

      s_axis_tvalid <= 1'b0;
      s_axis_tlast  <= 1'b0;
      s_axis_tuser  <= 1'b0;
      s_axis_tdata  <= 64'd0;
      @(posedge clk);
    end
  endtask

  // 從 TXT 讀一幀送出：第一拍 header + SOF，之後每列 TLAST
  task automatic send_frame_from_txt(input string in_path,
                                     output int W, output int H);
    integer fin;
    int rv, r, c, pix;
    reg [63:0] beat;
    begin
      fin = $fopen(in_path, "r");
      if (!fin) begin
        $display("[%0t] ERROR: cannot open %s", $time, in_path);
        $finish;
      end
      rv = $fscanf(fin, "%d %d", W, H);
      if (rv != 2) begin
        $display("[%0t] ERROR: bad header in %s", $time, in_path);
        $finish;
      end
      $display("[%0t] Sending frame %s (W=%0d, H=%0d)", $time, in_path, W, H);
      repeat (8) @(posedge clk);

      for (r = 0; r < H; r++) begin
        for (c = 0; c < W; c++) begin
          rv = $fscanf(fin, "%d", pix);
          if (rv != 1) begin
            $display("[%0t] ERROR: not enough pixels in %s", $time, in_path);
            $finish;
          end
          pix = (pix < 0) ? 0 : (pix > 255) ? 255 : pix;

          if ((r==0) && (c==0)) begin
            beat = 64'd0;
            beat[7:0]   = pix[7:0];    // gray
            beat[23:8]  = H[15:0];     // cfg_h
            beat[39:24] = W[15:0];     // cfg_w
            axis_send_beat(beat, (W==1), 1'b1); // SOF
          end else begin
            beat = 64'd0;
            beat[7:0]   = pix[7:0];
            axis_send_beat(beat, (c==(W-1)), 1'b0); // 每列尾 TLAST
          end
        end
      end
      $fclose(fin);
    end
  endtask

  // 驗收摘要
  task automatic summarize(string tag);
    begin
      $display("== %s: out_total=%0d keep=%0d rows=%0d frames=%0d ==",
               tag, out_total, out_keep, out_rows, out_frames);
    end
  endtask

  // =========================
  // Stimulus / Checks
  // =========================
  int W, H, exp_rows;
  initial begin : STIM
    axis_idle();
    wait(rst_n);

    out_total = 0; out_keep = 0; out_rows = 0; out_frames = 0;

    fout = $fopen(OUT_PATH, "w");
    if (!fout) begin
      $display("[%0t] ERROR: cannot open OUT=%s", $time, OUT_PATH);
      $finish;
    end

    send_frame_from_txt(IN_PATH, W, H);

    // 估算期望的 TLAST 次數
    case (OUT_TLAST_MODE)
      0: exp_rows = 1;               // frame-mode
      1: exp_rows = (H>=3) ? (H-2) : 0; // 每列最後中心
      default: exp_rows = H;         // 每列
    endcase

    // 等一段時間讓尾巴沖掉
    repeat (8000) @(posedge clk);
    $fclose(fout);

    summarize("Summary");

    // 基本檢查
    if (out_frames != 1) begin
      $error("Expected 1 output frame, got %0d", out_frames);
      $stop;
    end
    if (out_rows != exp_rows) begin
      $error("Expected %0d output rows (TLAST), got %0d", exp_rows, out_rows);
      $stop;
    end

    $display("[%0t] All done.", $time);
    #100;
    $finish;
  end

endmodule
