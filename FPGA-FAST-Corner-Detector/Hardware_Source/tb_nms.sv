// ============================================================================
// tb_nms3x3_event_pix_dense.sv  -- ModelSim 2020 相容版
// - 以「接收順序(raster)」當索引，不用 (m_x,m_y) 當 key ⇒ 不會再誤判 DUP
// - always 區塊內不做「宣告同時初始化」⇒ 避免 vlog-2244
// - 過濾註解改用 sscanf 回傳值判斷，提升相容性
// ============================================================================

module tb_nms3x3_event_pix_dense;

  // ===== Clock / Reset =====
  logic clk=0; always #5 clk=~clk;   // 100 MHz
  logic rst_n=0; initial begin repeat(8) @(posedge clk); rst_n=1; end

  // ===== DUT I/O =====
  logic [15:0] cfg_w, cfg_h;
  logic        s_valid, s_ready;
  logic [15:0] s_x, s_y;
  logic        s_is_strong;
  logic [9:0]  s_score;
  logic        m_valid, m_ready;
  logic [15:0] m_x, m_y;
  logic        m_is_strong;
  logic [9:0]  m_score;

  // ===== Instantiate DUT =====
  nms3x3_event_pix_dense #(
    .MAX_W(2048), .MAX_H(1536),
    .MIN_SCORE(1), .STRICT_GREATER(1)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .cfg_w(cfg_w), .cfg_h(cfg_h),
    .s_valid(s_valid), .s_ready(s_ready),
    .s_x(s_x), .s_y(s_y), .s_is_strong(s_is_strong), .s_score(s_score),
    .m_valid(m_valid), .m_ready(m_ready),
    .m_x(m_x), .m_y(m_y), .m_is_strong(m_is_strong), .m_score(m_score)
  );

  // ===== Ready policy =====
  initial m_ready = 1'b1;
  // // 想測 backpressure 就打開：
  // always @(posedge clk) if (rst_n) m_ready <= $urandom_range(0,3)!=0;

  // ===== Paths =====
  string IN_L0  = "D:/ic_design/vivado_practice/project_new/nms/txt/pyr_l0_kp.txt";
  string IN_L1  = "D:/ic_design/vivado_practice/project_new/nms/txt/pyr_l1_kp.txt";
  string OUT_L0 = "D:/ic_design/vivado_practice/project_new/nms/txt/nms_dense_l0.txt";
  string OUT_L1 = "D:/ic_design/vivado_practice/project_new/nms/txt/nms_dense_l1.txt";

  // ===== Frame buffer (score map) =====
  typedef struct packed { logic st; logic [9:0] sc; } pix_t;
  pix_t frame[]; int F_W, F_H;
  function automatic int idx(int x,int y,int W); return y*W+x; endfunction

  // ===== Safe cast =====
  function automatic int u16(input logic [15:0] v); return $isunknown(v) ? 0 : v; endfunction
  function automatic int u10(input logic [9:0]  v); return $isunknown(v) ? 0 : v; endfunction
  function automatic int u1 (input logic v);       return (v===1'b1) ? 1 : 0;   endfunction

  // ===== Dense sink buffer（以 raster index 寫入，不看 m_x/m_y） =====
  logic  [0:0] out_st   [];   // strong（post-NMS 語意）
  logic  [9:0] out_sc   [];   // score
  int          out_cnt;       // 已收握手數
  int          wr_x, wr_y;    // 當前 raster 座標（0..W-1, 0..H-1）

  // ★★★ 為了避開 ModelSim 2020 限制，把這些暫存變數宣告在 module 作用域
  int k_reg, exp_mx_reg, exp_my_reg;

  task automatic sink_reset();
    out_st  = new[F_W*F_H];
    out_sc  = new[F_W*F_H];
    out_cnt = 0;
    wr_x=0; wr_y=0;
    foreach (out_st[i]) begin out_st[i]=0; out_sc[i]=0; end
  endtask

  // 接收：握手即以 (wr_x,wr_y) 寫入；之後自增 raster 位置
  // 同時做「一致性檢查」（不影響輸出）：m_x/m_y 是否等於 (wr_x-1, wr_y-1)（0 端飽和）
  always @(posedge clk) begin
    if (rst_n && m_valid && m_ready) begin
      k_reg = idx(wr_x, wr_y, F_W);
      out_st[k_reg] = u1 (m_is_strong);
      out_sc[k_reg] = u10(m_score);
      out_cnt++;

      exp_mx_reg = (wr_x==0) ? 0 : (wr_x-1);
      exp_my_reg = (wr_y==0) ? 0 : (wr_y-1);
      if ( (u16(m_x) != exp_mx_reg) || (u16(m_y) != exp_my_reg) ) begin
        $warning("COORDCHK exp=(%0d,%0d) got=(%0d,%0d) @%0t",
                 exp_mx_reg, exp_my_reg, u16(m_x), u16(m_y), $time);
      end

      // 前進 raster
      if (wr_x == F_W-1) begin wr_x = 0; wr_y++; end
      else                wr_x++;
    end
  end

  // ===== 讀檔 -> 填滿 raster =====
  task automatic load_frame_from_file(string path);
    int fd; string line; int x,y,str_i,sc_i,r,maxx=0,maxy=0;

    fd=$fopen(path,"r"); if(fd==0) begin $error("open %s fail", path); $finish; end
    // 掃描一次取 W/H（只吃能被 "%d %d %d %d" 解析的行；註解/空行略過）
    while(!$feof(fd)) begin
      void'($fgets(line,fd));
      r=$sscanf(line, "%d %d %d %d", x,y,str_i,sc_i);
      if(r==4) begin if(x>maxx)maxx=x; if(y>maxy)maxy=y; end
    end
    $fclose(fd);

    F_W=maxx+1; F_H=maxy+1;
    frame=new[F_W*F_H];
    foreach(frame[i]) begin frame[i].st=1'b0; frame[i].sc=10'd0; end

    // 第二次實際填資料
    fd=$fopen(path,"r");
    while(!$feof(fd)) begin
      void'($fgets(line,fd));
      r=$sscanf(line,"%d %d %d %d",x,y,str_i,sc_i);
      if(r==4 && x>=0 && x<F_W && y>=0 && y<F_H) begin
        frame[idx(x,y,F_W)].st = (str_i!=0);
        frame[idx(x,y,F_W)].sc = sc_i[9:0];
      end
    end
    $fclose(fd);
    $display("[LOAD] %s -> W=%0d H=%0d", path, F_W, F_H);
  endtask

  // ===== 串流 raster（每像素送一拍） =====
  task automatic stream_frame(string in_path, string out_path);
    int x,y, ofd;
    load_frame_from_file(in_path);

    // 設定尺寸並穩定一拍
    cfg_w=F_W; cfg_h=F_H; @(posedge clk);

    // 初始化 source 與 sink
    sink_reset();
    s_valid=1'b0; s_x='0; s_y='0; s_is_strong='0; s_score='0;

    // 逐像素送，確保 ready
    for (y=0; y<F_H; y++) begin
      for (x=0; x<F_W; x++) begin
        do @(posedge clk); while(!rst_n || !s_ready);
        s_x = x; s_y = y;
        s_is_strong = frame[idx(x,y,F_W)].st;
        s_score     = frame[idx(x,y,F_W)].sc;
        s_valid = 1'b1;
        @(posedge clk);
        s_valid = 1'b0;
      end
    end

    // 等到「收滿 W*H 筆輸出」
    wait (out_cnt == F_W*F_H);
    repeat (8) @(posedge clk); // 收尾幾拍

    // 一次性輸出（raster 順序；每像素恰好一行）
    ofd=$fopen(out_path,"w"); if(ofd==0) begin $error("open %s fail",out_path); $finish; end
    $fwrite(ofd,"# x y strong score  // dense; post-NMS strong; raster order\n");
    for (int yy=0; yy<F_H; yy++) begin
      for (int xx=0; xx<F_W; xx++) begin
        int k = idx(xx,yy,F_W);
        $fwrite(ofd, "%0d %0d %0d %0d\n", xx, yy, out_st[k], out_sc[k]);
      end
    end
    $fclose(ofd);

    $display("[DONE] %s -> %s  out_cnt=%0d expect=%0d",
             in_path, out_path, out_cnt, F_W*F_H);
  endtask

  // ===== Test flow =====
  initial begin
    cfg_w=0; cfg_h=0;
    s_valid=0; s_x=0; s_y=0; s_is_strong=0; s_score=0;

    @(posedge rst_n);

    stream_frame(IN_L0, OUT_L0);
    stream_frame(IN_L1, OUT_L1);

    repeat(20) @(posedge clk);
    $finish;
  end

  // ===== 注意 =====
  // 你的 RTL 目前將 m_is_strong 直接傳遞 center 的 strong 標記，
  // 即使被 NMS 壓掉 (m_score==0) 也可能保持為 1。
  // 這是「語意設計」而非錯誤；若你希望 strong 也一起被抑制，
  // 請在 RTL 的 Stage-2 把 st2_c_str <= pass_keep ? st1_c_str : 1'b0。

endmodule
