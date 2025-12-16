#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import glob
import sys
import socket
import struct
import threading
import time
import queue
import collections
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import numpy as np

try:
    import cv2
except ImportError:
    print("請先安裝 OpenCV: pip install opencv-python")
    sys.exit(1)

# ================= 設定區 =================
DEFAULT_IMG_DIR = "/home/user/Datasets/EuRoc/MH01/mav0/cam0/data" #you have to motify it to your path
DEFAULT_HOST = "192.168.3.1"
DEFAULT_PORT = 9092
TARGET_FPS = 25.0  # [修正] 鎖定播放速度為 20 FPS (正常速度)
# =========================================

class NetworkClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10.0) 
            self.sock.connect((self.host, self.port))
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return True
        except Exception as e:
            print(f"[Network] Connection failed: {e}")
            return False

    def disconnect(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def send_image(self, img_gray):
        if not self.sock: return False
        try:
            H, W = img_gray.shape
            self.sock.sendall(struct.pack("<HH", H, W))
            self.sock.sendall(img_gray.tobytes())
            return True
        except: return False

    def recv_result(self):
        if not self.sock: return []
        try:
            n_bytes = self.recv_exact(4)
            N = struct.unpack("<I", n_bytes)[0]
            points = []
            if N > 0:
                body = self.recv_exact(N * 8)
                points = np.frombuffer(body, dtype="<u2").reshape(-1, 4)
            return points
        except: return []

    def recv_exact(self, n):
        buf = bytearray(n)
        view = memoryview(buf)
        pos = 0
        while pos < n:
            read = self.sock.recv_into(view[pos:], n - pos)
            if read == 0: raise ConnectionError("EOF")
            pos += read
        return bytes(buf)

class UltimateGUI:
    def __init__(self, root, img_dir):
        self.root = root
        self.root.title("FPGA FAST Detector - Dashboard (Normal Speed)")
        self.root.geometry("1280x800")
        self.root.configure(bg="#2E2E2E")
        
        self.img_dir = img_dir
        self.image_files = []
        self.is_running = False
        
        # 佇列
        self.raw_queue = queue.Queue(maxsize=2)
        self.render_queue = queue.Queue(maxsize=2)
        
        self.fps_history = collections.deque(maxlen=100)
        
        self.setup_ui()
        self.load_images()
        self.update_gui_loop()

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TFrame", background="#2E2E2E")
        style.configure("TLabel", background="#2E2E2E", foreground="#FFFFFF")
        
        top_bar = ttk.Frame(self.root)
        top_bar.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        ttk.Label(top_bar, text="IP:").pack(side=tk.LEFT)
        self.ent_ip = ttk.Entry(top_bar, width=15)
        self.ent_ip.insert(0, DEFAULT_HOST)
        self.ent_ip.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(top_bar, text="Port:").pack(side=tk.LEFT)
        self.ent_port = ttk.Entry(top_bar, width=6)
        self.ent_port.insert(0, str(DEFAULT_PORT))
        self.ent_port.pack(side=tk.LEFT, padx=5)
        
        self.btn_start = tk.Button(top_bar, text="[ 啟動系統 ]", bg="#00AA00", fg="white", 
                                   command=self.toggle_system, font=("Arial", 11, "bold"), relief="flat")
        self.btn_start.pack(side=tk.LEFT, padx=15)

        self.var_strong_only = tk.BooleanVar(value=False)
        self.chk_strong = tk.Checkbutton(top_bar, text="僅顯示強角點", variable=self.var_strong_only,
                                         bg="#2E2E2E", fg="#00FF00", selectcolor="#444", activebackground="#2E2E2E")
        self.chk_strong.pack(side=tk.LEFT, padx=10)

        mid_frame = tk.Frame(self.root, bg="#111")
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=10)
        
        self.canvas = tk.Canvas(mid_frame, bg="#000000", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Double-Button-1>", self.toggle_fullscreen)
        self.img_id = None

        btm_bar = tk.Frame(self.root, bg="#222", height=100)
        btm_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.lbl_fps = tk.Label(btm_bar, text="HW FPS: 00.0", font=("Arial", 24, "bold"), bg="#222", fg="#00FFFF")
        self.lbl_fps.pack(side=tk.LEFT, padx=20, pady=10)
        
        self.lbl_stats = tk.Label(btm_bar, text="Waiting...", justify=tk.LEFT, font=("Arial", 10), bg="#222", fg="#AAA")
        self.lbl_stats.pack(side=tk.LEFT, padx=20)
        
        self.canvas_graph = tk.Canvas(btm_bar, bg="#111", height=60, width=300, highlightthickness=1, highlightbackground="#444")
        self.canvas_graph.pack(side=tk.RIGHT, padx=20, pady=10)

    def load_images(self):
        if not os.path.exists(self.img_dir):
            return
        exts = ['*.png', '*.jpg', '*.jpeg']
        files = sorted([f for e in exts for f in glob.glob(os.path.join(self.img_dir, e))])
        self.image_files = files
        self.lbl_stats.config(text=f"Dataset Loaded: {len(files)} images")

    def toggle_system(self):
        if not self.is_running:
            ip = self.ent_ip.get()
            try: port = int(self.ent_port.get())
            except: return
            
            self.client = NetworkClient(ip, port)
            if self.client.connect():
                self.is_running = True
                self.btn_start.config(text="[ 停止系統 ]", bg="#AA0000")
                threading.Thread(target=self.thread_network_io, daemon=True).start()
                threading.Thread(target=self.thread_processor, daemon=True).start()
            else:
                messagebox.showerror("Error", "無法連線 Server")
        else:
            self.is_running = False
            self.btn_start.config(state=tk.DISABLED)

    # --- 執行緒 1: 網路 I/O (含 20 FPS 速限) ---
    def thread_network_io(self):
        idx = 0
        total = len(self.image_files)
        target_interval = 1.0 / TARGET_FPS  # 0.05秒 (20 FPS)

        while self.is_running and idx < total:
            loop_start = time.perf_counter() # 計時開始
            
            path = self.image_files[idx]
            frame = cv2.imread(path)
            if frame is None: idx+=1; continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # --- 核心傳輸 (全力跑) ---
            t0 = time.perf_counter()
            if not self.client.send_image(gray): break
            points = self.client.recv_result()
            t1 = time.perf_counter()
            
            # 計算硬體純延遲 (Pure Hardware Latency)
            # 這是不含 sleep 的時間，反映硬體真實性能
            net_time = t1 - t0 
            
            try:
                if self.raw_queue.full():
                    try: self.raw_queue.get_nowait()
                    except: pass
                self.raw_queue.put((frame, points, net_time, idx))
            except: pass
            
            idx += 1
            
            # --- [修正] 速度控制 (只影響播放，不影響 FPS 儀表板) ---
            # 如果跑太快 (例如硬體只花了 12ms)，我們就睡 38ms，湊滿 50ms (20 FPS)
            elapsed = time.perf_counter() - loop_start
            wait_time = target_interval - elapsed
            if wait_time > 0:
                time.sleep(wait_time)
            
        self.is_running = False
        self.client.disconnect()

    # --- 執行緒 2: 影像處理 ---
    def thread_processor(self):
        while self.is_running:
            try:
                frame, points, net_time, idx = self.raw_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            only_strong = self.var_strong_only.get()
            n_pts = len(points)
            for i in range(n_pts):
                x, y, stg, score = points[i]
                if only_strong and stg == 0: continue
                cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            
            if self.render_queue.full():
                try: self.render_queue.get_nowait()
                except: pass
            self.render_queue.put((img_tk, net_time, n_pts, idx))

    # --- 主執行緒: GUI 更新 ---
    def update_gui_loop(self):
        try:
            img_tk, net_time, n_pts, idx = self.render_queue.get_nowait()
            
            if self.img_id is None:
                self.img_id = self.canvas.create_image(
                    self.canvas.winfo_width()//2, 
                    self.canvas.winfo_height()//2, 
                    anchor=tk.CENTER, image=img_tk)
            else:
                self.canvas.itemconfig(self.img_id, image=img_tk)
            self.canvas.img_ref = img_tk
            
            # 計算硬體能力 FPS (基於純延遲，不受播放速度影響)
            hw_fps = 1.0 / net_time if net_time > 0 else 0
            bw = (752*480 / 1024 / 1024) * hw_fps
            
            self.fps_history.append(hw_fps)
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            
            # 顯示 HW FPS (硬體能跑多快)，而不是播放 FPS
            self.lbl_fps.config(text=f"HW FPS: {avg_fps:.1f}")
            
            stats_text = (
                f"Frame ID : {idx}\n"
                f"Points   : {n_pts}\n"
                f"Latency  : {net_time*1000:.1f} ms\n"
                f"Bandwidth: {bw:.2f} MB/s\n"
                f"Display  : Locked @ {TARGET_FPS} FPS"
            )
            self.lbl_stats.config(text=stats_text)
            self.draw_graph()
        except queue.Empty: pass
        
        if not self.is_running and "停止" in self.btn_start['text']:
             self.btn_start.config(text="[ 啟動系統 ]", bg="#00AA00", state=tk.NORMAL)
        self.root.after(10, self.update_gui_loop)

    def draw_graph(self):
        self.canvas_graph.delete("all")
        w = 300
        h = 60
        data = list(self.fps_history)
        if len(data) < 2: return
        max_val = 100.0
        step = w / len(data)
        points = []
        for i, val in enumerate(data):
            y = h - (min(val, max_val) / max_val * h)
            points.append(i * step)
            points.append(y)
        self.canvas_graph.create_line(points, fill="#00FF00", width=2)

    def toggle_fullscreen(self, event=None):
        is_fs = self.root.attributes("-fullscreen")
        self.root.attributes("-fullscreen", not is_fs)

if __name__ == "__main__":
    img_path = DEFAULT_IMG_DIR
    if len(sys.argv) > 1: img_path = sys.argv[1]
    root = tk.Tk()
    app = UltimateGUI(root, img_path)
    root.mainloop()