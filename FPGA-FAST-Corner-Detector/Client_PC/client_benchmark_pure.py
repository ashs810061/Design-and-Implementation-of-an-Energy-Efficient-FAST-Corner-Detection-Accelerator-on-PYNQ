#!/usr/bin/env python3
import os
import glob
import sys
import socket
import struct
import time
import numpy as np
import cv2

# ================= 設定區 =================
DEFAULT_IMG_DIR = "/home/user/Datasets/EuRoc/MH01/mav0/cam0/data" # you have to motify it to your path
DEFAULT_HOST = "192.168.2.99"
DEFAULT_PORT = 9092
# =========================================

def main():
    img_dir = DEFAULT_IMG_DIR
    if len(sys.argv) > 1: img_dir = sys.argv[1]

    # 1. 預先讀取所有圖片到記憶體 (排除硬碟 I/O 影響)
    print("正在預先載入圖片到記憶體...")
    exts = ['*.png', '*.jpg', '*.jpeg']
    files = sorted([f for e in exts for f in glob.glob(os.path.join(img_dir, e))])
    
    # 只取前 500 張來測就好，不用全部
    files = files[:500] 
    if not files:
        print("找不到圖片"); return

    images_data = []
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            H, W = img.shape
            # 預先打包好 Header 和 Body
            header = struct.pack("<HH", H, W)
            body = img.tobytes()
            images_data.append((header, body, len(body)))
    
    print(f"已載入 {len(images_data)} 張圖片，開始連接 Server...")

    # 2. 連線
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(20.0)
    try:
        sock.connect((DEFAULT_HOST, DEFAULT_PORT))
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception as e:
        print(f"連線失敗: {e}"); return

    print("開始極速傳輸測試 (Pure Network Benchmark)...")
    print("-" * 50)
    
    total_time = 0
    start_global = time.perf_counter()

    # Buffer 預分配
    recv_header = bytearray(4)
    recv_header_mv = memoryview(recv_header)

    for i, (header, body, body_len) in enumerate(images_data):
        t0 = time.perf_counter()

        # (A) 發送
        sock.sendall(header)
        sock.sendall(body)

        # (B) 接收 N
        read_n = 0
        while read_n < 4:
            read_n += sock.recv_into(recv_header_mv[read_n:], 4 - read_n)
        N = struct.unpack("<I", recv_header)[0]

        # (C) 接收 Points
        target_len = N * 8
        read_len = 0
        # 這裡我們只讀掉數據，不花時間轉成 numpy，只測傳輸速度
        while read_len < target_len:
            chunk = sock.recv(target_len - read_len)
            if not chunk: break
            read_len += len(chunk)

        t1 = time.perf_counter()
        
        # 略過第一張 (Numba 編譯)
        if i > 0:
            total_time += (t1 - t0)

        # 每 50 張印一次狀態
        if i % 50 == 0:
            print(f"Frame {i}: {(t1-t0)*1000:.2f} ms")

    total_frames = len(images_data) - 1
    avg_time = total_time / total_frames
    avg_fps = 1.0 / avg_time

    print("-" * 50)
    print(f"測試結束 (排除 GUI、硬碟讀取、OpenCV 繪圖)")
    print(f"平均延遲 (RTT): {avg_time*1000:.2f} ms")
    print(f"系統極限 FPS  : {avg_fps:.2f} FPS")
    print("-" * 50)
    
    sock.close()

if __name__ == "__main__":
    main()