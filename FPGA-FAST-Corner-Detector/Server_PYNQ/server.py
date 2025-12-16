#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, socket, struct, time, threading
import numpy as np
import cv2
from pynq import Overlay, MMIO, allocate
from numba import njit

# ===== VDMA register offsets =====
MM2S_DMACR, MM2S_DMASR   = 0x00, 0x04
MM2S_VSIZE, MM2S_HSIZE   = 0x50, 0x54
MM2S_STRIDE              = 0x58
MM2S_START_ADDR          = 0x5C

S2MM_DMACR, S2MM_DMASR   = 0x30, 0x34
S2MM_VSIZE, S2MM_HSIZE   = 0xA0, 0xA4
S2MM_STRIDE              = 0xA8
S2MM_START_ADDR          = 0xAC

# ================= VDMA & Reset Functions =================
def vdma_init(mmio, W):
    bpl = 8*W
    mmio.write(MM2S_DMASR, 0xFFFFFFFF)
    mmio.write(S2MM_DMASR, 0xFFFFFFFF)
    mmio.write(MM2S_DMACR, 0x00000001)  # RS=1
    mmio.write(S2MM_DMACR, 0x00000001)  # RS=1
    mmio.write(MM2S_STRIDE, bpl); mmio.write(MM2S_HSIZE, bpl)
    mmio.write(S2MM_STRIDE, bpl); mmio.write(S2MM_HSIZE, bpl)

def vdma_soft_reset(mmio, timeout_s=0.01):
    # 簡單暴力重置：直接寫入 Reset bit
    for off in (MM2S_DMACR, S2MM_DMACR):
        mmio.write(off, mmio.read(off) | (1<<2))
    
    # 等待重置完成
    t0 = time.perf_counter()
    while True:
        r1 = mmio.read(MM2S_DMACR) & (1<<2)
        r2 = mmio.read(S2MM_DMACR) & (1<<2)
        if (r1|r2)==0: break
        if (time.perf_counter()-t0) > timeout_s: break
    
    # 清除狀態
    mmio.write(MM2S_DMASR, 0xFFFFFFFF)
    mmio.write(S2MM_DMASR, 0xFFFFFFFF)

def vdma_start(mmio, in_phys, out_phys, H):
    mmio.write(MM2S_DMASR, 0xFFFFFFFF)
    mmio.write(S2MM_DMASR, 0xFFFFFFFF)
    mmio.write(MM2S_START_ADDR + 0*4, in_phys  & 0xFFFFFFFF)
    mmio.write(S2MM_START_ADDR + 0*4, out_phys & 0xFFFFFFFF)
    mmio.write(S2MM_VSIZE, H)
    mmio.write(MM2S_VSIZE, H)

def wait_ioc(mmio, timeout_s=2.0):
    deadline = time.perf_counter_ns() + int(timeout_s*1e9)
    while True:
        s1 = mmio.read(MM2S_DMASR)
        s2 = mmio.read(S2MM_DMASR)
        if (s1 & (1<<12)) and (s2 & (1<<12)):
            # 清除 IOC
            mmio.write(MM2S_DMASR, 1<<12)
            mmio.write(S2MM_DMASR, 1<<12)
            return s1, s2
        if time.perf_counter_ns() > deadline:
            raise TimeoutError("VDMA timeout")

def recv_exact_into(conn, mv):
    got = 0; n = len(mv)
    while got < n:
        k = conn.recv_into(mv[got:], n-got)
        if k == 0: raise ConnectionError("EOF")
        got += k
    return got

# ================= 演算法核心 =================
@njit(fastmath=True)
def parse_corners_numba(words_u64):
    MAX_CORNERS = 20000 
    xs = np.empty(MAX_CORNERS, dtype=np.uint16)
    ys = np.empty(MAX_CORNERS, dtype=np.uint16)
    stg = np.empty(MAX_CORNERS, dtype=np.uint16)
    scr = np.empty(MAX_CORNERS, dtype=np.uint16)
    count = 0
    n_pixels = len(words_u64)
    for i in range(n_pixels):
        w = words_u64[i]
        if (w & 0x3FF) != 0:
            if count >= MAX_CORNERS: break
            ys[count]  = (w >> 48) & 0xFFFF
            xs[count]  = (w >> 32) & 0xFFFF
            stg[count] = (w >> 10) & 0x1
            scr[count] = w & 0x3FF
            count += 1
    return xs[:count], ys[:count], stg[:count], scr[:count]

def run_cpu_fast(img_gray, threshold=20):
    fast = cv2.FastFeatureDetector_create(threshold=threshold, nonmaxSuppression=True)
    keypoints = fast.detect(img_gray, None)
    N = len(keypoints)
    xs = np.empty(N, dtype=np.uint16)
    ys = np.empty(N, dtype=np.uint16)
    stg = np.full(N, 1, dtype=np.uint16)
    scr = np.empty(N, dtype=np.uint16)
    for i, kp in enumerate(keypoints):
        xs[i] = int(kp.pt[0]); ys[i] = int(kp.pt[1])
        scr[i] = int(kp.response) if kp.response else threshold 
    return xs, ys, stg, scr

# ================= 主處理邏輯 =================
def handle_client(conn, addr, ctx):
    stats_total_ms = 0.0
    stats_count = 0
    frame_id = 0
    
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[TCP] client {addr} connected")
        print(f"\n{'Frame':<6} | {'Time(ms)':<10} | {'FPS':<8} | {'N-Points':<8} | {'Mode':<6}")
        print("-" * 55)

        header = bytearray(4)
        hdr_mv = memoryview(header)

        while True:
            # 1. 接收 Header
            recv_exact_into(conn, hdr_mv)
            H, W = struct.unpack("<HH", header)

            # 2. Buffer Init (只在尺寸變更時做)
            if not ctx['use_cpu']:
                if ctx['cur_shape'] != (H, W):
                    vdma_init(ctx['mmio'], W)
                    if ctx['in_buf']:
                        try: ctx['in_buf'].freebuffer(); ctx['out_buf'].freebuffer()
                        except: pass
                    ctx['in_buf']  = allocate((H, W), dtype=np.uint64, cacheable=1)
                    ctx['out_buf'] = allocate((H, W), dtype=np.uint64, cacheable=1)
                    ctx['in_bytes']  = ctx['in_buf'].view(np.uint8).reshape(H, W, 8)
                    ctx['in_plane0'] = ctx['in_bytes'][:, :, 0]
                    ctx['rx_buf']    = bytearray(H*W)
                    ctx['rx_mv']     = memoryview(ctx['rx_buf'])
                    ctx['cur_shape'] = (H, W)
                    print(f"[FPGA] re-init buffers {W}x{H} (CACHEABLE=1)")
            else:
                if ctx['cur_shape'] != (H, W):
                    ctx['rx_buf'] = bytearray(H*W)
                    ctx['rx_mv']  = memoryview(ctx['rx_buf'])
                    ctx['cur_shape'] = (H, W)
                    print(f"[CPU] re-init buffers {W}x{H}")

            # 3. 接收影像
            recv_exact_into(conn, ctx['rx_mv'][:H*W])
            rx_plane = np.frombuffer(ctx['rx_buf'], dtype=np.uint8, count=H*W).reshape(H, W)

            # ===== Benchmark Start =====
            t_start = time.perf_counter()
            
            if ctx['use_cpu']:
                xs, ys, stg, sc = run_cpu_fast(rx_plane, threshold=ctx['cpu_th'])
            else:
                np.copyto(ctx['in_plane0'], rx_plane, casting='no')
                ib = ctx['in_bytes']
                ib[0,0,1] = (H & 0xFF); ib[0,0,2] = ((H >> 8) & 0xFF)
                ib[0,0,3] = (W & 0xFF); ib[0,0,4] = ((W >> 8) & 0xFF)
                ctx['in_buf'].flush()

                # [這裡就是你要的 Reset Per Frame]
                # 使用最簡單的 Soft Reset，不加任何複雜檢查，保證不會卡住
                if ctx['reset_per_frame']:
                    vdma_soft_reset(ctx['mmio'])
                    vdma_init(ctx['mmio'], W)

                vdma_start(ctx['mmio'], ctx['in_buf'].physical_address, ctx['out_buf'].physical_address, H)
                wait_ioc(ctx['mmio'], ctx['timeout'])

                ctx['out_buf'].invalidate()
                xs, ys, stg, sc = parse_corners_numba(ctx['out_buf'].reshape(-1))

            t_end = time.perf_counter()
            proc_ms = (t_end - t_start) * 1000.0
            # ===== Benchmark End =====

            if stats_count > 0: stats_total_ms += proc_ms
            stats_count += 1
            
            N = int(xs.size)
            curr_fps = 1000.0 / proc_ms if proc_ms > 0 else 0
            status_tag = "CPU" if ctx['use_cpu'] else ("RST" if ctx['reset_per_frame'] else "FPGA")

            print(f"{frame_id:<6} | {proc_ms:<10.2f} | {curr_fps:<8.1f} | {N:<8} | {status_tag:<6}")
            frame_id += 1

            conn.sendall(struct.pack("<I", N))
            if N > 0:
                pkt = np.empty((N, 4), dtype="<u2")
                pkt[:,0]=xs; pkt[:,1]=ys; pkt[:,2]=stg; pkt[:,3]=sc
                conn.sendall(pkt.tobytes(order="C"))

    except Exception as e:
        if "EOF" not in str(e): print(f"[ERR] {e}")
    finally:
        try: conn.close()
        except: pass
        valid_frames = stats_count - 1
        print("\n" + "="*40)
        print(f"  Session Summary ({'CPU' if ctx['use_cpu'] else 'FPGA'})")
        print("="*40)
        if valid_frames > 0:
            avg_ms = stats_total_ms / valid_frames
            print(f"  Valid Frames (no 1st)  : {valid_frames}")
            print(f"  Total Time (Valid)     : {stats_total_ms:.2f} ms")
            print(f"  >> Average Time        : {avg_ms:.4f} ms")
            print(f"  >> Hardware FPS        : {(1000.0/avg_ms):.2f} FPS")
        print("="*40 + "\n")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--bit", required=True, help="Path to bitstream")
    ap.add_argument("--reset-per-frame", action="store_true", help="Reset VDMA per frame")
    ap.add_argument("--port", type=int, default=9092)
    ap.add_argument("--rst-base", type=lambda x:int(x,0), default=0)
    ap.add_argument("--cpu", action="store_true", help="Run on ARM CPU (OpenCV)")
    ap.add_argument("--threshold", type=int, default=20, help="FAST threshold for CPU")

    args=ap.parse_args()

    print(f"[Init] Loading bitstream: {args.bit} ...")
    ol=Overlay(args.bit); ol.download()
    
    mmio = None
    rst_mmio = None

    if not args.cpu:
        print("[Init] Mode: FPGA Hardware Acceleration")
        vdma_name = [k for k in ol.ip_dict.keys() if "vdma" in k.lower()][0]
        mmio = MMIO(ol.ip_dict[vdma_name]["phys_addr"], ol.ip_dict[vdma_name]["addr_range"])
    else:
        print(f"[Init] Mode: ARM CPU (OpenCV Software)")

    ctx=dict(
        mmio=mmio, cur_shape=None, in_buf=None, out_buf=None,
        timeout=2.0, 
        reset_per_frame=args.reset_per_frame,
        rst_mmio=rst_mmio, rst_mask=1, rst_on=1, rst_off=0, rst_hold=0.00002,
        rx_buf=None, rx_mv=None,
        use_cpu=args.cpu,
        cpu_th=args.threshold
    )

    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        s.bind(("0.0.0.0", args.port)); s.listen(1)
        print(f"[TCP] Listening on {args.port} ...")
        while True:
            conn,addr=s.accept()
            th=threading.Thread(target=handle_client,args=(conn,addr,ctx),daemon=True)
            th.start()

if __name__=="__main__":
    main()