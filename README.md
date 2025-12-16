# FPGA-Accelerated FAST Corner Detector

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-PYNQ-green?logo=xilinx&logoColor=white)](http://www.pynq.io/)
[![Hardware](https://img.shields.io/badge/Hardware-Zynq%20UltraScale%2B-orange)](https://www.xilinx.com/products/silicon-devices/soc/zynq-ultrascale-mpsoc.html)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)]()

## 📖 Introduction
This project implements a high-performance **FAST (Features from Accelerated Segment Test) Corner Detector** using a heterogeneous computing architecture on the Xilinx Zynq UltraScale+ MPSoC (PYNQ platform).

The computationally intensive feature extraction task is offloaded to the **Programmable Logic (PL)**, utilizing **VDMA (Video Direct Memory Access)** for high-speed data transfer. A custom TCP/IP communication protocol enables real-time data streaming to a Host PC, which visualizes the results on a comprehensive dashboard.

This system demonstrates a complete **Hardware-Software Co-design**, achieving significant acceleration and energy efficiency compared to pure software implementations.

## 🚀 Key Features
* **Hardware Acceleration**: Custom IP core for FAST Corner Detection and NMS (Non-Maximum Suppression) implemented on FPGA.
* **High-Throughput Data Path**: Utilizes AXI-Stream and VDMA to maximize memory bandwidth and minimize CPU intervention.
* **Client-Server Architecture**:
    * **Server (PYNQ)**: Handles hardware control, VDMA management, and algorithm execution.
    * **Client (PC)**: Multi-threaded Python GUI for real-time visualization, performance monitoring, and bandwidth analysis.
* **Real-time Dashboard**: Displays live FPS charts, bandwidth usage (MB/s), and processing latency.
* **Adaptive Visualization**: Supports switching between "All Corners" and "Strong Corners" modes instantly.

## 🎥 Demo
![Demo Screenshot](https://github.com/ashs810061/FPGA-FAST-Corner-Detector/blob/main/FPGA-FAST-Corner-Detector/Docs/demo.png?raw=true)
> *A snapshot of the dashboard visualizing real-time detection results. While the display is locked at 20 FPS for human viewing comfort, the backend hardware throughput exceeds 90 FPS.*

## 🛠️ System Architecture
The system utilizes a heterogeneous architecture where the ARM CPU handles network communication and VDMA configuration, while the FPGA PL accelerates the image processing pipeline.

![System Architecture](https://github.com/ashs810061/FPGA-FAST-Corner-Detector/blob/main/FPGA-FAST-Corner-Detector/Docs/architecture.png?raw=true)

## ⚙️ Hardware Implementation Results

The hardware accelerator is implemented on the **Xilinx Zynq UltraScale+** FPGA. The following data is obtained from the Vivado post-implementation reports.

### 1. Resource Utilization
The FAST Corner Detector IP utilizes approximately 36% of the available LUTs, demonstrating a balanced trade-off between hardware complexity and performance.

| Resource | Utilization | Available | Utilization % |
| :--- | :--- | :--- | :--- |
| **LUT** (Look-Up Tables) | 42,498 | 117,120 | **36.29 %** |
| **LUTRAM** | 1,400 | 57,600 | **2.43 %** |
| **FF** (Flip-Flops) | 39,715 | 234,240 | **16.95 %** |
| **BRAM** (Block RAM) | 6.50 | 144 | **4.51 %** |

![Resource Utilization](https://github.com/ashs810061/FPGA-FAST-Corner-Detector/blob/main/FPGA-FAST-Corner-Detector/Docs/utilization.png?raw=true)
> *Vivado post-implementation utilization report.*

### 2. Power Consumption
Total on-chip power consumption is **3.301 W**. Notably, the FPGA Programmable Logic (PL) itself consumes significantly less power compared to the Processing System (PS), proving the extreme energy efficiency of the hardware accelerator.

| Power Component | Consumption (Watts) | Note |
| :--- | :--- | :--- |
| **Dynamic Power** | **2.871 W** | PS: ~2.732W (94%), PL: ~0.139W (6%) |
| **Device Static Power** | **0.430 W** | |
| **Total On-Chip Power** | **3.301 W** | |

![Power Analysis](https://github.com/ashs810061/FPGA-FAST-Corner-Detector/blob/main/FPGA-FAST-Corner-Detector/Docs/power.png?raw=true)
> *Vivado power analysis report. The majority of dynamic power is consumed by the PS (ARM CPU), while the custom hardware logic remains highly efficient.*

## 📊 System Performance Benchmark

The following benchmark compares the end-to-end execution time of the FAST algorithm running on the **ARM Cortex-A53 CPU (OpenCV implementation)** versus the **FPGA Hardware Accelerator**.

| Implementation | Processing Time (ms) | Throughput (FPS) | Speedup |
| :--- | :--- | :--- | :--- |
| **Software (ARM CPU)** | ~30.22 ms | ~33.0 FPS | 1.0x |
| **Hardware (FPGA)** | **~11.05 ms** | **~90.5 FPS** | **2.73x** |

![Performance Evidence](https://github.com/ashs810061/FPGA-FAST-Corner-Detector/blob/main/FPGA-FAST-Corner-Detector/Docs/performance.png?raw=true)
> *Terminal output demonstrating the hardware processing latency (~11ms) and high throughput.*

### ⚠️ Performance Note: Connection Interface
The system supports both Gigabit Ethernet and USB-Ethernet (RNDIS) connections.
* **Gigabit Ethernet (RJ45)**: Recommended for maximum throughput (**90+ FPS**).
* **Micro USB (Ethernet over USB)**: If using the USB interface, the effective frame rate will be limited to **~35-40 FPS** due to the bandwidth limitations and protocol overhead of the USB 2.0 standard. *Note: The internal hardware acceleration speed remains unaffected.*

## 🔧 Prerequisites

### Hardware
* **Development Board**: PYNQ-ZU, Ultra96-V2, or other Zynq UltraScale+ boards.
* **Connection**: Micro USB cable (for RNDIS) or Ethernet cable.

### Software
* **Client Side (PC)**: Python 3.x
    * `numpy`
    * `opencv-python`
    * `Pillow`
* **Server Side (PYNQ)**: PYNQ image v2.5 or later.

## 💻 Installation & Usage

### 1. Server Setup (FPGA)
Upload the `Server_PYNQ` directory to your PYNQ board.
```bash
# Navigate to the server directory
cd Server_PYNQ

# Run the server with the bitstream
sudo python3 server.py --bit design_1.bit
