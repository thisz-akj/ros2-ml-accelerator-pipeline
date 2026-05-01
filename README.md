# Real-Time Pose Estimation ML-Accelerated Pipeline

## Overview
This repository implements a high-throughput keypoint detection and pose estimation pipeline optimized for edge deployment on the SiMa.ai MLSoC. The system bridges the gap between research-grade models and production-ready inference by leveraging hardware-specific acceleration and an optimized compilation workflow.

The pipeline supports the complete transition from a PyTorch-trained model to a hardware-executable ELF artifact, enabling real-time inference under resource-constrained edge environments. Performance is achieved through precision-aware quantization, efficient memory management, and tight integration with the SiMa.ai software stack.

---

## Key Technical Features

### Hardware Acceleration
- Utilizes the SiMa.ai Machine Learning Accelerator (MLA) to offload compute-intensive tensor operations
- Reduces CPU dependency and improves overall pipeline throughput

### Precision Optimization
- Implements BF16 quantization for efficient inference
- Achieves ~2× speedup compared to FP32 baseline
- Maintains high keypoint localization accuracy with minimal precision loss

### Edge Compilation Workflow
- Converts PyTorch models to ONNX format
- Uses SiMa.ai Palette SDK (ModelSDK) for profiling, optimization, and compilation
- Generates hardware-specific ELF binaries for deployment

### Memory Optimization
- Reduces host-to-device transfer overhead
- Optimizes tensor layouts for accelerator compatibility
- Improves pipeline efficiency by minimizing memory bottlenecks

---

## System Architecture

### 1. Model Training and Export
- Custom multi-head keypoint detection model developed in PyTorch
- Exported to ONNX format for hardware compatibility

### 2. Model Optimization
- Profiled and optimized using `sima-cli` and ModelSDK
- Applied BF16 quantization for performance gains

### 3. Compilation
- Compiled ONNX model into a hardware-specific ELF binary using the Palette compiler

### 4. Deployment
- Integrated compiled artifact into a C++/Python runtime
- Supports real-time inference with optimized post-processing

---

## Performance Metrics

| Configuration     | Device           | Throughput (FPS) | Latency (ms) |
|------------------|----------------|------------------|--------------|
| FP32 (Baseline)  | CPU / Generic  | 5–7 FPS          | ~150 ms      |
| BF16 (Optimized) | SiMa.ai MLSoC  | 15+ FPS          | < 60 ms      |

- Achieves >95% accuracy retention compared to FP32 baseline  
- Sustains real-time inference under edge deployment constraints  

---

## Skills and Tools

### Languages
- Python
- C++

### Frameworks
- PyTorch
- ONNX
- ROS2

### Deployment Stack
- SiMa.ai Palette SDK (ModelSDK, sima-cli, MPK Tool)

### Optimization Techniques
- Quantization (BF16, INT8)
- Model compression
- Memory and tensor layout optimization

---

## Repository Structure
