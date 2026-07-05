# 🚀 基于ELF2RK3588的边缘AI多模态设备巡检智能运维系统

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-brightgreen.svg)](https://docs.ros.org/en/humble/)
[![Hardware](https://img.shields.io/badge/Hardware-ELF2_RK3588_%7C_STM32G431-orange.svg)]()

> 本项目为**飞凌嵌入式（ELFBOARD）**相关赛事的开源参赛作品。

本项目以阿克曼转向结构车模为载体，基于“**RK3588（上位机决策） + STM32G431（下位机控制）**”的双脑协同架构，深度集成了 **ROS2 导航栈**与**端侧多模态大模型技术**。系统致力于解决工业复杂环境下的自主巡检、零样本异常研判以及高可靠无人值守运维等核心工程痛点。

---

## 🌟 核心技术亮点

### 🧠 1. 端侧大模型与多模态感知 (Edge AI & VLM)
- **零样本目标检测**：充分调用 RK3588 的端侧 NPU 算力，本地加速部署 **Qwen3-VL** 多模态大模型。系统可作为智能体（Agent）自主调度摄像头进行特征采集，无需重新训练即可精准识别厂区散落水瓶、破损纸箱等异常隐患。
- **本地 RAG 知识库检索**：整合基于 ONNX 部署的轻量化语义向量模型（multilingual-e5-small），将实时视觉画面与内置的《工业 5S 安全标准》深度结合，克服传统视觉算法的特征混淆，生成高准确率的结构化巡检报告。

### ⚡ 2. 物理级电源管控与高可靠生命周期
- **100W PD 诱骗与供电隔离**：底层单片机通过 PMOS 物理控制 SW3518S 快充芯片，向上位机持续提供 100W 的 12V PD 冗余供电，完美保障大模型高负载推理时的系统绝对稳定。
- **RTC 硬件级断电休眠与唤醒**：支持全自主生命周期管理，定时任务结束后自动切断主控电源；到达预设时间后，RTC 中断触发继电器瞬间导通实现设备自动上电，极大延长了设备的整体续航与长期部署时间。

### 🏎️ 3. 阿克曼运动学与多级安全闭环
- **1000Hz 高频运动控制**：自制底板上的 STM32 以 1ms 周期执行 IMU 姿态融合（ICM42688）与轮式里程计解算，内置双轮动力同步、前轮舵机限幅及直线偏航修正等多级 PID 算法。
- **ROS2 导航与避障**：基于 ROS2 Nav2 导航栈，适配阿克曼混合 A*（SmacPlannerHybrid）路径规划。并在底层通讯网关中嵌入 FTG 局部避障与防撞熔断机制。

### 🔄 4. 工业级防砖 OTA 远程升级
- 针对无人值守运维痛点，开发了基于 Y-Modem 协议的定制化 Bootloader，结合外部 SPI 闪存，设计了“连续启动异常自动回滚”机制，大幅降低了偏远无网环境下设备升级失败的维护成本。

---

## 📂 仓库目录结构

```text
ELF2_Inspection_Robot/
├── 1_ROS2_Workspace/         # 上位机工作空间（Python/C++）
│   ├── src/                  # 核心节点：大模型调度、RKNN 图像编码、UI交互
│   ├── patrol_commander.py   # 巡检任务调度与航点控制主程序
│   └── start_system.sh       # 系统一键启动脚本
├── 2_STM32_Code/             # 下位机嵌入式源码（C 语言）
│   ├── STM32_Project/        # 包含高频 PID、IMU 解算及通信协议
│   └── G431bootloader/       # 支持防砖回滚机制的 OTA 引导程序
├── 3_RAG_Knowledge_Base/     # RAG 本地知识检索库
│   └── inspection_rag_docs/  # 工业安全标准 Markdown 格式文档
├── 4_Documentation/          # 项目完整设计文档
│   └── 嵌入式设计说明书.docx   # 详细的软硬件架构图、电路原理图与算法说明
└── README.md
```

---

## 🚀 快速启动指南

### 环境依赖
- **上位机**：Ubuntu 22.04 + ROS2 Humble + RKLLM Runtime
- **下位机**：Keil MDK (ARMCC) + STM32CubeMX

### 运行步骤
1. **编译下位机**：将 `2_STM32_Code` 中的固件烧录至 STM32G431 底板。
2. **构建 ROS2 环境**：在 `1_ROS2_Workspace` 中执行 `colcon build` 编译节点。
3. **启动系统**：
   ```bash
   # 启动雷达、底盘通信网关及 Nav2 导航栈
   ros2 launch my_nav_launch.py
   
   # 启动核心巡检任务与大模型 Agent
   python3 patrol_commander.py
   ```

---

## 🎬 演示视频

项目中实车在模拟厂区内的自主导航、大模型智能交互、RTC 定时唤醒及 OTA 升级的完整演示，请观看以下视频：

👉 **[点击观看 Bilibili 演示视频](https://www.bilibili.com/)** *(记得替换为您自己的链接)*

---

## 📄 开源协议

本项目采用 MIT 开源协议。
