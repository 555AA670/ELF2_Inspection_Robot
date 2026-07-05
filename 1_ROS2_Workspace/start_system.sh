#!/bin/bash
echo "========================================="
echo "    🚀 启动 AI 智能巡检系统 (独立 Nav2)"
echo "========================================="

# ================= 退出清理逻辑 =================
cleanup() {
    echo ""
    echo "========================================="
    echo "    🛑 收到关闭指令，正在安全清理所有后台节点..."
    echo "========================================="
    # 杀掉所有由脚本或 UI 拉起的 Python/ROS 进程
    pkill -f "stm32_sllidar_avoid"
    pkill -f "tts_node.py"
    pkill -f "dashboard.py"
    pkill -f "patrol_commander.py"
    pkill -f "my_nav.launch.py"
    pkill -f "llm_node"
    echo "✅ 所有系统进程已彻底关闭！"
    exit 0
}
# 拦截 Ctrl+C (SIGINT) 信号，触发 cleanup 函数
trap cleanup SIGINT SIGTERM
# ===============================================

# 强制指定工作空间 (容错处理)
WS_DIR="/root/ros2_ws"

# 1. 导入 ROS 2 环境变量
source /opt/ros/humble/setup.bash
source $WS_DIR/install/setup.bash

# 2. 导入大模型动态链接库
export LD_LIBRARY_PATH=/root/rknn-llm-main/rkllm-runtime/Linux/librkllm_api/aarch64:$WS_DIR/src/my_robot_llm/3rdparty/librknnrt/Linux/librknn_api/aarch64:$LD_LIBRARY_PATH

echo "[1/4] 启动底层控制与雷达避障模块 (后台运行)..."
if [ -f "$WS_DIR/stm32_sllidar_avoid_launch.py" ]; then
    cp -f "$WS_DIR/stm32_sllidar_avoid_launch.py" "$WS_DIR/stm32_sllidar_avoid.launch.py"
    ros2 launch "$WS_DIR/stm32_sllidar_avoid.launch.py" > /tmp/avoid_launch.log 2>&1 &
else
    cp -f "/root/stm32_sllidar_avoid_launch.py" "/root/stm32_sllidar_avoid.launch.py" 2>/dev/null
    ros2 launch "/root/stm32_sllidar_avoid.launch.py" > /tmp/avoid_launch.log 2>&1 &
fi
sleep 3

echo "[2/4] 启动语音合成 (TTS) 节点 (后台运行)..."
if [ -f "$WS_DIR/tts_node.py" ]; then
    python3 "$WS_DIR/tts_node.py" > /tmp/tts_node.log 2>&1 &
else
    python3 "/root/tts_node.py" > /tmp/tts_node.log 2>&1 &
fi

echo "[3/4] 启动 UI 交互面板 (后台运行)..."
if [ -f "$WS_DIR/src/ui/dashboard.py" ]; then
    python3 "$WS_DIR/src/ui/dashboard.py" &
else
    echo "❌ 找不到 UI 界面文件: $WS_DIR/src/ui/dashboard.py"
fi
sleep 2

echo "[4/4] 启动大模型 (LLM) 推理节点 (前台显示终端)..."
echo "=========================================================="
echo "当前终端已交由大模型接管，你可以直接在此查看大模型的原生日志！"
echo "按 Ctrl+C 即可关闭大模型与整个系统。"
echo "=========================================================="
ros2 run my_robot_llm llm_node /root/IMG_20260504_194523.jpg /root/qwen3-vl-4b_vision_rk3588.rknn /root/qwen3-vl-4b-instruct_w8a8_rk3588.rkllm 512 4095 3
