#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml
import math
import os
import sys
import time
import threading

# ==========================================
# 调试开关：设为 False 时，只跑航点不等待大模型
# ==========================================
USE_LLM = True

class PatrolCommander(Node):
    def __init__(self):
        super().__init__('patrol_commander')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.prompt_pub = self.create_publisher(String, '/llm/prompt', 10)
        self.response_sub = self.create_subscription(String, '/llm/response', self.llm_response_callback, 10)
        
        # 专门用来发语音播报的 publisher (因为 tts_node 监听的是 /tts/text)
        self.tts_pub = self.create_publisher(String, '/tts/text', 10)
        
        # 专门下发串口指令给 STM32 (设置 RTC、ALARM、FINISH)
        self.action_pub = self.create_publisher(String, '/stm32/action_cmd', 10)
        
        # 强制等待订阅者上线 (解决 ROS 2 节点启动初期的丢包问题)
        self.get_logger().info('等待语音播报节点上线...')
        wait_count = 0
        while self.tts_pub.get_subscription_count() == 0 and wait_count < 20:
            time.sleep(0.1)
            wait_count += 1
        
        self.llm_response_received = False
        self.llm_response_data = ""
        self.waypoints = []
        self.current_wp_index = 0
        self.inspection_records = []

    def euler_to_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return 0.0, 0.0, sy, cy

    def llm_response_callback(self, msg):
        # 只要不是工具调用 <tool_call>，我们就认为它是大模型的最终文字回复
        if "<tool_call>" not in msg.data:
            self.llm_response_data = msg.data
            self.llm_response_received = True

    def start_patrol(self, waypoints_file):
        self.get_logger().info('等待 Nav2 navigate_to_pose 动作服务器上线...')
        self._action_client.wait_for_server()
        self.get_logger().info('服务器已连接！正在解析 yaml 文件...')

        with open(waypoints_file, 'r') as f:
            data = yaml.safe_load(f)

        for pt in data['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = pt['x']
            pose.pose.position.y = pt['y']
            
            _, _, z, w = self.euler_to_quaternion(pt['yaw'])
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            
            self.waypoints.append(pose)

        self.get_logger().info(f'成功解析 {len(self.waypoints)} 个航点！小车即将发车！')
        
        # 播报开始巡检
        msg = String()
        msg.data = "开始巡检"
        self.tts_pub.publish(msg)
        
        threading.Thread(target=self.patrol_loop).start()

    def patrol_loop(self):
        while self.current_wp_index < len(self.waypoints):
            self.get_logger().info(f'>>> 小车当前正在前往第 {self.current_wp_index + 1} 个航点...')
            
            pose = self.waypoints[self.current_wp_index]
            pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            while not send_goal_future.done():
                time.sleep(0.1)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('导航任务被 Nav2 拒绝！')
                return
                
            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                time.sleep(0.1)
            
            # 到达目标点后
            is_target = self.current_wp_index in [2, 4, 6, 8, 11]
            
            if is_target:
                self.get_logger().info('============= [核心巡检点] =============')
                if USE_LLM:
                    self.get_logger().info(f'到达第 {self.current_wp_index + 1} 个核心巡检点！正在呼叫大模型...')
                    
                    self.llm_response_received = False
                    prompt = String()
                    
                    if self.current_wp_index == 11:
                        # 最后一个巡检点（第12个点）：综合判断 + RAG
                        records_str = "\n".join(self.inspection_records) if self.inspection_records else "无历史记录"
                        prompt.data = (
                            f"[System] 机器人已完成全部沿途巡检，现进行最终总结。\n"
                            f"以下是沿途各点位的客观画面记录：\n{records_str}\n"
                            f"【必须执行】请仔细提取上述客观记录中出现的所有物品，自己总结一个涵盖它们的通用搜索词，调用 rag_search 工具去查询它们的安全规范，必须设置 top_k=1。\n"
                            f"【严禁死循环】只能调用一次 rag_search！拿到一次知识库结果后，无论有没有查到对应物品，都不允许再查第二次，严禁调用 get_robot_status 等无关工具！\n"
                            f"【极致速度优先】拿到检索结果后，严禁输出任何分析过程。请直接以极简格式输出最终判定结果。特别注意：整齐堆叠的完好纸箱是正常现象，不属于违规！输出格式示例：'巡检点1：水瓶违规；巡检点2：散乱纸箱违规。' 如果某巡检点正常，则绝对不要列出它！严禁多说任何废话！"
                        )
                    else:
                        # 沿途巡检点：极简客观记录
                        prompt.data = (
                            "[System] 机器人已到达核心巡检点。请立刻且仅调用一次 capture_image 工具拍摄前方画面。"
                            "拍摄完成后，请直接用一句话极其客观地描述画面中最显眼的物体及其状态。"
                            "【关键要求】如果是纸箱，请务必明确描述它是破损/散乱的，还是完好且整齐堆叠的。"
                            "【警告】严禁进行任何“正常”或“异常”的判断，也严禁输出任何多余的分析或寒暄文字，只需描述你看到的事物！严禁调用其他工具。"
                        )
                        
                    self.get_logger().info("正在发送大模型请求，并开始后台推理...")
                    self.prompt_pub.publish(prompt)
                    
                    # =============== 并发语音播报逻辑（完美掩盖大模型延迟） ===============
                    # 定义一个可中断的休眠，如果大模型提前回复，直接跳过废话
                    def sleep_with_check(duration):
                        s_t = time.time()
                        while time.time() - s_t < duration:
                            if self.llm_response_received:
                                break
                            time.sleep(0.1)

                    msg = String()
                    if self.current_wp_index == 11:
                        msg.data = "到达起点，巡检结束"
                        self.tts_pub.publish(msg)
                        sleep_with_check(6.0)
                        
                        if not self.llm_response_received:
                            msg.data = "正在通过 R A G 调取知识库" 
                            self.tts_pub.publish(msg)
                            sleep_with_check(6.0)
                            
                        if not self.llm_response_received:
                            msg.data = "知识库内容已加载，大模型正在生成全局巡检总结报告"
                            self.tts_pub.publish(msg)
                    else:
                        # 沿途点位 (2, 4, 6, 8 对应 1, 2, 3, 4)
                        point_idx = (self.current_wp_index // 2)
                        zh_nums = ["零", "一", "二", "三", "四", "五"]
                        p_str = zh_nums[point_idx] if point_idx < len(zh_nums) else str(point_idx)
                        msg.data = f"到达巡检点{p_str}"
                        self.tts_pub.publish(msg)
                        sleep_with_check(6.0)
                        
                        if not self.llm_response_received:
                            msg.data = "正在调用摄像头"
                            self.tts_pub.publish(msg)
                            sleep_with_check(6.0)
                            
                        if not self.llm_response_received:
                            msg.data = "图像编码器已加载图片，大模型正在推理"
                            self.tts_pub.publish(msg)
                    # ==========================================
                    
                    self.get_logger().info("等待大模型最终响应...")
                    wait_time = 0
                    while not self.llm_response_received and wait_time < 3000:
                        time.sleep(0.1)
                        wait_time += 1
                        
                    if self.llm_response_received:
                        response_text = self.llm_response_data.replace("<final>", "").replace("</final>", "").strip()
                        self.get_logger().info(f"大模型巡检报告：\n{response_text}")
                        
                        # 将大模型的最终报告转发给语音节点进行播报
                        msg_tts = String()
                        msg_tts.data = response_text
                        self.tts_pub.publish(msg_tts)
                        
                        # 如果是沿途点，记录下来
                        if self.current_wp_index != 11:
                            point_num = (self.current_wp_index // 2)
                            self.inspection_records.append(f"巡检点 {point_num}: {response_text}")
                    else:
                        self.get_logger().error("大模型响应超时！(>300s)")
                else:
                    self.get_logger().info(f'到达第 {self.current_wp_index + 1} 个核心点，当前为纯导航调试模式，原地停留 3 秒后继续...')
                    time.sleep(3)
                
                self.get_logger().info('=====================================')
            else:
                self.get_logger().info(f'到达第 {self.current_wp_index + 1} 个航点（非巡检点）。直接通过！')
                time.sleep(0.1)
                
            self.current_wp_index += 1

        self.get_logger().info('🎉 所有航点巡检完毕！任务圆满结束！')
        
        # --- 电源管理逻辑 ---
        if hasattr(self, 'power_mode') and self.power_mode.startswith('sleep-until-'):
            time_str = self.power_mode.split('-')[-1] # e.g. "14:30"
            if ':' in time_str:
                hh, mm = time_str.split(':')
                # 1. 同步当前时间给单片机
                now = time.localtime()
                msg_rtc = String()
                msg_rtc.data = f"RTC,{now.tm_hour:02d},{now.tm_min:02d},{now.tm_sec:02d}"
                self.action_pub.publish(msg_rtc)
                time.sleep(0.5)
                
                # 2. 设置单片机的闹钟唤醒时间
                msg_alarm = String()
                msg_alarm.data = f"ALARM,{hh},{mm},00"
                self.action_pub.publish(msg_alarm)
                time.sleep(0.5)
                
                self.get_logger().info(f"已下发定时唤醒时间: {hh}:{mm}，即将关机进入休眠...")
                msg = String()
                msg.data = f"巡检已结束，系统将在明天的{hh}点{mm}分自动唤醒，现在开始休眠"
                self.tts_pub.publish(msg)
                time.sleep(8.0)
                
                # 3. 发送 FINISH 信号让单片机准备断电
                msg_finish = String()
                msg_finish.data = "FINISH"
                self.action_pub.publish(msg_finish)
                
                # 留出几秒让指令发出去，单片机随后会倒计时 15 秒并直接暴力断电
                time.sleep(2.0)
                self.get_logger().info("FINISH 信号已发送，单片机已启动断电倒计时。正在清理所有后台进程并同步磁盘缓存...")
                
                # 立即播放最后一句专业关机提示，为语音播报争取到足足 12 秒的时间！
                msg_bye = String()
                msg_bye.data = f"即将切断主电源进入低功耗模式，系统将于{hh}点{mm}分准时唤醒。本次巡检任务圆满结束。"
                self.tts_pub.publish(msg_bye)
                
                # 强制同步磁盘缓存到物理硬盘，极大地降低损坏概率
                os.system("sync && sync")
                self.get_logger().info("磁盘同步完成，静待单片机切断电源...")
                
                # 【关键修复】：绝对不能在这里 pkill 杀掉所有 python3 进程！
                # 因为一旦杀掉 stm32_ros2_bridge.py，USB 串口被强制关闭，CH340 芯片会产生 DTR/RTS 脉冲，
                # 这会直接导致单片机发生硬件重启！单片机一重启，刚刚启动的 15 秒断电倒计时就被清零了！
                
                time.sleep(20) # 挂起脚本，静静等待单片机拉低 PC13 断电
        else:
            self.get_logger().info("电源策略为保持开机，系统将继续运行。")
            msg = String()
            msg.data = "巡检已结束，系统保持待命"
            self.tts_pub.publish(msg)

        os._exit(0)

def main(args=sys.argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--power-mode', default='keep-on')
    parsed_args, unknown_args = parser.parse_known_args(args)
    
    rclpy.init(args=unknown_args)
    
    print("=========================================")
    print("      AI 巡检无人车 - 发车控制台 (独立 Python 版)")
    print(f"      当前大模型调用状态: {'[开启]' if USE_LLM else '[关闭 - 仅导航]'}")
    print("=========================================")
    print("警告：请确认小车前方无人员阻挡，并且已加载建图。巡检任务即刻开始！")
    
    commander = PatrolCommander()
    commander.power_mode = parsed_args.power_mode
    file_path = os.path.expanduser('~/ros2_ws/my_waypoints.yaml')
    commander.start_patrol(file_path)
    
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        rclpy.spin(commander, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
