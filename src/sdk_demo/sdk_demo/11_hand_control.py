#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
灵巧手多模式控制节点

功能说明：
1. 通过ROS2话题和服务控制左右手灵巧手。
2. 支持同时进行位置、力矩、速度控制。
3. 依次控制每个手指（关节ID 1~6）运动到目标状态。

控制话题和服务：
  - /inspire_hand/ctrl/left_hand  - 左手位置控制话题
  - /inspire_hand/ctrl/right_hand - 右手位置控制话题
  - /inspire_hand/set_force/left_hand  - 左手力矩控制服务
  - /inspire_hand/set_force/right_hand - 右手力矩控制服务
  - /inspire_hand/set_speed/left_hand  - 左手速度控制服务
  - /inspire_hand/set_speed/right_hand - 右手速度控制服务
  - /inspire_hand/set_clear_error/left_hand  - 左手清除错误服务
  - /inspire_hand/set_clear_error/right_hand - 右手清除错误服务

============================================================================
使用说明
============================================================================

【位置、力矩、速度控制(默认控制1号关节)】
    ros2 run sdk_demo hand_control --pos 0.5 --tor 0.1 --spd 0.1

【位置和力矩控制(默认控制1号关节)】
    ros2 run sdk_demo hand_control --pos 0.6 --tor 0.2

【仅位置控制(默认控制1号关节)】
    ros2 run sdk_demo hand_control --pos 0.8
    
【指定关节控制】
    # 控制大拇指外的所有关节
    ros2 run sdk_demo hand_control --ids 1,2,3,4 --pos 0.5 --tor 0.1 --spd 0.1
    ros2 run sdk_demo hand_control --ids 1,2,3,4 --pos 0.8 --tor 0.1 --spd 0.1

    # 控制大拇指弯曲关节
    ros2 run sdk_demo hand_control --ids 5 --pos 0.6 --tor 0.1 --spd 0.1
    ros2 run sdk_demo hand_control --ids 5 --pos 1.0 --tor 0.1 --spd 0.1

    # 控制大拇指旋转关节
    ros2 run sdk_demo hand_control --ids 6 --pos 0.6 --tor 0.1 --spd 0.1
    ros2 run sdk_demo hand_control --ids 6 --pos 1.0 --tor 0.1 --spd 0.1

【清除错误】
    ros2 run sdk_demo hand_control --clear_error

【参数说明】
  --ids <值>：要控制的关节ID列表，用逗号分隔（如 1,2,3），可选值1-6，默认仅控制1号关节(小拇指)，如果要同时控制大拇指旋转关节(6号关节)和其他关节，请注意其与其他关节的相对位置关系，避免发生碰撞损坏灵巧手
  --pos <值>：手指关节位置，取值范围 0.0~1.0，1表示完全张开，0表示完全握紧
  --tor <值>：手指关节转动过程中施加的力矩，取值范围 0.0~1.0，手指关节转动过程中会施加的最大力矩，0代表0g，1代表1000g
  --spd <值>：手指关节转动速度，取值范围 0.0~1.0，手指关节转动速度，1代表800ms从最大角度到最小角度，0.5代表1600ms，0.25代表3200ms
  --clear_error：清除错误
============================================================================

"""

from typing import Optional
import argparse
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bodyctrl_msgs.srv import SetForce, SetSpeed, SetClearError


class InspireHandControllerDemo(Node):
    """灵巧手多控制节点
    
    按顺序执行一次动作，完成后自动退出。
    
    属性：
        left_hand_publisher: 左手控制话题发布器
        right_hand_publisher: 右手控制话题发布器
        config: 控制配置
    """

    # ========== 常量定义 ==========
    
    # 关节ID顺序
    JOINT_ID_SEQUENCE = ["1"]                          # 单独控制小拇指关节(1号关节)
    # 如果要同时控制拇指旋转关节(6号关节)，请注意其与其他关节的相对位置关系，避免发生碰撞损坏灵巧手

    # 关节ID对应中文名称（用于日志）
    JOINT_NAME_MAP = {
        "1": "小指",
        "2": "无名指",
        "3": "中指",
        "4": "食指",
        "5": "拇指弯曲",
        "6": "拇指旋转",
    }

    # 控制模式对应的描述文本
    CONTROL_MODE_DESCRIPTIONS = {
        "pos": "位置控制",
        "torque": "力矩控制",
        "vel": "速度控制",
    }

    def __init__(self, pos: Optional[float] = None, tor: Optional[float] = None, spd: Optional[float] = None, clear_error: bool = False, joint_ids: Optional[list] = None):
        """
        初始化节点
        
        参数：
            pos (float, optional): 位置控制值，范围0.0~1.0
            tor (float, optional): 力矩控制值，范围0.0~1.0
            spd (float, optional): 速度控制值，范围0.0~1.0
            clear_error (bool): 是否清除错误
            joint_ids (list, optional): 要控制的关节ID列表，如 ["1", "2", "3"]，默认为 JOINT_ID_SEQUENCE
        """
        super().__init__("inspire_hand_controller_demo")

        # 保存控制参数
        self.pos_value = pos
        self.tor_value = tor
        self.spd_value = spd
        self.clear_error_flag = clear_error
        self.joint_ids = joint_ids if joint_ids else self.JOINT_ID_SEQUENCE

        # 位置话题发布器初始化
        self.left_hand_publisher = self.create_publisher(JointState, "/inspire_hand/ctrl/left_hand", 10)
        self.right_hand_publisher = self.create_publisher(JointState, "/inspire_hand/ctrl/right_hand", 10)

        # 手指关节力矩服务和手指关节速度服务客户端初始化
        self.left_hand_force_client = self.create_client(SetForce, "/inspire_hand/set_force/left_hand")
        self.right_hand_force_client = self.create_client(SetForce, "/inspire_hand/set_force/right_hand")
        self.left_hand_speed_client = self.create_client(SetSpeed, "/inspire_hand/set_speed/left_hand")
        self.right_hand_speed_client = self.create_client(SetSpeed, "/inspire_hand/set_speed/right_hand")
        
        self.left_hand_clear_error_client = self.create_client(SetClearError, "/inspire_hand/set_clear_error/left_hand")
        self.right_hand_clear_error_client = self.create_client(SetClearError, "/inspire_hand/set_clear_error/right_hand")

        # 等待服务可用
        if self.clear_error_flag:
            if not self.left_hand_clear_error_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning("左手清除错误服务不可用")
            if not self.right_hand_clear_error_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning("右手清除错误服务不可用")
        else:
            if self.tor_value is not None:
                if not self.left_hand_force_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warning("左手力矩控制服务不可用")
                if not self.right_hand_force_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warning("右手力矩控制服务不可用")

            if self.spd_value is not None:
                if not self.left_hand_speed_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warning("左手速度控制服务不可用")
                if not self.right_hand_speed_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warning("右手速度控制服务不可用")

        # ========== 参数初始化 ==========
        
        if self.clear_error_flag:
            self.get_logger().info("灵巧手控制节点已初始化\n  操作：清除错误")
        else:
            mode_info = []
            if self.pos_value is not None:
                mode_info.append(f"位置: {self.pos_value:.2f}")
            if self.tor_value is not None:
                mode_info.append(f"力矩: {self.tor_value:.2f}")
            if self.spd_value is not None:
                mode_info.append(f"速度: {self.spd_value:.2f}")
            
            joint_names = [self.JOINT_NAME_MAP.get(jid, f"未知关节{jid}") for jid in self.joint_ids]
            
            self.get_logger().info(
                f"灵巧手控制节点已初始化\n"
                f"  控制参数：{', '.join(mode_info) if mode_info else '无'}\n"
                f"  控制关节：{', '.join(joint_names)} (IDs: {', '.join(self.joint_ids)})"
            )

    def clear_error(self):
        """
        清除手指错误状态
        """
        self.get_logger().info("开始清除错误状态...")
        # 创建 ClearError 服务请求
        request = SetClearError.Request()

        # 发送请求到左右手清除错误服务
        try:
            # 左手服务调用 和 右手服务调用
            future_left = self.left_hand_clear_error_client.call_async(request)
            future_right = self.right_hand_clear_error_client.call_async(request)

            # 等待服务响应
            rclpy.spin_until_future_complete(self, future_left, timeout_sec=2.0)
            rclpy.spin_until_future_complete(self, future_right, timeout_sec=2.0)

            # 获取并打印服务响应
            if future_left.done():
                response_left = future_left.result()
                self.get_logger().info(f"  左手响应 - setclear_error_accepted: {response_left.setclear_error_accepted}")

            if future_right.done():
                response_right = future_right.result()
                self.get_logger().info(f"  右手响应 - setclear_error_accepted: {response_right.setclear_error_accepted}")

            # 输出控制日志
            self.get_logger().info(f"[清除错误] 服务调用参数：{request}")
        except Exception as e:
            self.get_logger().error(f"清除错误服务调用失败: {e}")

    def execute_hand_grasp(self):
        """
        执行动作 - 同时执行指定的控制模式或清除错误
        """    
        self.get_logger().info("开始执行控制命令...")
        
        # 执行力矩控制
        if self.tor_value is not None:
            self.set_force(self.tor_value)
        
        # 设置手指关节转动速度
        if self.spd_value is not None:
            self.set_speed(self.spd_value)
        
        # 执行位置控制
        if self.pos_value is not None:
            self._execute_position_control(self.joint_ids, self.pos_value)
        
        # 动作完成
        self.get_logger().info("✓ 动作已完成")

    def _execute_position_control(self, joint_ids: list, target_value):
        """
        位置控制模式执行函数
        
        发布单个关节的位置控制指令，使关节运动到目标位置。
        
        参数：
            joint_ids (list): 关节ID列表（字符串形式）
        """        
        # 构建位置控制消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
    
        # 遍历所有关节，依次执行控制
        for joint_id in joint_ids:
            joint_name = self.JOINT_NAME_MAP.get(joint_id, f"未知关节{joint_id}")
            msg.name.append(joint_id)
            msg.position.append(target_value)

        # 发布到左右手控制话题
        self.left_hand_publisher.publish(msg)
        self.right_hand_publisher.publish(msg)

        # 输出控制日志
        joint_names = [self.JOINT_NAME_MAP.get(jid, f"未知关节{jid}") for jid in joint_ids]
        self.get_logger().info(
            f"[位置控制] 关节：{', '.join(joint_names)} (IDs: {', '.join(joint_ids)})，"
            f"目标位置：{self.pos_value}，"
            f"发布消息：{msg}"
        )
            

    def set_force(self, force_value : float):
        """
        设置手指各个关节的旋转过程中会施加的最大力矩
        
        通过 SetForce 服务发送力矩控制命令，该服务可同时控制所有6个手指关节的力矩。
                
        参数：
            force_value: 力矩比例值列表（浮点数，范围0.0~1.0，0代表0g，1代表1000g）
        """
        # 创建 SetForce 服务请求
        request = SetForce.Request()
        
        # 填充所有力反馈比例参数（这里简单将所有关节的力矩比例设置为相同值，实际应用中可根据具体场景调整）
        request.force0_ratio = force_value
        request.force1_ratio = force_value
        request.force2_ratio = force_value
        request.force3_ratio = force_value
        request.force4_ratio = force_value
        request.force5_ratio = force_value

        # 发送请求到左右手力矩控制服务
        try:
            # 左手服务调用
            future_left = self.left_hand_force_client.call_async(request)
            
            # 右手服务调用
            future_right = self.right_hand_force_client.call_async(request)
            
            # 等待服务响应
            rclpy.spin_until_future_complete(self, future_left, timeout_sec=2.0)
            rclpy.spin_until_future_complete(self, future_right, timeout_sec=2.0)
            
            # 获取并打印服务响应
            if future_left.done():
                response_left = future_left.result()
                self.get_logger().info(f"  左手响应 - force_accepted: {response_left.force_accepted}")
            
            if future_right.done():
                response_right = future_right.result()
                self.get_logger().info(f"  右手响应 - force_accepted: {response_right.force_accepted}")
            
            # 输出控制日志
            self.get_logger().info(
                f"[力矩控制] 目标力矩比例：{force_value}，"
                f"服务调用参数：{request}，"
            )
        except Exception as e:
            self.get_logger().error(f"力矩控制服务调用失败: {e}")

    def set_speed(self, speed_value : float):
        """
        设置手指各个关节的转动速度
        
        通过 SetSpeed 服务发送速度控制命令，该服务可同时控制所有6个手指关节的速度。
                
        参数：
            speed_value: 速度比例值列表（浮点数，范围0.0~1.0，1代表800ms从最大角度到最小角度，0.5代表1600ms，0.25代表3200ms）
        """
        # 创建 SetSpeed 服务请求
        request = SetSpeed.Request()
        
        # 填充所有速度比例参数（这里简单将所有关节的速度比例设置为相同值，实际应用中可根据具体场景调整）
        request.speed0_ratio = speed_value
        request.speed1_ratio = speed_value
        request.speed2_ratio = speed_value
        request.speed3_ratio = speed_value
        request.speed4_ratio = speed_value
        request.speed5_ratio = speed_value

        # 发送请求到左右手速度控制服务
        try:
            # 左手服务调用 和 右手服务调用
            future_left = self.left_hand_speed_client.call_async(request)
            future_right = self.right_hand_speed_client.call_async(request)
            
            # 等待服务响应
            rclpy.spin_until_future_complete(self, future_left, timeout_sec=2.0)
            rclpy.spin_until_future_complete(self, future_right, timeout_sec=2.0)
            
            # 获取并打印服务响应
            if future_left.done():
                response_left = future_left.result()
                self.get_logger().info(f"  左手响应 - speed_accepted: {response_left.speed_accepted}")
            
            if future_right.done():
                response_right = future_right.result()
                self.get_logger().info(f"  右手响应 - speed_accepted: {response_right.speed_accepted}")
            
            # 输出控制日志
            self.get_logger().info(
                f"[速度控制] 目标速度比例：{speed_value}，"
                f"服务调用参数：{request}，"
            )
        except Exception as e:
            self.get_logger().error(f"速度控制服务调用失败: {e}")

def main(args=None):
    
    # 定义参数解析器
    parser = argparse.ArgumentParser(
        description="灵巧手多模式控制节点",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例用法：\n"
               "  位置控制（默认控制1号关节）：ros2 run sdk_demo hand_control --pos 0.5\n"
               "  指定关节：ros2 run sdk_demo hand_control --ids 1,2,3 --pos 0.5\n"
               "  力矩设置：ros2 run sdk_demo hand_control --tor 0.3\n"
               "  速度设置：ros2 run sdk_demo hand_control --spd 0.2\n"
               "  组合控制：ros2 run sdk_demo hand_control --ids 1,2,3 --pos 0.5 --tor 0.3 --spd 0.2\n"
               "  清除错误：ros2 run sdk_demo hand_control --clear_error"
    )
    
    # 可选参数：关节ID列表
    parser.add_argument(
        '--ids',
        type=str,
        default=None,
        help="要控制的关节ID列表，用逗号分隔（如 1,2,3），可选值1-6，默认仅控制1号关节(小拇指)，如果要同时控制大拇指旋转关节(6号关节)和其他关节，请注意其与其他关节的相对位置关系，避免发生碰撞损坏灵巧手"
    )
    
    # 可选参数：位置值
    parser.add_argument(
        '--pos',
        type=float,
        default=None,
        help="位置控制值（范围0.0~1.0，0表示完全握紧，1表示完全张开）"
    )
    
    # 可选参数：力矩值
    parser.add_argument(
        '--tor',
        type=float,
        default=None,
        help="力矩设置值（范围0.0~1.0，0代表0g，1代表1000g）"
    )
    
    # 可选参数：速度值
    parser.add_argument(
        '--spd',
        type=float,
        default=None,
        help="速度设置值（范围0.0~1.0，1代表800ms从最大角度到最小角度，0.5代表1600ms，0.25代表3200ms）"
    )
    
    # 可选参数：清除错误
    parser.add_argument(
        '--clear_error',
        action='store_true',
        help="清除手指错误状态（不能与其他参数共用）"
    )
    
    # 解析命令行参数
    parsed_args = parser.parse_args(args)
    
    # ========== 参数验证 ==========
    
    # 处理 --ids 参数
    joint_ids = None
    if parsed_args.ids is not None:
        try:
            joint_ids = [id_str.strip() for id_str in parsed_args.ids.split(',')]
            # 对关节ID进行去重
            joint_ids = list(set(joint_ids))
            
            # 验证关节ID是否在有效范围内
            valid_ids = set(['1', '2', '3', '4', '5', '6'])
            invalid_ids = set(joint_ids) - valid_ids
            if invalid_ids:
                print(f"错误：无效的关节ID {invalid_ids}，有效值为 1-6")
                return
            if not joint_ids:
                print("错误：--ids 参数不能为空")
                return
        except Exception as e:
            print(f"错误：--ids 参数格式错误，应为逗号分隔的数字（如 1,2,3）")
            return
    
    if parsed_args.clear_error:
        # 清除错误模式下，不能有其他参数
        if parsed_args.pos is not None or parsed_args.tor is not None or parsed_args.spd is not None:
            print("错误：--clear_error 参数不能与 --pos、--tor 或 --spd 参数共用")
            return
    else:
        # 非清除错误模式下，至少需要一个参数
        if parsed_args.pos is None and parsed_args.tor is None and parsed_args.spd is None:
            parser.print_help()
            print("\n错误：至少需要指定 --pos、--tor、--spd 中的一个参数，或使用 --clear_error")
            return
    
    # ========== ROS2初始化和运行 ==========
    
    rclpy.init(args=args)
    
    # 创建节点实例
    node = InspireHandControllerDemo(
        pos=parsed_args.pos,
        tor=parsed_args.tor,
        spd=parsed_args.spd,
        clear_error=parsed_args.clear_error,
        joint_ids=joint_ids
    )
    
    try:
        # 执行动作
        if parsed_args.clear_error:
            node.clear_error()
        else:
            node.execute_hand_grasp()
        
        # 动作执行完毕，正常退出
        node.get_logger().info("程序执行完成，退出")
        
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号
        print("\n程序被用户中断")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
