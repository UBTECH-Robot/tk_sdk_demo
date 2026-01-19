#!/usr/bin/env python3
from datetime import datetime
import glob
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAX_FILES = 100
SAVE_DIR = 'json_data'

class ASRMonitor(Node):
    def __init__(self):
        super().__init__('asr_monitor')
        self.topic_name = '/xunfei/aiui_msg'
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.msgs_callback,
            10
        )
        
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.get_logger().info(f"已订阅话题：{self.topic_name}")

    def msgs_callback(self, msg: String):
        json_data = None
        try:
            json_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON. Skipping.")
            return

        # 如果在这里保存就所有消息都保存了，其实没有必要
        # self.try_save_json_file(json_data)

        self.cleanup_old_files()

        self.try_parse_and_print(json_data)

    def try_save_json_file(self, json_data):
        try:
            os.makedirs(SAVE_DIR,exist_ok=True)
            timestamp = datetime.now().strftime('%H%M%S%f')[:-3]  # 时分秒+毫秒（保留3位）
            filename = f"{timestamp}.json"
            filepath = os.path.join(SAVE_DIR, filename)
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(json_data, f, ensure_ascii=False, indent=2)
            self.get_logger().info(f"Saved JSON to {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON file: {e}")
        
    def try_parse_and_print(self, json_data):
        if not json_data:
            return
        if "content" not in json_data or "result" not in json_data.get("content"):
            return
        
        result = json_data.get("content").get("result")
        if not result:
            return
        
        if "cbm_meta" not in result:
            return
        

        if not result.get("cbm_meta"):
            return
        cbm_meta = result.get("cbm_meta")        
        if "text" not in cbm_meta:
            return
        
        text_data = json.loads(cbm_meta.get("text"))
        if not text_data:
            return
        key = next(iter(text_data))
        if key not in result:
            return
        result_text = result.get(key).get("text")        
        try:
            res_data = json.loads(result_text)
            print(json.dumps(res_data, indent=2, ensure_ascii=False))

        except json.JSONDecodeError:
            self.get_logger().info(f"content.result.{key}.text: ")
            print(result_text)

        self.try_save_json_file(json_data)

        if result_text:
            print("-" * 20 + "\r\n")

    def cleanup_old_files(self):
        files = sorted(
            glob.glob(os.path.join(SAVE_DIR, '*.json')),
            key=os.path.getmtime
        )
        if len(files) > MAX_FILES:
            to_delete = files[:len(files) - MAX_FILES]
            for f in to_delete:
                try:
                    os.remove(f)
                    self.get_logger().info(f"Deleted old file: {f}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to delete {f}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ASRMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
