#!/usr/bin/env python3
"""
音频文件播放节点 (Audio File Player Node)
==========================================

此模块通过调用远程 lyre 服务播放音频文件。

主要功能:
    1. 调用远程音频播放服务 (/audio_play/play_file)
    2. 支持强制打断播放
    3. 支持同步和异步调用
    4. 支持命令行参数指定音频文件路径

服务依赖:
    - 服务名: /audio_play/play_file
    - 服务类型: lyre_msgs/srv/PlayFile
    - 服务提供者: lyre (运行在 192.168.41.2)

PlayFile 服务接口:
    请求字段:
        - sid (string): 播放流 ID，建议保持唯一
        - seq (uint32): 流式序列号，单包播放时通常为 0
        - last (bool): 是否为最后一个包，单包播放时通常为 True
        - force (bool): 是否强制打断当前播放
        - path (string): 要播放的音频文件路径

    响应字段:
        - sid (string): 播放流 ID
        - code (int8): 结果码，0 成功，1 参数无效，-1 失败
        - message (string): 结果说明

使用方法:
    1. 确保 lyre 服务已启动
       如果 lyre 服务未启动，可在 192.168.41.2 上执行:
       cd /home/nvidia/lyre_ros2 && . install/setup.bash && ros2 launch lyre audio.launch.py

    2. 配置环境变量（根据运行平台选择）
       # Orin 板
       source ~/lyre_ros2/install/setup.bash
       # 或 x86 板
       source ~/ros2ws/install/setup.bash

    3. 运行节点
       ros2 run sdk_demo audio_player_lyre /path/to/audio.wav

       或直接传参:
       ros2 run sdk_demo audio_player_lyre --path /path/to/audio.wav --force

    4. 也可以直接执行脚本
       python3 src/sdk_demo/sdk_demo/9_audio_player_lyre.py /path/to/audio.wav

         # 等效的 ros2 service call 命令（把 path 替换成你的音频文件路径）
         ros2 service call /audio_play/play_file lyre_msgs/srv/PlayFile "{sid: 'audio_play_001', seq: 0, last: true, force: false, path: '/home/nvidia/sdk_demo/saved_data/audio_files/audio_113335553.wav'}"

注意事项:
    - 需要先确保 lyre 服务已启动
    - 音频文件路径通常应是服务端可访问的有效路径
    - 建议使用绝对路径，避免服务端路径解析歧义
    - 如果当前有语音/音乐正在播放，使用 --force 可强制打断
"""

from __future__ import annotations

import argparse
import os
import sys
import uuid

import rclpy
from rclpy.node import Node

try:
    from lyre_msgs.srv import PlayFile
except ImportError:
    print("=" * 70)
    print("错误: 无法导入 lyre_msgs.srv.PlayFile")
    print("=" * 70)
    print("请先配置 lyre_ros2 环境变量:")
    print("")
    print("  如果在天工 Orin 板上运行:")
    print("    source ~/lyre_ros2/install/setup.bash")
    print("")
    print("  如果在天工 x86 板上运行:")
    print("    source ~/ros2ws/install/setup.bash")
    print("")
    print("然后重新运行此节点:")
    print("  ros2 run sdk_demo audio_player_lyre /path/to/audio.wav")
    print("=" * 70)
    sys.exit(1)


class AudioFilePlayerNode(Node):
    """音频文件播放节点。"""

    def __init__(self):
        super().__init__('audio_player_lyre_node')

        self.service_name = '/audio_play/play_file'
        self.service_timeout = 10.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('正在初始化音频文件播放节点...')
        self.get_logger().info('=' * 60)

        self.client = self.create_client(PlayFile, self.service_name)
        self.get_logger().info(f'已创建服务客户端: {self.service_name}')

        if not self.check_service():
            self.get_logger().error('=' * 60)
            self.get_logger().error(f'未发现服务: {self.service_name}')
            self.get_logger().error('请确认 192.168.41.2 上的 lyre 服务是否启动')
            self.get_logger().error('检查项:')
            self.get_logger().error('  1. lyre 服务是否正在运行')
            self.get_logger().error('  2. 网络连接是否正常 (ping 192.168.41.2)')
            self.get_logger().error('  3. 运行环境是否已 source 对应 setup.bash')
            self.get_logger().error('=' * 60)
            sys.exit(1)

        self.get_logger().info('音频文件播放服务已就绪，节点初始化完成')

    def check_service(self) -> bool:
        """检查服务是否可用。"""
        self.get_logger().info(f'正在等待服务: {self.service_name}')
        self.get_logger().info(f'超时时间: {self.service_timeout} 秒')

        service_available = self.client.wait_for_service(timeout_sec=self.service_timeout)
        if service_available:
            self.get_logger().info(f'服务已发现: {self.service_name}')
            return True

        self.get_logger().warning(f'服务等待超时 ({self.service_timeout} 秒)')
        return False

    def _resolve_audio_path(self, path: str) -> tuple[bool, str, str]:
        """
        解析并检查音频文件路径。

        返回:
            tuple[bool, str, str]: (是否有效, 规范化路径, 提示信息)
        """
        if not path:
            return False, '', '音频文件路径不能为空'

        normalized_path = os.path.abspath(os.path.expanduser(path))
        if not os.path.isfile(normalized_path):
            return False, normalized_path, f'音频文件不存在: {normalized_path}'

        return True, normalized_path, ''

    def play_file(
        self,
        path: str,
        force: bool = False,
        sid: str | None = None,
        seq: int = 0,
        last: bool = True,
        timeout: float = 30.0,
    ) -> tuple[bool, str, str]:
        """
        播放音频文件（同步调用）。

        参数:
            path (str): 音频文件路径
            force (bool): 是否强制播放，默认 False
            sid (str): 播放流 ID；如果不传则自动生成 UUID
            seq (int): 序列号，单包播放默认 0
            last (bool): 是否为最后一个包，默认 True
            timeout (float): 服务调用超时时间（秒）

        返回:
            tuple[bool, str, str]: (success, sid, message)
        """
        if not path:
            message = '音频文件路径不能为空'
            self.get_logger().error(message)
            return False, sid or '', message

        is_valid, normalized_path, message = self._resolve_audio_path(path)
        if not is_valid:
            self.get_logger().error(message)
            return False, sid or '', message

        if sid is None:
            sid = str(uuid.uuid4())

        self.get_logger().info(
            f'正在请求播放文件: "{normalized_path}" (force={force}, sid={sid})'
        )

        request = PlayFile.Request()
        request.sid = sid
        request.seq = seq
        request.last = last
        request.force = force
        request.path = normalized_path

        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

            if not future.done():
                message = f'服务调用超时 ({timeout} 秒)'
                self.get_logger().error(message)
                return False, sid, message

            response = future.result()
            if response.code == 0:
                self.get_logger().info(
                    f'播放成功: {response.message} (sid={response.sid})'
                )
                return True, response.sid, response.message

            if response.code == 1:
                self.get_logger().error(f'参数无效: {response.message}')
                return False, response.sid, response.message

            self.get_logger().error(f'播放失败: {response.message}')
            return False, response.sid, response.message

        except Exception as exc:
            message = f'服务调用异常: {exc}'
            self.get_logger().error(message)
            return False, sid, message

    def play_file_async(
        self,
        path: str,
        callback=None,
        force: bool = False,
        sid: str | None = None,
        **kwargs,
    ):
        """
        播放音频文件（异步调用）。

        参数:
            path (str): 音频文件路径
            callback (callable): 回调函数，签名为 callback(future)
            force (bool): 是否强制播放
            sid (str): 播放流 ID
            **kwargs: 其他请求字段（seq、last）
        """
        if sid is None:
            sid = str(uuid.uuid4())

        is_valid, normalized_path, message = self._resolve_audio_path(path)
        if not is_valid:
            self.get_logger().error(message)
            return None

        self.get_logger().info(f'异步请求播放文件: "{normalized_path}" (sid={sid})')

        request = PlayFile.Request()
        request.sid = sid
        request.seq = int(kwargs.get('seq', 0))
        request.last = bool(kwargs.get('last', True))
        request.force = force
        request.path = normalized_path

        future = self.client.call_async(request)
        if callback is not None:
            future.add_done_callback(callback)
        return future


def build_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description='调用 /audio_play/play_file 服务播放音频文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  ros2 run sdk_demo audio_player_lyre /path/to/audio.wav
  ros2 run sdk_demo audio_player_lyre --path /path/to/audio.wav --force
  python3 src/sdk_demo/sdk_demo/9_audio_player_lyre.py /path/to/audio.wav
''',
    )
    parser.add_argument('audio_path', nargs='?', help='要播放的音频文件路径')
    parser.add_argument('--path', dest='path', help='要播放的音频文件路径（等价于位置参数）')
    parser.add_argument('--force', action='store_true', help='强制打断当前播放')
    parser.add_argument('--sid', default=None, help='自定义播放流 ID')
    parser.add_argument('--seq', type=int, default=0, help='序列号，单包播放通常为 0')
    parser.add_argument('--last', dest='last', action='store_true', help='标记为最后一个包')
    parser.add_argument('--not-last', dest='last', action='store_false', help='标记为非最后一个包')
    parser.set_defaults(last=True)
    parser.add_argument('--timeout', type=float, default=30.0, help='服务调用超时时间（秒）')
    parser.add_argument('--service-timeout', type=float, default=10.0, help='等待服务可用的超时时间（秒）')
    return parser


def main(args=None):
    parser = build_parser()
    parsed_args, _unknown_args = parser.parse_known_args(args=args)

    audio_path = parsed_args.path or parsed_args.audio_path
    if not audio_path:
        parser.error('必须提供音频文件路径，可以使用位置参数或 --path')

    resolved_audio_path = os.path.abspath(os.path.expanduser(audio_path))
    if not os.path.isfile(resolved_audio_path):
        print('=' * 70)
        print(f'错误: 音频文件不存在: {resolved_audio_path}')
        print('请检查路径是否正确，并确认文件已存在后再运行。')
        print('=' * 70)
        return 1

    rclpy.init(args=args)
    node = None

    try:
        node = AudioFilePlayerNode()
        node.service_timeout = parsed_args.service_timeout

        success, sid, message = node.play_file(
            path=resolved_audio_path,
            force=parsed_args.force,
            sid=parsed_args.sid,
            seq=parsed_args.seq,
            last=parsed_args.last,
            timeout=parsed_args.timeout,
        )

        if success:
            node.get_logger().info('=' * 60)
            node.get_logger().info(f'播放任务已提交成功，sid={sid}')
            node.get_logger().info(message)
            node.get_logger().info('=' * 60)
        else:
            node.get_logger().error('=' * 60)
            node.get_logger().error(f'播放任务执行失败，sid={sid}')
            node.get_logger().error(message)
            node.get_logger().error('=' * 60)

        return 0 if success else 1

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('收到中断信号，节点退出')
        return 130

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())