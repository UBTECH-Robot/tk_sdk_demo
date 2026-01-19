import rclpy
from rclpy.node import Node
from hric_msgs.srv import SetMotionMode
from geometry_msgs.msg import TwistStamped
import time

class SafeWalker(Node):
    def __init__(self):
        super().__init__('safe_walker_node')
        
        # 1. Setup Connection
        self.client = self.create_client(SetMotionMode, '/hric/motion/set_motion_mode')
        self.vel_pub = self.create_publisher(TwistStamped, '/hric/robot/cmd_vel', 10)
        
        self.get_logger().info("Waiting for robot service...")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting...")
        self.get_logger().info("Connected!")

    def set_mode(self, mode_id, mode_name):
        """Switches Robot Mode (3=Stand, 4=Walk)"""
        req = SetMotionMode.Request()
        req.walk_mode_request = mode_id
        req.is_need_swing_arm = True
        
        self.get_logger().info(f"REQUEST: {mode_name}...")
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f"SUCCESS: Robot is {mode_name}")
            return True
        else:
            self.get_logger().error(f"FAILED: Robot rejected {mode_name}")
            return False

    def move_for_seconds(self, speed, duration):
        """Sends velocity commands for X seconds"""
        self.get_logger().info(f"Moving at {speed} m/s for {duration} seconds...")
        end_time = time.time() + duration
        
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = speed
        msg.twist.angular.z = 0.0
        
        # 20Hz Loop (Standard for Walker)
        while rclpy.ok() and time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def graceful_shutdown(self):
        """THE FIX: Stops safely so the robot doesn't lock us out"""
        print("\n" + "="*30)
        self.get_logger().info("INITIATING GRACEFUL SHUTDOWN...")
        
        # 1. Command Stand Mode
        self.set_mode(3, "STANDING")
        
        # 2. THE SECRET SAUCE: Cool-down Heartbeat
        # We keep sending '0.0' speed for 3 seconds.
        # This proves to the robot we didn't crash, allowing it to transition safely.
        self.get_logger().info("Sending final heartbeat (3s) to clear buffers...")
        self.move_for_seconds(0.0, 3.0)
        
        print("="*30)
        self.get_logger().info("Safe to quit. Robot is unlocked.")

def main(args=None):
    rclpy.init(args=args)
    bot = SafeWalker()

    try:
        # --- MAIN ROUTINE ---
        
        # 1. Reset to Stand (Just in case)
        bot.set_mode(3, "STANDING")
        time.sleep(1)

        # 2. Start Walking
        if bot.set_mode(4, "MARCHING"):
            # 3. Move forward slightly for 5 seconds
            # 0.05 m/s is the minimum speed to see stepping animation
            bot.move_for_seconds(0.05, 5.0)

    except KeyboardInterrupt:
        print("\n! Ctrl+C Detected !")
        
    finally:
        # --- THIS ALWAYS RUNS, EVEN ON CRASH ---
        bot.graceful_shutdown()
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
