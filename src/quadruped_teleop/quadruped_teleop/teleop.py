#!/usr/bin/env python3

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController

# --- Configuration ---
# The whole_body_controller listens for velocity commands here
# (linear.x, linear.y [m/s], angular.z [rad/s]).
CMD_VEL_TOPIC = '/vel_cmd'
STAND_CONTROLLER = 'stand_up'
WALK_CONTROLLER = 'whole_body_controller'

msg = """
Quadruped Teleop (whole_body_controller)
----------------------------------------
Hold a key to move, release to stop:
        w
   a    s    d        q/e : turn left/right
        x

w/x : forward/backward
a/d : left/right
s   : stop

Speed Control:
+ / - : Max Linear Speed +/- 10%
* / / : Max Angular Speed +/- 10%

CTRL-C to quit (switches back to the stand_up controller)
"""

moveBindings = {
    'w': (1.0, 0.0, 0.0), 'x': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0), 'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0), 'e': (0.0, 0.0, -1.0),
}

speedBindings = {
    '+': (1.1, 1.0), '-': (0.9, 1.0),
    '*': (1.0, 1.1), '/': (1.0, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class WBCTeleop(Node):
    def __init__(self):
        super().__init__('wbc_teleop')
        self.pub_twist = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        # Service client to talk to the ros2_control controller manager
        self.switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

    def switch(self, activate, deactivate, timeout_sec=5.0):
        if not self.switch_client.wait_for_service(timeout_sec=timeout_sec):
            print("\r\n[ERROR] Controller Manager service not available!")
            return
        req = SwitchController.Request()
        req.activate_controllers = [activate]
        req.deactivate_controllers = [deactivate]
        req.strictness = SwitchController.Request.STRICT
        future = self.switch_client.call_async(req)
        future.add_done_callback(
            lambda f: self.switch_callback(f, activate))

    def switch_callback(self, future, activated):
        try:
            if future.result().ok:
                print(f"\r\n[SUCCESS] Activated {activated}.")
            else:
                print(f"\r\n[FAILED] Could not activate {activated}. Make sure "
                      "the launch file has fully finished loading it!")
        except Exception as e:
            print(f"\r\nService call failed: {e}")

    def activate_wbc(self):
        self.switch(WALK_CONTROLLER, STAND_CONTROLLER)

    def deactivate_wbc(self):
        self.switch(STAND_CONTROLLER, WALK_CONTROLLER, timeout_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WBCTeleop()
    settings = termios.tcgetattr(sys.stdin)

    linear_speed = 0.3
    angular_speed = 0.6
    x = y = th_z = 0.0
    
    print("Initializing teleop and connecting to Controller Manager...")
    node.activate_wbc()

    try:
        print(msg)
        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                x, y, th_z = moveBindings[key]
            
            elif key in speedBindings.keys():
                linear_speed *= speedBindings[key][0]
                angular_speed *= speedBindings[key][1]
                print(f"\rCurrent speeds - Linear: {linear_speed:.2f} Angular: {angular_speed:.2f}")

            else:  # 's', no key held, or any other key: stop
                x = y = th_z = 0.0
                if (key == '\x03'): # CTRL-C
                    print("\r\nShutting down... reverting to stand_up controller.")
                    node.deactivate_wbc()
                    break

            twist = Twist()
            twist.linear.x = x * linear_speed
            twist.linear.y = y * linear_speed
            twist.angular.z = th_z * angular_speed
            node.pub_twist.publish(twist)

            # Process ROS callbacks (allows the service future to resolve)
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        node.pub_twist.publish(twist) # Send zero velocity on shutdown
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()