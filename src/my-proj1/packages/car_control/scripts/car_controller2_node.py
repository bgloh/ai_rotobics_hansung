#!/usr/bin/env python3
import os, rospy, threading, curses
from duckietown_msgs.msg import Twist2DStamped

KEYS = {
    ord('w'): ('v', +1),
    ord('s'): ('v', -1),
    ord('a'): ('w', +1),
    ord('d'): ('w', -1),
    ord('q'): ('vw', +1),   # forward-left
    ord('e'): ('vw', -1),   # forward-right
    ord('x'): ('stop', 0),
    ord('z'): ('quit', 0),
}

class CarTeleop:
    def __init__(self, stdscr):
        rospy.init_node('car_controller2_node', anonymous=True)
        self.robot_name = os.environ.get('VEHICLE_NAME', 'duckiealexa')
        self.pub = rospy.Publisher(f"/{self.robot_name}/car_cmd_switch_node/cmd",
                                   Twist2DStamped, queue_size=1)
        self.lock = threading.Lock()

        # 기본 속도(ROS 파라미터로 덮어쓰기 가능)
        self.base_v = rospy.get_param("~linear_speed", 0.3)
        self.base_w = rospy.get_param("~angular_speed", 1.5)

        # 현재 목표 속도 (키 누름 상태 반영)
        self.v_cmd, self.w_cmd = 0.0, 0.0

        # curses 설정
        self.stdscr = stdscr
        curses.curs_set(0)            # 커서 숨김
        stdscr.nodelay(True)          # getch() 논블로킹
        stdscr.keypad(True)           # 특수키 처리
        curses.noecho()               # 키 입력 표시 안함
        curses.cbreak()               # 즉시 키 전달

        self.draw_help()

        # 10 Hz로 명령 유지 퍼블리시
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish)

        rospy.on_shutdown(self.stop)
        rospy.loginfo(f"Car Controller2 Node initialized for robot: {self.robot_name}")
        rospy.loginfo(f"Publishing to topic: /{self.robot_name}/car_cmd_switch_node/cmd")

    def draw_help(self):
        s = self.stdscr
        s.clear()
        s.addstr(0, 0, "=== Duckietown Real-Time Car Teleop ===")
        s.addstr(1, 0, "Single character control - No Enter required")
        s.addstr(2, 0, "")
        s.addstr(3, 0, "Controls:")
        s.addstr(4, 0, "  w = Forward          s = Backward")
        s.addstr(5, 0, "  a = Turn Left        d = Turn Right")
        s.addstr(6, 0, "  q = Forward + Left   e = Forward + Right")
        s.addstr(7, 0, "  x = Stop             z = Quit")
        s.addstr(8, 0, "")
        s.addstr(9, 0, "Status:")
        s.refresh()

    def _publish(self, _evt):
        with self.lock:
            msg = Twist2DStamped()
            msg.header.stamp = rospy.Time.now()
            msg.v = self.v_cmd
            msg.omega = self.w_cmd
        self.pub.publish(msg)

    def stop(self):
        with self.lock:
            self.v_cmd, self.w_cmd = 0.0, 0.0
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = 0.0
        msg.omega = 0.0
        self.pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            ch = self.stdscr.getch()   # 논블로킹: 키 없으면 -1
            if ch == -1:
                rospy.sleep(0.01)
                continue

            action = KEYS.get(ch)
            if not action:
                continue

            kind, sign = action
            with self.lock:
                if kind == 'v':
                    self.v_cmd = sign * self.base_v
                    self.w_cmd = 0.0
                elif kind == 'w':
                    self.v_cmd = 0.0
                    self.w_cmd = sign * self.base_w
                elif kind == 'vw':
                    self.v_cmd = self.base_v
                    self.w_cmd = sign * self.base_w
                elif kind == 'stop':
                    self.v_cmd = 0.0
                    self.w_cmd = 0.0
                elif kind == 'quit':
                    break

            # 상태 표시
            self.stdscr.addstr(10, 0, f"Linear Velocity (v):  {self.v_cmd:+.2f} m/s     ")
            self.stdscr.addstr(11, 0, f"Angular Velocity (ω): {self.w_cmd:+.2f} rad/s   ")
            
            # 현재 동작 표시
            if self.v_cmd > 0 and self.w_cmd == 0:
                action_str = "Moving Forward"
            elif self.v_cmd < 0 and self.w_cmd == 0:
                action_str = "Moving Backward"
            elif self.v_cmd == 0 and self.w_cmd > 0:
                action_str = "Turning Left"
            elif self.v_cmd == 0 and self.w_cmd < 0:
                action_str = "Turning Right"
            elif self.v_cmd > 0 and self.w_cmd > 0:
                action_str = "Forward + Left"
            elif self.v_cmd > 0 and self.w_cmd < 0:
                action_str = "Forward + Right"
            elif self.v_cmd == 0 and self.w_cmd == 0:
                action_str = "Stopped"
            else:
                action_str = "Custom Movement"
                
            self.stdscr.addstr(12, 0, f"Action: {action_str}                    ")
            self.stdscr.refresh()

        self.stop()

def main(stdscr):
    try:
        CarTeleop(stdscr).run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        # curses가 활성화된 상태에서 예외 처리
        curses.endwin()
        rospy.logerr(f"Error in car_controller2_node: {e}")

if __name__ == "__main__":
    curses.wrapper(main)