"""Publish ROS 2 velocity commands while keys are held in a small window."""

import argparse
import math
import tkinter as tk

from geometry_msgs.msg import Twist
import rclpy


def positive(value):
    """Parse a finite positive speed."""
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('Speed must be finite and positive')
    return result


class Keyboard:
    """Track key state, including X11 repeat and window focus changes."""

    def __init__(self, root, node, speed, turn):
        """Create the keyboard window and ROS publisher."""
        self.root = root
        self.node = node
        self.speed = speed
        self.turn = turn
        self.keys = set()
        self.releases = {}
        self.publisher = node.create_publisher(Twist, 'cmd_vel', 1)
        root.title('RB-KAIROS WASD')
        tk.Label(root, text='Hold W/S: forward/backward\n'
                 'Hold A/D: turn left/right\n'
                 'Space: stop while held | Escape: quit\n'
                 'Keep this window focused', padx=25, pady=20).pack()
        self.status = tk.StringVar()
        tk.Label(root, textvariable=self.status, padx=15, pady=10).pack()
        root.bind('<KeyPress>', self.press)
        root.bind('<KeyRelease>', self.release)
        root.bind('<FocusOut>', self.stop)
        root.protocol('WM_DELETE_WINDOW', root.quit)
        self.tick()

    def press(self, event):
        """Record keys and cancel synthetic key releases during repeat."""
        key = event.keysym.lower()
        pending = self.releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        if key == 'escape':
            self.root.quit()
        elif key in ('w', 'a', 's', 'd', 'space'):
            self.keys.add(key)

    def release(self, event):
        """Defer releases until pending repeat presses have been handled."""
        key = event.keysym.lower()
        pending = self.releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.releases[key] = self.root.after_idle(self.finish_release, key)

    def finish_release(self, key):
        """Remove a genuinely released key."""
        self.releases.pop(key, None)
        self.keys.discard(key)

    def stop(self, event=None):
        """Clear held keys and send a zero command immediately."""
        self.keys.clear()
        for pending in self.releases.values():
            self.root.after_cancel(pending)
        self.releases.clear()
        if rclpy.ok():
            self.publisher.publish(Twist())

    def tick(self):
        """Publish at 20 Hz using wall time, even when simulation is paused."""
        if not rclpy.ok():
            self.root.quit()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        msg = Twist()
        if 'space' not in self.keys:
            msg.linear.x = self.speed * (('w' in self.keys) - ('s' in self.keys))
            msg.angular.z = self.turn * (('a' in self.keys) - ('d' in self.keys))
        self.publisher.publish(msg)
        count = self.publisher.get_subscription_count()
        self.status.set(f'v={msg.linear.x:.2f} m/s  yaw={msg.angular.z:.2f} rad/s\n'
                        f'ROS subscribers: {count}'
                        + (' — connect Isaac drive controller' if count == 0 else ''))
        self.root.after(50, self.tick)


def main():
    """Start the keyboard window and release the robot on normal exit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=positive, default=0.25)
    parser.add_argument('--turn', type=positive, default=1.0)
    args, ros_args = parser.parse_known_args()
    root = tk.Tk()
    node = None
    keyboard = None
    try:
        rclpy.init(args=ros_args)
        node = rclpy.create_node('rbkairos_wasd')
        keyboard = Keyboard(root, node, args.speed, args.turn)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if keyboard is not None:
            keyboard.stop()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()


if __name__ == '__main__':
    main()
