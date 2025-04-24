import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopTwistKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.settings = termios.tcgetattr(sys.stdin)

        self.linear_velocity = 0.5  # Velocidad lineal predeterminada
        self.angular_velocity = 1.0  # Velocidad angular predeterminada

        self.key_bindings = {
            'w': (1, 0),
            's': (-1, 0),
            'a': (0, 1),
            'd': (0, -1),
            'q': (0, 0),
        }

        self.get_logger().info("Listo para controlar el robot. Usa las teclas W/A/S/D para moverte y Q para detener.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in self.key_bindings:
                    linear, angular = self.key_bindings[key]

                    twist = Twist()
                    twist.linear.x = self.linear_velocity * linear
                    twist.angular.z = self.angular_velocity * angular
                    self.publisher_.publish(twist)

                    if key == 'q':
                        break

        except Exception as e:
            self.get_logger().error(f"Ocurrió un error: {str(e)}")

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)

    teleop_twist_keyboard = TeleopTwistKeyboard()

    teleop_twist_keyboard.run()

    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
