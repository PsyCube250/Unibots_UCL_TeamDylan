import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

class GyroPublisher(Node):
    def __init__(self):
        super().__init__('gyro_publisher_node')
        self.pub = self.create_publisher(Vector3, '/rotational_vel', 10)
        
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.sensor = LSM6DSOX(i2c)
        
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Vector3()
        acc = self.sensor.acceleration
        gyro = self.sensor.gyro
        
        msg.x = float(gyro[0])
        msg.y = float(gyro[1])
        msg.z = float(gyro[2])
        
        self.pub.publish(msg)
        
        self.get_logger().info(f"ACC X: {acc[0]:.2f} m/s^2 | GYRO X: {msg.x:.2f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = GyroPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
