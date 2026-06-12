"""
ROS2 node that bridges the 'motor_speeds' topic to the STM32 over serial.
Subscribes to std_msgs/Int32MultiArray with 4 values (-100 to 100)
and sends framed packets over UART to the STM32.

Usage:
  ros2 run your_package jetson_serial_bridge
  or just: python3 jetson_serial_bridge.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

SERIAL_PORT = '/dev/ttyTHS1'  # Jetson Orin Nano UART1 (GPIO pins 8=TX, 10=RX)
BAUD_RATE = 115200
HEADER = bytes([0xFF, 0xFE])


class SerialBridge(Node):
    def __init__(self):
        super().__init__('stm32_serial_bridge')
        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baud', BAUD_RATE)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Connected to STM32 on {port} at {baud}')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'motor_speeds',
            self.callback,
            10)

    def callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn('Expected 4 values, got %d' % len(msg.data))
            return

        # Clamp and offset: speed (-100..100) -> wire byte (0..200)
        wire = []
        for i in range(4):
            speed = max(-100, min(100, msg.data[i]))
            wire.append(speed + 100)

        checksum = sum(wire) & 0xFF
        frame = HEADER + bytes(wire) + bytes([checksum])
        self.ser.write(frame)

    def destroy_node(self):
        # Stop motors on shutdown
        wire = [100, 100, 100, 100]  # all zeros (offset)
        checksum = sum(wire) & 0xFF
        frame = HEADER + bytes(wire) + bytes([checksum])
        self.ser.write(frame)
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
