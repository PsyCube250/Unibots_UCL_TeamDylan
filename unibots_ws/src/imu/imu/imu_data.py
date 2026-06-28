import json
import math
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def signed16(lo, hi):
    value = int(lo) | (int(hi) << 8)
    return value - 65536 if value & 0x8000 else value


class ImuBus:
    LSM6DS_ADDRS = (0x6A, 0x6B)
    MPU6050_ADDRS = (0x68, 0x69)

    def __init__(self, bus_arg='auto', address_arg='auto', calibrate_s=0.5):
        self.bus_arg = str(bus_arg)
        self.address_arg = str(address_arg)
        self.calibrate_s = max(0.0, float(calibrate_s))
        self.bus = None
        self.bus_no = None
        self.address = None
        self.sensor = ''
        self.bias_z = 0.0
        self.yaw_deg = 0.0
        self.last_s = 0.0
        self.last_ok_s = 0.0
        self.reason = 'not opened'

    def open(self):
        try:
            import smbus2 as smbus
        except Exception:
            try:
                import smbus
            except Exception as exc:
                self.reason = f'smbus unavailable: {exc}'
                return False

        buses = [int(self.bus_arg)] if self.bus_arg != 'auto' else [1, 0, 7, 2, 5, 9]
        addrs = (
            [int(self.address_arg, 0)]
            if self.address_arg != 'auto'
            else list(self.LSM6DS_ADDRS) + list(self.MPU6050_ADDRS)
        )

        for bus_no in buses:
            try:
                bus = smbus.SMBus(bus_no)
            except Exception:
                continue
            found = self.detect(bus, addrs)
            if found is None:
                try:
                    bus.close()
                except Exception:
                    pass
                continue
            self.sensor, self.address = found
            self.bus = bus
            self.bus_no = bus_no
            self.configure()
            self.last_s = time.monotonic()
            self.last_ok_s = self.last_s
            self.calibrate()
            self.reason = 'ok'
            return True

        self.reason = 'no supported IMU found'
        return False

    def detect(self, bus, addrs):
        for addr in addrs:
            if addr in self.LSM6DS_ADDRS:
                try:
                    who = bus.read_byte_data(addr, 0x0F)
                    if who in (0x69, 0x6A, 0x6C):
                        return 'LSM6DS', addr
                except Exception:
                    pass
            if addr in self.MPU6050_ADDRS:
                try:
                    who = bus.read_byte_data(addr, 0x75)
                    if who in (0x68, 0x69, 0x70, 0x71):
                        return 'MPU6050', addr
                except Exception:
                    pass
        return None

    def configure(self):
        if self.sensor == 'LSM6DS':
            self.bus.write_byte_data(self.address, 0x10, 0x40)
            self.bus.write_byte_data(self.address, 0x11, 0x40)
        elif self.sensor == 'MPU6050':
            self.bus.write_byte_data(self.address, 0x6B, 0x00)
            self.bus.write_byte_data(self.address, 0x1B, 0x00)
            self.bus.write_byte_data(self.address, 0x1C, 0x00)

    def raw(self):
        if self.sensor == 'LSM6DS':
            data = self.bus.read_i2c_block_data(self.address, 0x22, 12)
            gx = signed16(data[0], data[1])
            gy = signed16(data[2], data[3])
            gz = signed16(data[4], data[5])
            ax = signed16(data[6], data[7])
            ay = signed16(data[8], data[9])
            az = signed16(data[10], data[11])
            gyro = (
                gx * 0.00875 * math.pi / 180.0,
                gy * 0.00875 * math.pi / 180.0,
                gz * 0.00875 * math.pi / 180.0,
            )
            accel = (
                ax * 0.000061 * 9.80665,
                ay * 0.000061 * 9.80665,
                az * 0.000061 * 9.80665,
            )
            return gyro, accel

        if self.sensor == 'MPU6050':
            accel_data = self.bus.read_i2c_block_data(self.address, 0x3B, 6)
            gyro_data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
            ax = signed16(accel_data[1], accel_data[0])
            ay = signed16(accel_data[3], accel_data[2])
            az = signed16(accel_data[5], accel_data[4])
            gx = signed16(gyro_data[1], gyro_data[0])
            gy = signed16(gyro_data[3], gyro_data[2])
            gz = signed16(gyro_data[5], gyro_data[4])
            gyro = (gx / 131.0 * math.pi / 180.0, gy / 131.0 * math.pi / 180.0, gz / 131.0 * math.pi / 180.0)
            accel = (ax / 16384.0 * 9.80665, ay / 16384.0 * 9.80665, az / 16384.0 * 9.80665)
            return gyro, accel

        raise RuntimeError(f'unsupported IMU {self.sensor}')

    def calibrate(self):
        if self.calibrate_s <= 0.0:
            return
        values = []
        deadline = time.monotonic() + self.calibrate_s
        while time.monotonic() < deadline:
            gyro, _accel = self.raw()
            values.append(gyro[2])
            time.sleep(0.01)
        if values:
            self.bias_z = sum(values) / len(values)

    def read(self):
        now = time.monotonic()
        if self.bus is None:
            return False, (0.0, 0.0, 0.0), None, 999.0, self.reason
        try:
            gyro, accel = self.raw()
            dt = max(0.0, min(0.20, now - self.last_s)) if self.last_s else 0.0
            gyro_z = gyro[2] - self.bias_z
            self.yaw_deg = (self.yaw_deg + gyro_z * dt * 180.0 / math.pi) % 360.0
            self.last_s = now
            self.last_ok_s = now
            self.reason = 'ok'
            return True, (gyro[0], gyro[1], gyro_z), accel, 0.0, 'ok'
        except Exception as exc:
            age = now - self.last_ok_s if self.last_ok_s else 999.0
            self.reason = f'imu read failed: {exc}'
            return False, (0.0, 0.0, 0.0), None, age, self.reason

    def close(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None


class GyroPublisher(Node):
    def __init__(self):
        super().__init__('gyro_publisher_node')
        self.declare_parameter('bus', 'auto')
        self.declare_parameter('address', 'auto')
        self.declare_parameter('calibrate_seconds', 0.5)
        self.declare_parameter('publish_hz', 20.0)

        self.rot_pub = self.create_publisher(Vector3, '/rotational_vel', 10)
        self.state_pub = self.create_publisher(String, '/imu_state', 10)

        self.imu = ImuBus(
            self.get_parameter('bus').value,
            self.get_parameter('address').value,
            float(self.get_parameter('calibrate_seconds').value),
        )
        if self.imu.open():
            self.get_logger().info(
                f'IMU opened: {self.imu.sensor} i2c-{self.imu.bus_no}@0x{self.imu.address:02x}'
            )
        else:
            self.get_logger().warn(f'IMU unavailable: {self.imu.reason}')

        hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self.timer = self.create_timer(1.0 / hz, self.timer_callback)

    def timer_callback(self):
        ok, gyro, accel, age_s, reason = self.imu.read()

        msg = Vector3()
        msg.x = float(gyro[0])
        msg.y = float(gyro[1])
        msg.z = float(gyro[2])
        self.rot_pub.publish(msg)

        state = String()
        state.data = json.dumps({
            'enabled': self.imu.bus is not None,
            'ok': ok,
            'sensor': self.imu.sensor,
            'bus': self.imu.bus_no,
            'address': self.imu.address,
            'yaw_deg': round(self.imu.yaw_deg, 2) if self.imu.bus is not None else None,
            'gyro_z_rad_s': round(float(gyro[2]), 5),
            'accel_m_s2': [round(float(v), 4) for v in accel] if accel is not None else None,
            'age_s': round(age_s, 3),
            'reason': reason,
            'timestamp': time.time(),
        })
        self.state_pub.publish(state)

    def destroy_node(self):
        self.imu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GyroPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
