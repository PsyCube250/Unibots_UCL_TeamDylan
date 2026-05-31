import board
import busio
import time

from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

i2c = busio.I2C(board.SCL_1, board.SDA_1)

sensor = LSM6DSOX(i2c)

class SensorData:
    def __init__(self, acceleration, gyro):
        self.x_acc, self.y_acc, self.z_acc = acceleration
        self.x_gyro, self.y_gyro, self.z_gyro = gyro

while True:
    IMU_data = SensorData(sensor.acceleration, sensor.gyro)

    print(f"ACC X: {IMU_data.x_acc:.2f} m/s^2 | GYRO X: {IMU_data.x_gyro:.2f} rad/s")
    
    time.sleep(0.5)