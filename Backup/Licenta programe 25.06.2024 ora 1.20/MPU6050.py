import smbus
import math
import time
import numpy as np

class MPU6050:
    def __init__(self, bus_number=1, device_address=0x68):
        self.bus = smbus.SMBus(bus_number)
        self.Device_Address = device_address
        self.ACCEL_SCALE_FACTOR = 16384.0
        self.GYRO_SCALE_FACTOR = 131.0
        self.Axoffset = 0.00498
        self.Ayoffset = 0.03055
        self.Azoffset = 0.0029
        self.Gx_offset = -0.19
        self.Gy_offset = -0.09
        self.Gz_offset = 0.15
        self.P_pitch = np.array([[0, 0], [0, 0]], dtype=float)
        self.K_pitch = np.array([0, 0], dtype=float)
        self.P_roll = np.array([[0, 0], [0, 0]], dtype=float)
        self.K_roll = np.array([0, 0], dtype=float)
        self.Q_unghi_pitch = 0.001
        self.Q_gyro_pitch = 0.003
        self.R_unghi_pitch = 0.03
        self.Q_unghi_roll = 0.001
        self.Q_gyro_roll = 0.003
        self.R_unghi_roll = 0.03
        self.dt = 0.0065
        self.MPU_Init()

    def MPU_Init(self):
        self.bus.write_byte_data(self.Device_Address, 0x6B, 1)
        self.bus.write_byte_data(self.Device_Address, 0x1A, 0)
        self.bus.write_byte_data(self.Device_Address, 0x1B, 24)
        self.bus.write_byte_data(self.Device_Address, 0x38, 1)
        self.bus.write_byte_data(self.Device_Address, 26, 5)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value -= 65536
        return value

    def kalman_filter(self, unghi, P, K, Q_unghi, Q_gyro, R_unghi, rata_gyro):
        unghi += self.dt * rata_gyro
        P[0][0] += self.dt * (self.dt * P[1][1] - P[0][1] - P[1][0] + Q_unghi)
        P[0][1] -= self.dt * P[1][1]
        P[1][0] -= self.dt * P[1][1]
        P[1][1] += Q_gyro * self.dt
        y = unghi - unghi
        S = P[0][0] + R_unghi
        K[0] = P[0][0] / S
        K[1] = P[1][0] / S
        unghi += K[0] * y
        rata_gyro -= unghi
        unghi += K[1] * rata_gyro
        P00_temp = P[0][0]
        P01_temp = P[0][1]
        P[0][0] -= K[0] * P00_temp
        P[0][1] -= K[0] * P01_temp
        P[1][0] -= K[1] * P00_temp
        P[1][1] -= K[1] * P01_temp
        return unghi

    def get_pitch_roll(self):
        acc_x = (self.read_raw_data(0x3B) / self.ACCEL_SCALE_FACTOR) - self.Axoffset
        acc_y = (self.read_raw_data(0x3D) / self.ACCEL_SCALE_FACTOR) - self.Ayoffset
        acc_z = (self.read_raw_data(0x3F) / self.ACCEL_SCALE_FACTOR) - self.Azoffset
        gyro_x = (self.read_raw_data(0x43) / self.GYRO_SCALE_FACTOR) - self.Gx_offset
        gyro_y = (self.read_raw_data(0x45) / self.GYRO_SCALE_FACTOR) - self.Gy_offset
        gyro_z = (self.read_raw_data(0x47) / self.GYRO_SCALE_FACTOR) - self.Gz_offset
        pitch = math.degrees(math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2)))
        roll = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2)))
        unghi_pitch = self.kalman_filter(pitch, self.P_pitch, self.K_pitch, self.Q_unghi_pitch, self.Q_gyro_pitch, self.R_unghi_pitch, gyro_x)
        unghi_roll = self.kalman_filter(roll, self.P_roll, self.K_roll, self.Q_unghi_roll, self.Q_gyro_roll, self.R_unghi_roll, gyro_y)
        return unghi_pitch, unghi_roll, acc_x, acc_y, acc_z

    def vertical_velocity(self, Pitch, Roll, Ax, Ay, Az, ZVelocity, Time):
        AccZInertial = -Ax * math.sin(Pitch * math.pi / 180) + Ay * math.cos(Pitch * math.pi / 180) * math.sin(Roll * math.pi / 180) + Az * math.cos(Pitch * math.pi / 180) * math.cos(Roll * math.pi / 180)
        AccZInertial = (AccZInertial - 1) * 9.81 * 10
        ZVelocity = ZVelocity + AccZInertial * Time
        return ZVelocity


if __name__ == "__main__":
        mpu = MPU6050()
        z_velocity = 0
        try:
            while True:
                pitch, roll, acc_x, acc_y, acc_z = mpu.get_pitch_roll()
                z_velocity = mpu.vertical_velocity(pitch, roll, acc_x, acc_y, acc_z, z_velocity, mpu.dt)
                print(f"MPU6050 - Pitch: {pitch:.2f}, Roll: {roll:.2f}, Z Velocity: {z_velocity:.2f}")
                time.sleep(0)
        except KeyboardInterrupt:
                print("Iesire...")
