import smbus
import math
import time
import numpy as np

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
LOWPASSFILTER = 26

# MPU6050 Address
Device_Address = 0x68

# Accelerometer and gyroscope scaling factors
ACCEL_SCALE_FACTOR = 16384.0  # +/- 2g range
GYRO_SCALE_FACTOR = 131.0  # +/- 250 degree/s range

# Calibration offsets
Axoffset = 0.065
Ayoffset = 0.015
Azoffset = 0
Gx_offset = -0.2006
Gy_offset = -0.08
Gz_offset = -0.15

# Complementary filter constants
alpha = 0.98
dt = 0.01  # Sampling time

# Kalman filter parameters
Q_angle = 0.001
Q_gyro = 0.003
R_angle = 0.03

def MPU_Init():
    # Initialize MPU6050
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    bus.write_byte_data(Device_Address, LOWPASSFILTER, 5)

def read_raw_data(addr):
    # Read raw accelerometer and gyroscope data
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

def get_pitch_roll():
    global P, K
    
    # Read accelerometer and gyroscope data
    acc_x = (read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE_FACTOR) - Axoffset
    acc_y = (read_raw_data(ACCEL_YOUT_H) / ACCEL_SCALE_FACTOR) - Ayoffset
    acc_z = (read_raw_data(ACCEL_ZOUT_H) / ACCEL_SCALE_FACTOR) - Azoffset
    gyro_x = (read_raw_data(GYRO_XOUT_H) / GYRO_SCALE_FACTOR) - Gx_offset
    gyro_y = (read_raw_data(GYRO_YOUT_H) / GYRO_SCALE_FACTOR) - Gy_offset
    gyro_z = (read_raw_data(GYRO_ZOUT_H) / GYRO_SCALE_FACTOR) - Gz_offset
    
    # Calculate pitch and roll angles from accelerometer
    pitch = math.degrees(math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2)))
    roll = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2)))
    
    # Kalman filter
    # Prediction step
    angle = pitch
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_gyro * dt
    
    # Update step
    y = angle - angle
    S = P[0][0] + R_angle
    K[0] = P[0][0] / S
    K[1] = P[1][0] / S
    
    # Correction step
    angle += K[0] * y
    gyro_x_rate = gyro_x - angle
    angle += K[1] * gyro_x_rate
    P00_temp = P[0][0]
    P01_temp = P[0][1]
    
    P[0][0] -= K[0] * P00_temp
    P[0][1] -= K[0] * P01_temp
    P[1][0] -= K[1] * P00_temp
    P[1][1] -= K[1] * P01_temp
    
    return angle, roll

bus = smbus.SMBus(1)
MPU_Init()

# Initialize Kalman filter variables
P = np.array([[0, 0], [0, 0]], dtype=float)
K = np.array([0, 0], dtype=float)

try:
    while True:
        pitch, roll = get_pitch_roll()
        print("Pitch: {:.2f} degrees, Roll: {:.2f} degrees".format(pitch, roll))
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
