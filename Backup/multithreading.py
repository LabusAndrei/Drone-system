import threading
import smbus
import smbus2
import math
import time
import numpy as np

# Cod pentru MPU6050
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

Device_Address = 0x68
ACCEL_SCALE_FACTOR = 16384.0
GYRO_SCALE_FACTOR = 131.0
Axoffset = 0.00498
Ayoffset = 0.03055
Azoffset = 0.0029
Gx_offset = -0.19
Gy_offset = -0.09
Gz_offset = 0.15
Q_unghi_pitch = 0.001
Q_gyro_pitch = 0.003
R_unghi_pitch = 0.03
Q_unghi_roll = 0.001
Q_gyro_roll = 0.003
R_unghi_roll = 0.03
dt = 0.01

bus_mpu = smbus.SMBus(1)

def MPU_Init():
    bus_mpu.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus_mpu.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus_mpu.write_byte_data(Device_Address, CONFIG, 0)
    bus_mpu.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus_mpu.write_byte_data(Device_Address, INT_ENABLE, 1)
    bus_mpu.write_byte_data(Device_Address, LOWPASSFILTER, 5)

def read_raw_data(addr):
    high = bus_mpu.read_byte_data(Device_Address, addr)
    low = bus_mpu.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

P_pitch = np.array([[0, 0], [0, 0]], dtype=float)
K_pitch = np.array([0, 0], dtype=float)
P_roll = np.array([[0, 0], [0, 0]], dtype=float)
K_roll = np.array([0, 0], dtype=float)

def kalman_filter(unghi, P, K, Q_unghi, Q_gyro, R_unghi, rata_gyro):
    unghi += dt * rata_gyro
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_unghi)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_gyro * dt
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

def get_pitch_roll():
    global P_pitch, K_pitch, P_roll, K_roll
    acc_x = (read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE_FACTOR) - Axoffset
    acc_y = (read_raw_data(ACCEL_YOUT_H) / ACCEL_SCALE_FACTOR) - Ayoffset
    acc_z = (read_raw_data(ACCEL_ZOUT_H) / ACCEL_SCALE_FACTOR) - Azoffset
    gyro_x = (read_raw_data(GYRO_XOUT_H) / GYRO_SCALE_FACTOR) - Gx_offset
    gyro_y = (read_raw_data(GYRO_YOUT_H) / GYRO_SCALE_FACTOR) - Gy_offset
    gyro_z = (read_raw_data(GYRO_ZOUT_H) / GYRO_SCALE_FACTOR) - Gz_offset
    pitch = math.degrees(math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2)))
    roll = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2)))
    unghi_pitch = kalman_filter(pitch, P_pitch, K_pitch, Q_unghi_pitch, Q_gyro_pitch, R_unghi_pitch, gyro_x)
    unghi_roll = kalman_filter(roll, P_roll, K_roll, Q_unghi_roll, Q_gyro_roll, R_unghi_roll, gyro_y)
    return unghi_pitch, unghi_roll, acc_x, acc_y, acc_z

def vertical_velocity(Pitch, Roll, Ax, Ay, Az, ZVelocity, Time):
    AccZInertial = -Ax * math.sin(Pitch*math.pi/180) + Ay * math.cos(Pitch*math.pi/180) * math.sin(Roll*math.pi/180) + Az * math.cos(Pitch*math.pi/180) * math.cos(Roll*math.pi/180)
    AccZInertial = (AccZInertial - 1) * 9.81 * 10
    ZVelocity = ZVelocity + AccZInertial * Time
    return ZVelocity

# Cod pentru BMP280
BMP280_I2C_ADDR = 0x76
BMP280_REG_DIG_T1 = 0x88
BMP280_REG_DIG_T2 = 0x8A
BMP280_REG_DIG_T3 = 0x8C
BMP280_REG_DIG_P1 = 0x8E
BMP280_REG_DIG_P2 = 0x90
BMP280_REG_DIG_P3 = 0x92
BMP280_REG_DIG_P4 = 0x94
BMP280_REG_DIG_P5 = 0x96
BMP280_REG_DIG_P6 = 0x98
BMP280_REG_DIG_P7 = 0x9A
BMP280_REG_DIG_P8 = 0x9C
BMP280_REG_DIG_P9 = 0x9E
BMP280_REG_CHIPID = 0xD0
BMP280_REG_VERSION = 0xD1
BMP280_REG_SOFTRESET = 0xE0
BMP280_REG_CONTROL = 0xF4
BMP280_REG_CONFIG = 0xF5
BMP280_REG_PRESSUREDATA = 0xF7
BMP280_REG_TEMPDATA = 0xFA
BMP280_CHIPID = 0x58

bus_bmp = smbus2.SMBus(1)

def read_unsigned_short(reg):
    data = bus_bmp.read_i2c_block_data(BMP280_I2C_ADDR, reg, 2)
    return data[0] + (data[1] << 8)

def read_signed_short(reg):
    data = bus_bmp.read_i2c_block_data(BMP280_I2C_ADDR, reg, 2)
    result = data[0] + (data[1] << 8)
    if result > 32767:
        result -= 65536
    return result

def read_calibration_data():
    cal = {}
    cal['dig_T1'] = read_unsigned_short(BMP280_REG_DIG_T1)
    cal['dig_T2'] = read_signed_short(BMP280_REG_DIG_T2)
    cal['dig_T3'] = read_signed_short(BMP280_REG_DIG_T3)
    cal['dig_P1'] = read_unsigned_short(BMP280_REG_DIG_P1)
    cal['dig_P2'] = read_signed_short(BMP280_REG_DIG_P2)
    cal['dig_P3'] = read_signed_short(BMP280_REG_DIG_P3)
    cal['dig_P4'] = read_signed_short(BMP280_REG_DIG_P4)
    cal['dig_P5'] = read_signed_short(BMP280_REG_DIG_P5)
    cal['dig_P6'] = read_signed_short(BMP280_REG_DIG_P6)
    cal['dig_P7'] = read_signed_short(BMP280_REG_DIG_P7)
    cal['dig_P8'] = read_signed_short(BMP280_REG_DIG_P8)
    cal['dig_P9'] = read_signed_short(BMP280_REG_DIG_P9)
    return cal

def write_register(reg, value):
    bus_bmp.write_byte_data(BMP280_I2C_ADDR, reg, value)

def configure_sensor():
    write_register(BMP280_REG_SOFTRESET, 0xB6)
    time.sleep(0.2)
    chip_id = bus_bmp.read_byte_data(BMP280_I2C_ADDR, BMP280_REG_CHIPID)
    if chip_id != BMP280_CHIPID:
        raise RuntimeError('BMP280 chip ID mismatch')
    calibration_data = read_calibration_data()
    write_register(BMP280_REG_CONTROL, 0x27)
    write_register(BMP280_REG_CONFIG, 0x14)
    return calibration_data

def read_raw_data_bmp():
    data = bus_bmp.read_i2c_block_data(BMP280_I2C_ADDR, BMP280_REG_PRESSUREDATA, 6)
    raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    raw_press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    return raw_temp, raw_press

def compensate_temperature(raw_temp, cal):
    var1 = (raw_temp / 16384.0 - cal['dig_T1'] / 1024.0) * cal['dig_T2']
    var2 = ((raw_temp / 131072.0 - cal['dig_T1'] / 8192.0) ** 2) * cal['dig_T3']
    t_fine = var1 + var2
    temp = var1 + var2
    return t_fine, temp / 5120.0

def compensate_pressure(raw_press, t_fine, cal):
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * cal['dig_P6'] / 32768.0
    var2 = var2 + var1 * cal['dig_P5'] * 2.0
    var2 = (var2 / 4.0) + (cal['dig_P4'] * 65536.0)
    var1 = (cal['dig_P3'] * var1 * var1 / 524288.0 + cal['dig_P2'] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * cal['dig_P1']
    if var1 == 0:
        return 0
    pressure = 1048576.0 - raw_press
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
    var1 = cal['dig_P9'] * pressure * pressure / 2147483648.0
    var2 = pressure * cal['dig_P8'] / 32768.0
    pressure = pressure + (var1 + var2 + cal['dig_P7']) / 16.0
    return pressure / 100

def calculate_altitude(pressure, sea_level_pressure=1013.25):
    return 44330.0 * (1.0 - (pressure / sea_level_pressure) ** 0.1903)

def read_sensor_data(calibration_data):
    raw_temp, raw_press = read_raw_data_bmp()
    t_fine, temperature = compensate_temperature(raw_temp, calibration_data)
    pressure = compensate_pressure(raw_press, t_fine, calibration_data)
    altitude = calculate_altitude(pressure)
    return temperature, pressure, altitude

# Implementare filtru Kalman pentru altitudine
class KalmanFilter:
    def __init__(self, Q=0.1, R=0.1):
        self.Q = Q
        self.R = R
        self.x = 0.0
        self.P = 1.0

    def update(self, measurement):
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P + self.Q
        return self.x

kalman_altitude_filter = KalmanFilter()

def run_mpu():
    global pitch, roll, ZVelocity, LastTime
    MPU_Init()
    LastTime = time.time()
    ZVelocity = 0
    while True:
        pitch, roll, acc_x, acc_y, acc_z = get_pitch_roll()
        dateDateTimeNow = time.time()
        Time = dateDateTimeNow - LastTime
        ZVelocity = vertical_velocity(pitch, roll, acc_x, acc_y, acc_z, ZVelocity, Time)
        LastTime = dateDateTimeNow
        time.sleep(0.01)

def run_bmp(calibration_data):
    global altitude, kalman_altitude
    while True:
        _, _, altitude = read_sensor_data(calibration_data)
        kalman_altitude = kalman_altitude_filter.update(altitude)
        time.sleep(1)

def main():
    global pitch, roll, ZVelocity, altitude, kalman_altitude
    calibration_data = configure_sensor()

    # Inițializăm variabilele globale
    pitch, roll, ZVelocity = 0, 0, 0
    altitude, kalman_altitude = 0, 0

    mpu_thread = threading.Thread(target=run_mpu)
    bmp_thread = threading.Thread(target=run_bmp, args=(calibration_data,))

    mpu_thread.start()
    bmp_thread.start()

    try:
        while True:
            print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Vertical Speed: {ZVelocity:.2f} cm/s, Altitude: {altitude:.2f} m, Kalman Altitude: {kalman_altitude:.2f} m")
            time.sleep(0)
    except KeyboardInterrupt:
        pass

    mpu_thread.join()
    bmp_thread.join()

if __name__ == "__main__":
    main()
