import smbus
import math
import time
import numpy as np

# Înregistrări MPU6050
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

# Adresa MPU6050
Device_Address = 0x68

# Factori de scalare pentru accelerometru și giroscop
ACCEL_SCALE_FACTOR = 16384.0  # gama +/- 2g
GYRO_SCALE_FACTOR = 131.0  # gama +/- 250 grade/s

# Offset-uri de calibrare
Axoffset = 0.00498
Ayoffset = 0.03055
Azoffset = 0.0029
Gx_offset = -0.19
Gy_offset = -0.09
Gz_offset = 0.15

# Parametrii filtrului Kalman pentru pitch
Q_unghi_pitch = 0.001
Q_gyro_pitch = 0.003
R_unghi_pitch = 0.03

# Parametrii filtrului Kalman pentru roll
Q_unghi_roll = 0.001
Q_gyro_roll = 0.003
R_unghi_roll = 0.03

# Timp de eșantionare
dt = 0.01

def MPU_Init():
    # Inițializare MPU6050
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    bus.write_byte_data(Device_Address, LOWPASSFILTER, 5)

def read_raw_data(addr):
    # Citește date brute de la accelerometru și giroscop
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

def kalman_filter(unghi, P, K, Q_unghi, Q_gyro, R_unghi, rata_gyro):
    # Pasul de predicție
    unghi += dt * rata_gyro
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_unghi)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_gyro * dt
    
    # Pasul de actualizare
    y = unghi - unghi
    S = P[0][0] + R_unghi
    K[0] = P[0][0] / S
    K[1] = P[1][0] / S
    
    # Pasul de corecție
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
    
    # Citește datele de la accelerometru și giroscop
    acc_x = (read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE_FACTOR) - Axoffset
    acc_y = (read_raw_data(ACCEL_YOUT_H) / ACCEL_SCALE_FACTOR) - Ayoffset
    acc_z = (read_raw_data(ACCEL_ZOUT_H) / ACCEL_SCALE_FACTOR) - Azoffset
    gyro_x = (read_raw_data(GYRO_XOUT_H) / GYRO_SCALE_FACTOR) - Gx_offset
    gyro_y = (read_raw_data(GYRO_YOUT_H) / GYRO_SCALE_FACTOR) - Gy_offset
    gyro_z = (read_raw_data(GYRO_ZOUT_H) / GYRO_SCALE_FACTOR) - Gz_offset
    
    # Calculează unghiurile pitch și roll din datele accelerometrului
    pitch = math.degrees(math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2)))
    roll = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2)))
    
    # Aplică filtrul Kalman pentru pitch
    unghi_pitch = kalman_filter(pitch, P_pitch, K_pitch, Q_unghi_pitch, Q_gyro_pitch, R_unghi_pitch, gyro_x)
    
    # Aplică filtrul Kalman pentru roll
    unghi_roll = kalman_filter(roll, P_roll, K_roll, Q_unghi_roll, Q_gyro_roll, R_unghi_roll, gyro_y)
    
    return unghi_pitch, unghi_roll, acc_x, acc_y, acc_z

bus = smbus.SMBus(1)
MPU_Init()

def vertical_velocity(Pitch, Roll, Ax, Ay, Az, ZVelocity, Time):
    AccZInertial = -Ax * math.sin(Pitch*math.pi/180) + Ay * math.cos(Pitch*math.pi/180) * math.sin(Roll*math.pi/180) + Az * math.cos(Pitch*math.pi/180) * math.cos(Roll*math.pi/180)
    AccZInertial = (AccZInertial - 1) * 9.81 * 10
    ZVelocity = ZVelocity + AccZInertial * Time
    return ZVelocity

# Inițializează variabilele filtrului Kalman pentru pitch și roll
P_pitch = np.array([[0, 0], [0, 0]], dtype=float)
K_pitch = np.array([0, 0], dtype=float)
P_roll = np.array([[0, 0], [0, 0]], dtype=float)
K_roll = np.array([0, 0], dtype=float)

LastTime = time.time()
ZVelocity = 0
try:
    while True:
        pitch, roll, acc_x, acc_y, acc_z = get_pitch_roll()
        dateDateTimeNow = time.time()
        Time = dateDateTimeNow - LastTime
        ZVelocity = vertical_velocity(pitch, roll, acc_x, acc_y, acc_z, ZVelocity, Time)
        #print("Time: {:.2f} s, \t Pitch: {:.2f} degrees, \t Roll: {:.2f} degrees".format(Time,pitch, roll))
        print ("Time: ", Time, "Zvelocity: ", ZVelocity)
        LastTime = dateDateTimeNow
        time.sleep(0)
except KeyboardInterrupt:
    print("Iesire...")
