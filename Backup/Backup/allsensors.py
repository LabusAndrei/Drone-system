import smbus	#importa SMBus module pentru I2C
import time
import math
from bmp280 import BMP280
import serial
import datetime
import numpy as np

#MPU6050 registii si adrese
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#HMC5883L registii si adrese
ADDRESS = 0x1E
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
X_MSB = 0x03
Z_MSB = 0x05
Y_MSB = 0x07

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_dataMPU(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
        #concatenate higher and lower value
        value = ((high << 8) | low)
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
        
def setupHMC():
    bus.write_byte_data(ADDRESS, CONFIG_A, 0x70)  # Set to 8 samples @ 15Hz
    bus.write_byte_data(ADDRESS, CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
    bus.write_byte_data(ADDRESS, MODE, 0x00)  # Continuous measurement mode
 
def read_raw_dataHMC(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(ADDRESS, addr)
    low = bus.read_byte_data(ADDRESS, addr+1)
    # Combine them to get a 16-bit value
    value = (high << 8) + low
    if value > 32768:  # Adjust for 2's complement
        value = value - 65536
    return value
 
def compute_headingHMC(x, y):
    # Calculate heading in radians
    heading_rad = math.atan2(y, x)
    # Adjust for declination angle (e.g. 0.22 for ~13 degrees)
    declination_angle = 0.22
    heading_rad += declination_angle
    # Correct for when signs are reversed.
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    # Check for wrap due to addition of declination.
    if heading_rad > 2 * math.pi:
        heading_rad -= 2 * math.pi
    # Convert radians to degrees for readability.
    heading_deg = heading_rad * (180.0 / math.pi)
    return heading_deg
    
class KalmanFilter:
    def __init__(self, initial_state_mean, initial_state_covariance, transition_matrix, observation_matrix,
                 process_noise_covariance, observation_noise_covariance):
        self.state_mean = initial_state_mean
        self.state_covariance = initial_state_covariance
        self.transition_matrix = transition_matrix
        self.observation_matrix = observation_matrix
        self.process_noise_covariance = process_noise_covariance
        self.observation_noise_covariance = observation_noise_covariance

    def predict(self):
        self.state_mean = np.dot(self.transition_matrix, self.state_mean)
        self.state_covariance = np.dot(np.dot(self.transition_matrix, self.state_covariance), self.transition_matrix.T) + self.process_noise_covariance

    def update(self, observation):
        observation_residual = observation - np.dot(self.observation_matrix, self.state_mean)
        observation_covariance = np.dot(np.dot(self.observation_matrix, self.state_covariance), self.observation_matrix.T) + self.observation_noise_covariance
        kalman_gain = np.dot(np.dot(self.state_covariance, self.observation_matrix.T), np.linalg.inv(observation_covariance))
        self.state_mean = self.state_mean + np.dot(kalman_gain, observation_residual)
        self.state_covariance = self.state_covariance - np.dot(np.dot(kalman_gain, self.observation_matrix), self.state_covariance)

# Define the initial state of the system
initial_state_mean = np.zeros(10)  # Assuming 12-dimensional state
initial_state_covariance = np.eye(10)    # Assuming identity matrix as initial covariance

# Define the transition matrix (state transition model)
transition_matrix = np.eye(10)  # Assuming no change in state over time

# Define the observation matrix (observation model)
observation_matrix = np.eye(10)  # Assuming direct observation of state variables

# Define the process noise covariance (how much the system is expected to deviate from the model)
process_noise_covariance = np.eye(10) * 0.1  # Assuming small process noise

# Define the observation noise covariance (measurement noise)
observation_noise_covariance = np.eye(10) * 0.1  # Assuming small observation noise

# Create a Kalman filter instance
kf = KalmanFilter(initial_state_mean, initial_state_covariance, transition_matrix, observation_matrix,
                  process_noise_covariance, observation_noise_covariance)

baseline_values = []
baseline_size = 100
secunde = 100

print("Collecting baseline values for ", baseline_size/secunde," seconds. Do not move the sensor!\n".format(baseline_size))

for i in range(baseline_size):
    pressure = bmp280.get_pressure()
    baseline_values.append(pressure)
    time.sleep(1/secunde)

baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
MPU_Init()
setupHMC()

DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Conectat pe " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)

while True:
	print ("--------------------------------------------------------")
	
	try:
		line=DWM.readline()
		if(line):
			if len(line)>=140:
				parse=line.decode().split(",")
				x_pos=parse[parse.index("POS")+1]
				y_pos=parse[parse.index("POS")+2]
				z_pos=parse[parse.index("POS")+3]
				val = (x_pos,y_pos,z_pos)
				print("Time:", datetime.datetime.now().strftime("%H:%M:%S"),"X:",x_pos,",Y:",y_pos,",Z:",z_pos)

				#Read Accelerometer raw value
				acc_x = read_raw_dataMPU(ACCEL_XOUT_H)
				acc_y = read_raw_dataMPU(ACCEL_YOUT_H)
				acc_z = read_raw_dataMPU(ACCEL_ZOUT_H)
				#Read Gyroscope raw value
				gyro_x = read_raw_dataMPU(GYRO_XOUT_H)
				gyro_y = read_raw_dataMPU(GYRO_YOUT_H)
				gyro_z = read_raw_dataMPU(GYRO_ZOUT_H)
				#Full scale range +/- 250 degree/C as per sensitivity scale factor
				Ax = acc_x/16384.0
				Ay = acc_y/16384.0
				Az = acc_z/16384.0
				Gx = gyro_x/131.0
				Gy = gyro_y/131.0
				Gz = gyro_z/131.0
				#print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	

				x = read_raw_dataHMC(X_MSB)
				y = read_raw_dataHMC(Y_MSB)
				z = read_raw_dataHMC(Z_MSB)
				heading = compute_headingHMC(x, y) 
				#print(f"X: {x} uT, Y: {y} uT, Z: {z} uT, Heading: {heading:.2f}°")
				
				altitude = bmp280.get_altitude(qnh=baseline)
				#print('Relative altitude: {:05.2f} metres'.format(altitude))
				
				# Formulate the measurement vector
				measurement = np.array([Gx, Gy, Gz, Ax, Ay, Az, x, y, z, altitude])
				
				# Predict next state
				kf.predict()

				# Update with measurement
				kf.update(measurement)

				# Output filtered state
				print("Filtered state:")
				print("Gyroscope X (rad/s):", kf.state_mean[0])
				print("Gyroscope Y (rad/s):", kf.state_mean[1])
				print("Gyroscope Z (rad/s):", kf.state_mean[2])
				print("Accelerometer X (g):", kf.state_mean[3])
				print("Accelerometer Y (g):", kf.state_mean[4])
				print("Accelerometer Z (g):", kf.state_mean[5])
				print("Magnetometer X (uT):", kf.state_mean[6])
				print("Magnetometer Y (uT):", kf.state_mean[7])
				print("Magnetometer Z (uT):", kf.state_mean[8])
				# print("GPS X (cm)         :", kf.state_mean[9])
				# print("GPS Y (cm)         :", kf.state_mean[10])
				# print("GPS Z (cm)         :", kf.state_mean[11])
				print("Barometer Z (cm)   :", kf.state_mean[9])
				print()
			else:
				print("Pozitie necalculata")
		
	except Exception as ex:
		print(ex)
		break
DWM.write("\r".encode())
DWM.close()

	#time.sleep(1)
