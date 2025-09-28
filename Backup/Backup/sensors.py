import smbus					#import SMBus module of I2C
import time
import math
from bmp280 import BMP280

# HMC5883L register addresses
ADDRESS = 0x1E
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
X_MSB = 0x03
Z_MSB = 0x05
Y_MSB = 0x07

#some MPU6050 Registers and their Address
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
LOWPASSFILTER= 26

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

Ax =0
Ay =0
Az =0
Gx =0
Gy =0
Gz =0

Axoffset = 0
Ayoffset = 0
Azoffset = 0
Gxoffset = 0
Gyoffset = 0
Gzoffset = 0

Preasure = 0

def calibration():
	
	s = 0.1
	iterations = 50
	
	print ("SYSTEM NOTIFICATION", s*iterations*2, "seconds calibration")
	
	Axoffset = 0
	Ayoffset = 0
	Azoffset = 0
	Gxoffset = 0
	Gyoffset = 0
	Gzoffset = 0

	i=1
	while (i <= iterations):
		#Read Accelerometer raw value
		Axoffset += read_raw_data(ACCEL_XOUT_H)/16384.0
		Ayoffset += read_raw_data(ACCEL_YOUT_H)/16384.0
		Azoffset += read_raw_data(ACCEL_ZOUT_H)/16384.0
		
		#Read Gyroscope raw value
		Gxoffset += read_raw_data(GYRO_XOUT_H)/131.0
		Gyoffset += read_raw_data(GYRO_YOUT_H)/131.0
		Gzoffset += read_raw_data(GYRO_ZOUT_H)/131.0
		
		i += 1
		
		time.sleep(s)
		
	Axoffset = round((Axoffset/iterations), 3)
	Ayoffset = round((Ayoffset/iterations), 3)
	Azoffset = round((Azoffset/iterations), 3)
	Gxoffset = round((Gxoffset/iterations), 3)
	Gyoffset = round((Gyoffset/iterations), 3)
	Gzoffset = round((Gzoffset/iterations), 3)
	
	baseline_values = []
	
	for i in range(iterations):
		pressure = bmp280.get_pressure()
		baseline_values.append(pressure)
		time.sleep(s)
	
	Preasure = sum(baseline_values[:-25]) / len(baseline_values[:-25])
	
	return (Axoffset, Ayoffset, Azoffset, Gxoffset, Gyoffset, Gzoffset, Preasure)
	
	
	

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
	
	#Low pass filter
	bus.write_byte_data(Device_Address, LOWPASSFILTER, 5)

def read_raw_data(addr):
	#Accelero, Gyro and Magnetometer value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
bmp280 = BMP280(i2c_dev=bus)

def MPU_read():
    
	global Ax
	global Ay
	global Az
	global Gx
	global Gy
	global Gz
    
	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	Ax = acc_x/16384.0 - Axoffset
	Ay = acc_y/16384.0 - Ayoffset
	Az = acc_z/16384.0 - Azoffset
	
	Gx = (gyro_x/131.0 - Gxoffset)
	Gy = (gyro_y/131.0 - Gyoffset)
	Gz = (gyro_z/131.0 - Gzoffset)
	return;

def Dist(a,b):
    return math.sqrt((a*a)+(b*b));

def Get_y_rotation(x,y,z):
    radians = math.atan2(y, Dist(x,z))
    return math.degrees(radians);

def Get_x_rotation(x,y,z):
    radians = math.atan2(x, Dist(y,z))
    return -math.degrees(radians);

def Run():
	
	global Ax
	global Ay
	global Az
	global Gx
	global Gy
	global Gz
	
	[Axoffset, Ayoffset, Azoffset, Gxoffset, Gyoffset, Gzoffset, Preasure] = calibration()
	
	K = 0.98
	K1 = 1 - K
	
	MPU_Init()
	MPU_read()
	LastTime = time.time()
	Pitch = 0
	Roll  = 0
	AccAltitude = 0

	while True:
		MPU_read()
		dateDateTimeNow = time.time()
		
		APitch = Get_x_rotation( Ax, Ay, Az )
		ARoll = Get_y_rotation( Ax, Ay, Az )
		
		Time = dateDateTimeNow - LastTime
		GPitch = Pitch + Time * Gy 
		GRoll  = Roll  + Time * Gx 
		
		Pitch  = K * GPitch + K1 * APitch 
		Roll = K * GRoll  + K1 * ARoll  
		
		
		Altitude = bmp280.get_altitude(qnh=Preasure)
		
		Avelocity = -Ax*math.sin(Pitch) + Ay*math.sin(Roll)*math.cos(Pitch) + Az*math.cos(Roll)*math.cos(Pitch)
		Avelocity = Avelocity * 9.81 * 100
		AccAltitude = AccAltitude + Time * Avelocity
		
		#trebuie sa ved care e treaba aici ca nu am putut sa lucrez de Claudia
		
		
		print ("Time:", Time, "\tGx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
		#print ("Time:", Time, "\tARoll =", ARoll, "\tAPitch =", APitch, "\tGRoll =", GRoll, "\tGPitch =")
		#print ("Time:", Time, "Roll =", Roll, "\tPitch =", Pitch, "\tAltitude =", Altitude, "metres")
		LastTime = dateDateTimeNow
		time.sleep(0.0945-time.time()+LastTime)
		

Run()
