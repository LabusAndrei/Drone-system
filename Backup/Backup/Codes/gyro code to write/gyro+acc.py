import smbus					#import SMBus module of I2C
import time						#import
import numpy as np
import math
import time

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

Ax =0
Ay =0
Az =0
Gx =0
Gy =0
Gz =0

Axoffset = 0.065
Ayoffset = 0.015
Azoffset = 0
Gxoffset = -0.2006
Gyoffset = -0.08
Gzoffset = -0.15


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
	#Accelero and Gyro value are 16-bit
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
	
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
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
	
	K = 0.98
	K1 = 1 - K
	
	MPU_Init()
	MPU_read()
	LastTime = time.time()
	Pitch = 0
	Roll  = 0

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
		
		## de verificat si reparat citirile giroscopului
		
		print ("Time:", Time, "\tGx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
		#print ("Time:", Time, "\tARoll =", ARoll, "\tAPitch =", APitch, "\tGRoll =", GRoll, "\tGPitch =")
		#print ("Roll =", Roll, "\tPitch =", Pitch)
		LastTime = dateDateTimeNow
		time.sleep(0)
		


Run()
