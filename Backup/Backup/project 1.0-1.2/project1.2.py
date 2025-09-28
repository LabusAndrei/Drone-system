import smbus					#import SMBus module of I2C
import time						#import
import math
from bmp280 import BMP280

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

def AltitudeCalibration():
    s = 0.1
    baseline_values = []
    iterations = 100

    print("SYSTEM NOTIFICATION", s*iterations, "seconds calibration")

    for i in range(iterations):
        pressure = bmp280.get_pressure()
        baseline_values.append(pressure)
        time.sleep(s)

    baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])
    return baseline

def Calibration():
	
	s = 0.1
	iterations = 100
	
	print ("SYSTEM NOTIFICATION", s*iterations, "seconds calibration")
	
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
	
	print (Axoffset, Ayoffset, Azoffset, Gxoffset, Gyoffset, Gzoffset)

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
bmp280 = BMP280(i2c_dev=bus)

def MPU_read():
    
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
	return (Ax,Ay,Az,Gx,Gy,Gz);

def Dist(a,b):
    return math.sqrt((a*a)+(b*b));

def Get_y_rotation(x,y,z):
    radians = math.atan2(y, Dist(x,z))
    return math.degrees(radians);

def Get_x_rotation(x,y,z):
    radians = math.atan2(x, Dist(y,z))
    return -math.degrees(radians);

def Run():
	MPU_Init()
	baseline = AltitudeCalibration()
	
	K = 0.98
	K1 = 1 - K
	Pitch = 0
	Roll  = 0
	VelocityVertical = 0
	Atime = 1
	
	LastTime = time.time()
	
	while True:
		[Ax,Ay,Az,Gx,Gy,Gz] = MPU_read()
		dateDateTimeNow = time.time()
		
		APitch = Get_x_rotation( Ax, Ay, Az )
		ARoll = Get_y_rotation( Ax, Ay, Az )
		
		Time = dateDateTimeNow - LastTime
		GPitch = Pitch + Time * Gy 
		GRoll  = Roll  + Time * Gx 
		
		Pitch  = K * GPitch + K1 * APitch 
		Roll = K * GRoll  + K1 * ARoll  
		
		Atime = Atime + Time
		
		if (0.45 < Atime):
			Altitude = bmp280.get_altitude(qnh=baseline)*100
			Atime = 0
			
		AccZInertial = -Ax*math.sin(Pitch) + Ay*math.cos(Pitch)*math.sin(Roll) + Az*math.cos(Pitch)*math.cos(Roll)
		AccZInertial = AccZInertial*9.81*10
		VelocityVertical=VelocityVertical+AccZInertial*Time
		
		# Ce ramane de facut:
		# cap 19 video + pag 162 carte ramane de implementat filtrul kalman bidimensional (2D)
		# data de 2/4/2024 am pus pauza ora 10.08
		
		
		#print ("Time:", Time, "\tGx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
		#print ("Time:", Time, "\tARoll =", ARoll, "\tAPitch =", APitch, "\tGRoll =", GRoll, "\tGPitch =", GPitch)
		#print ("Time:", Time, "Roll =", Roll, "\tPitch =", Pitch)
		#print ("Time:", Time, "\tVertical Velocity =", VelocityVertical, "\tAltitude =", Altitude)
		LastTime = dateDateTimeNow
		time.sleep(0)

Axoffset = 0.031
Ayoffset = 0.021
Azoffset = 0.003
Gxoffset = -0.23
Gyoffset = -0.0764
Gzoffset = -0.149

#Calibration()

Run()
