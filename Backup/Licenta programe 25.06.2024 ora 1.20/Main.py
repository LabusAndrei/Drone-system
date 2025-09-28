import threading
import time
from MPU6050 import MPU6050
from BMP280 import BMP280
from Ibus import RadioController

def process_data(pitch, roll, z_velocity, altitude):


    
    
    print(f"Processed Data - Pitch: {pitch:.2f}, Roll: {roll:.2f}, Z Velocity: {z_velocity:.2f}, Altitude: {altitude:.2f}")
    


def data_processing_thread():
    mpu = MPU6050()
    bmp = BMP280()
    radio = RadioController()
    z_velocity = 0
    LastTime = time.time()
    while True:
        dateDateTimeNow = time.time()
        Time = dateDateTimeNow - LastTime
        
        pitch, roll, acc_x, acc_y, acc_z = mpu.get_pitch_roll()
        z_velocity = mpu.vertical_velocity(pitch, roll, acc_x, acc_y, acc_z, z_velocity, mpu.dt)
        temperature, pressure, altitude = bmp.read_sensor_data()
        #process_data(pitch, roll, z_velocity, altitude)
        
        radio.read_data()
        reference_altitude = radio.get_reference_altitude()
        ch1, ch2, ch3, ch4, ch5, ch6 = radio.get_channels_data()
        

        print("Reference_altitude:", reference_altitude)
        #print("Channels: ch1={}, ch2={}, ch3={}, ch4={}, ch5={}, ch6={}".format(ch1, ch2, ch3, ch4, ch5, ch6))    
        
        #print(Time)
        
        LastTime = dateDateTimeNow
        time.sleep(0)

if __name__ == "__main__":
    thread_data_processing = threading.Thread(target=data_processing_thread)

    thread_data_processing.start()

    thread_data_processing.join()
