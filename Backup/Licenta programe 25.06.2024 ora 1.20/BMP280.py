import smbus2
import time
import math

class BMP280:
    def __init__(self, bus_number=1, device_address=0x76):
        self.bus = smbus2.SMBus(bus_number)
        self.device_address = device_address
        
        # BMP280 Registers
        self.BMP280_REG_DIG_T1 = 0x88
        self.BMP280_REG_DIG_T2 = 0x8A
        self.BMP280_REG_DIG_T3 = 0x8C
        self.BMP280_REG_DIG_P1 = 0x8E
        self.BMP280_REG_DIG_P2 = 0x90
        self.BMP280_REG_DIG_P3 = 0x92
        self.BMP280_REG_DIG_P4 = 0x94
        self.BMP280_REG_DIG_P5 = 0x96
        self.BMP280_REG_DIG_P6 = 0x98
        self.BMP280_REG_DIG_P7 = 0x9A
        self.BMP280_REG_DIG_P8 = 0x9C
        self.BMP280_REG_DIG_P9 = 0x9E
        self.BMP280_REG_CHIPID = 0xD0
        self.BMP280_REG_VERSION = 0xD1
        self.BMP280_REG_SOFTRESET = 0xE0
        self.BMP280_REG_CONTROL = 0xF4
        self.BMP280_REG_CONFIG = 0xF5
        self.BMP280_REG_PRESSUREDATA = 0xF7
        self.BMP280_REG_TEMPDATA = 0xFA

        # Sensor constants
        self.BMP280_CHIPID = 0x58

        self.calibration_data = self.configure_sensor()

    def read_unsigned_short(self, reg):
        """Read an unsigned short from the specified register."""
        data = self.bus.read_i2c_block_data(self.device_address, reg, 2)
        return data[0] + (data[1] << 8)

    def read_signed_short(self, reg):
        """Read a signed short from the specified register."""
        data = self.bus.read_i2c_block_data(self.device_address, reg, 2)
        result = data[0] + (data[1] << 8)
        if result > 32767:
            result -= 65536
        return result

    def read_calibration_data(self):
        """Read calibration data from the BMP280 sensor."""
        cal = {}
        cal['dig_T1'] = self.read_unsigned_short(self.BMP280_REG_DIG_T1)
        cal['dig_T2'] = self.read_signed_short(self.BMP280_REG_DIG_T2)
        cal['dig_T3'] = self.read_signed_short(self.BMP280_REG_DIG_T3)
        cal['dig_P1'] = self.read_unsigned_short(self.BMP280_REG_DIG_P1)
        cal['dig_P2'] = self.read_signed_short(self.BMP280_REG_DIG_P2)
        cal['dig_P3'] = self.read_signed_short(self.BMP280_REG_DIG_P3)
        cal['dig_P4'] = self.read_signed_short(self.BMP280_REG_DIG_P4)
        cal['dig_P5'] = self.read_signed_short(self.BMP280_REG_DIG_P5)
        cal['dig_P6'] = self.read_signed_short(self.BMP280_REG_DIG_P6)
        cal['dig_P7'] = self.read_signed_short(self.BMP280_REG_DIG_P7)
        cal['dig_P8'] = self.read_signed_short(self.BMP280_REG_DIG_P8)
        cal['dig_P9'] = self.read_signed_short(self.BMP280_REG_DIG_P9)
        return cal

    def write_register(self, reg, value):
        """Write a byte to the specified register."""
        self.bus.write_byte_data(self.device_address, reg, value)

    def configure_sensor(self):
        """Configure the BMP280 sensor with the filter enabled."""
        # Reset the sensor
        self.write_register(self.BMP280_REG_SOFTRESET, 0xB6)
        time.sleep(0.2)

        # Read and verify chip ID
        chip_id = self.bus.read_byte_data(self.device_address, self.BMP280_REG_CHIPID)
        if chip_id != self.BMP280_CHIPID:
            raise RuntimeError('BMP280 chip ID mismatch')

        # Read calibration data
        calibration_data = self.read_calibration_data()

        # Configure control register: normal mode, oversampling x1 (temperature and pressure)
        self.write_register(self.BMP280_REG_CONTROL, 0x27)

        # Configure config register: standby time 0.5ms, filter coefficient 16 (0x14)
        self.write_register(self.BMP280_REG_CONFIG, 0x14)

        return calibration_data

    def read_raw_data(self):
        """Read raw temperature and pressure data from the BMP280 sensor."""
        data = self.bus.read_i2c_block_data(self.device_address, self.BMP280_REG_PRESSUREDATA, 6)
        raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        raw_press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return raw_temp, raw_press

    def compensate_temperature(self, raw_temp):
        """Compensate the raw temperature using the calibration data."""
        cal = self.calibration_data
        var1 = (raw_temp / 16384.0 - cal['dig_T1'] / 1024.0) * cal['dig_T2']
        var2 = ((raw_temp / 131072.0 - cal['dig_T1'] / 8192.0) ** 2) * cal['dig_T3']
        t_fine = var1 + var2
        temp = var1 + var2
        return t_fine, temp / 5120.0

    def compensate_pressure(self, raw_press, t_fine):
        """Compensate the raw pressure using the calibration data."""
        cal = self.calibration_data
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * cal['dig_P6'] / 32768.0
        var2 = var2 + var1 * cal['dig_P5'] * 2.0
        var2 = (var2 / 4.0) + (cal['dig_P4'] * 65536.0)
        var1 = (cal['dig_P3'] * var1 * var1 / 524288.0 + cal['dig_P2'] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * cal['dig_P1']
        if var1 == 0:
            return 0  # avoid division by zero
        pressure = 1048576.0 - raw_press
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
        var1 = cal['dig_P9'] * pressure * pressure / 2147483648.0
        var2 = pressure * cal['dig_P8'] / 32768.0
        pressure = pressure + (var1 + var2 + cal['dig_P7']) / 16.0
        return pressure / 100

    def calculate_altitude(self, pressure, sea_level_pressure=1013.25):
        """Calculate altitude using atmospheric pressure."""
        return 44330.0 * (1.0 - (pressure / sea_level_pressure) ** 0.1903)

    def read_sensor_data(self):
        """Read and compensate the temperature, pressure, and altitude data."""
        raw_temp, raw_press = self.read_raw_data()
        t_fine, temperature = self.compensate_temperature(raw_temp)
        pressure = self.compensate_pressure(raw_press, t_fine)
        altitude = self.calculate_altitude(pressure)
        return temperature, pressure, altitude

if __name__ == "__main__":
    bmp280 = BMP280()
    try:
        while True:
            temperature, pressure, altitude = bmp280.read_sensor_data()
            print(f"Temperature: {temperature:.2f} °C, Pressure: {pressure:.2f} hPa, Altitude: {altitude:.2f} m")
            time.sleep(0)
    except KeyboardInterrupt:
        print("Iesire...")
