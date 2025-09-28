import serial
import pigpio

class RadioController:
    def __init__(self, serial_port="/dev/serial0", baudrate=115200, host='soft', port=8888):
        self.pi = pigpio.pi(host, port)
        self.ser = serial.Serial(serial_port, baudrate)
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.Reference_altitude = 0
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = 0
        self.ch5 = 0
        self.ch6 = 0

    def read_data(self):
        frame = bytearray()
        received_data = self.ser.read()  # read serial port
        intReceived = int.from_bytes(received_data, byteorder='little')
        if intReceived == 32:
            frame.extend(received_data)  # add the header
            # read the next 31 bytes of the frame (to make a 32 byte frame size)
            nextBytes = self.ser.read(31)
            # add the read 31 bytes to the frame bytearray
            frame.extend(nextBytes)

            self.ch1, self.ch2, self.ch3, self.ch4, self.ch5, self.ch6 = self._process_frame(frame)
            self._update_reference_altitude()

    def _process_frame(self, frame):
        ch1 = int.from_bytes(frame[2:4], byteorder='little')
        ch2 = int.from_bytes(frame[4:6], byteorder='little')
        ch3 = int.from_bytes(frame[6:8], byteorder='little')
        ch4 = int.from_bytes(frame[8:10], byteorder='little')
        ch5 = int.from_bytes(frame[10:12], byteorder='little')
        ch6 = int.from_bytes(frame[12:14], byteorder='little')
        
        ch3 = (ch3 - 1000) / 10  # adjust ch3 according to your application
        
        return ch1, ch2, ch3, ch4, ch5, ch6

    def _update_reference_altitude(self):
        if self.ch3 < 10:
            self.Reference_altitude = max(0, self.Reference_altitude - 5)
        elif 10 < self.ch3 < 20:
            self.Reference_altitude = max(0, self.Reference_altitude - 0.5)
        elif 20 < self.ch3 < 40:
            self.Reference_altitude = max(0, self.Reference_altitude - 0.05)
        elif 60 < self.ch3 < 80:
            self.Reference_altitude = self.Reference_altitude + 0.05
        elif 80 < self.ch3 < 90:
            self.Reference_altitude = self.Reference_altitude + 0.5
        elif self.ch3 > 90:
            self.Reference_altitude = self.Reference_altitude + 5

    def get_reference_altitude(self):
        return self.Reference_altitude

    def get_channels_data(self):
        return self.ch1, self.ch2, self.ch3, self.ch4, self.ch5, self.ch6
