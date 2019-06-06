import serial
class sucker():
    def __init__(self):
        serialPort = "/dev/ttyUSB0"  
        baudRate = 9600  
        self.ser = serial.Serial(serialPort, baudRate, timeout=0.5)
    def suck(self):
        self.ser.write(b"0")
    def loose(self):
        self.ser.write(b"1")