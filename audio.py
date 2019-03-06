import serial
import threading 
from playsound import playsound

arduino = serial.Serial('COM6', 9600, timeout=0.1)

while True:
    data = arduino.readline()
    if data:
        line = data.decode('utf-8')
        print(line)
        if line.startswith('Calibrate glove'):
            key = input('')
            key_byte = key.encode('utf-8')
            print('Recieved key ')
            print(key)
            arduino.write(key_byte)
        if line.startswith('Final result') and line[-3].isupper():
            letter = line[-3]
            playsound('alphabet/%s.wav' % str(letter))
