import serial
from pynput import keyboard
from playsound import playsound

arduino = serial.Serial('COM6', 9600, timeout=0.1)
sound = False

def on_press(key):
    try:
    #    print('alphanumeric key {0} pressed'.format(key.char))
        print('SENDING COMMAND {0}'.format(key.char))
        key_byte = key.char.encode('utf-8')
        arduino.write(key_byte)
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
    
# Collect events until released
with keyboard.Listener(
        on_press=on_press) as listener:
    while True:
        data = arduino.readline()
        if data:
            line = data.decode('utf-8')
            print(line[:-2])
            if line.startswith('Calibrate glove'):
                key = input('')
                key_byte = key.encode('utf-8')
                print('Recieved key ')
                print(key)
                arduino.write(key_byte)
            if line.startswith('Sound is'):
                if 'OFF' in line: sound = False
                else: sound = True
            # Index at -3 to get rid of newline character
            if 'rating' in line and line[-3].isupper() and sound:
                letter = line[-3]
                playsound('alphabet/%s.wav' % str(letter))
