from gpiozero import Servo
from time import sleep

correction = 0
maxPW = (2.0+correction)/1000
minPW = (1.0+correction)/1000
servo = Servo(14, min_pulse_width = minPW, max_pulse_width = maxPW)
#servo = AngularServo(14, min_angle = 0, max_angle = 90)
#servo.angle  = 0.0

while True:
    try:
        #servo.mid()
       #print("mid")
        #sleep(1)
        servo.value = -1
        servo.min() 
        print("min")
        #servo.value = None
        sleep(1)
        servo.value = 1
        #servo.max()
        #servo.value = None
        print("max")
        sleep(1)
        
    except KeyboardInterrupt:
         print("bye")

    