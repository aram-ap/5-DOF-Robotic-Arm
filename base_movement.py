import sys
import time
import RPi.GPIO as GPIO
import signal
from xbox360controller import Xbox360Controller

baseStepPin = 20
baseStepDir = 12
baseStepEn = 19

step2pin = 23
step2dir = 22
step2en = 13

timeBetweenSteps = 0.0005
isFast = False
fastMult = 5
baseIsRotating = False
baseIsMoving = False

xVal = 0.0
yVal = 0.0

def on_button_pressed(button):
    global isFast
    global timeBetweenSteps
    global fastMult
    if isFast:
        timeBetweenSteps = timeBetweenSteps*fastMult
        isFast = False
    else:
        timeBetweenSteps = timeBetweenSteps/fastMult
        isFast = True
    print('Button {0} was pressed'.format(button.name))

def on_axis_moved(axis):
    global baseIsMoving
    global baseIsRotating
    global xVal
    global yVal

    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    if axis.x >= 0.2:
        baseIsRotating = True
    elif axis.x <= -0.2:
        baseIsRotating = True
    else:
        baseIsRotating = False

    xVal = axis.x

    if axis.y >= 0.2:
        baseIsMoving = True
    elif axis.y <= -0.2:
        baseIsMoving = True
    else:
        baseIsMoving = False

    yVal = axis.y

def updatePos():
    global xVal
    global baseIsMoving
    global baseIsRotating

    if baseIsRotating:
        if xVal > 0.1:
            GPIO.output(step2dir, GPIO.HIGH)
            GPIO.output(step2pin, GPIO.HIGH)
            time.sleep(timeBetweenSteps/abs(xVal))
            GPIO.output(step2pin, GPIO.LOW)
            time.sleep(timeBetweenSteps/abs(xVal))
        elif xVal < -0.1:
            GPIO.output(step2dir, GPIO.LOW)
            GPIO.output(step2pin, GPIO.HIGH)
            time.sleep(timeBetweenSteps/abs(xVal))
            GPIO.output(step2pin, GPIO.LOW)
            time.sleep(timeBetweenSteps/abs(xVal))

    if baseIsMoving:
        if yVal > 0.1:
            GPIO.output(baseStepDir, GPIO.HIGH)
            GPIO.output(baseStepPin, GPIO.HIGH)
            time.sleep(timeBetweenSteps/abs(yVal))
            GPIO.output(baseStepPin, GPIO.LOW)
            time.sleep(timeBetweenSteps/abs(yVal))
        elif yVal < -0.1:
            GPIO.output(baseStepDir, GPIO.LOW)
            GPIO.output(baseStepPin, GPIO.HIGH)
            time.sleep(timeBetweenSteps/abs(yVal))
            GPIO.output(baseStepPin, GPIO.LOW)
            time.sleep(timeBetweenSteps/abs(yVal))

    else:
        time.sleep(timeBetweenSteps)
        pass

def main():
    print("Starting base movement script")
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(step2en, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(step2pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(step2dir, GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(baseStepEn, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(baseStepPin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(baseStepDir, GPIO.OUT, initial=GPIO.LOW)
    # GPIO.output(step2en, GPIO.LOW)
    try:
        with Xbox360Controller(0, axis_threshold=0.1) as controller:
            controller.button_a.when_pressed = on_button_pressed

                    # Left and right axis move event
            # controller.axis_l.when_moved= on_axis_moved
            controller.axis_r.when_moved= on_axis_moved

            while True:
                updatePos()
            # signal.pause()


    except KeyboardInterrupt:
        GPIO.output(step2en, GPIO.HIGH)
        GPIO.output(baseStepEn, GPIO.HIGH)
        pass


if __name__ == "__main__":
    main()
