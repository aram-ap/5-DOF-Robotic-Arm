import odrive
from odrive.enums import *
import sys
import time
import RPi.GPIO as GPIO
import signal
from xbox360controller import Xbox360Controller
from enum import Enum
import asyncio


print("Starting robot test script!")
print("finding odrive")
odrv0 = odrive.find_any()
## Initialize odrive
time.sleep(1)
init_pos = 0

if not odrv0:
    print("Couldn't find any odrive connected! Please double check your connection!")
else:
    print(str(odrv0.vbus_voltage))
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)
    init_pos = odrv0.axis0.pos_vel_mapper.pos_rel
    odrv0.axis0.controller.input_pos = 1.2




## Initialize GPIO and necessary parameters

tmc2209StepDiv = 8.0

baseStep = 20
baseDir = 12
baseEn = 19
baseSpeed = 1.0
baseStepAngle = (1.8/5)/tmc2209StepDiv

arm1Step = 23
arm1Dir = 22
arm1En = 13
arm1Speed = 1
arm1StepAngle = (1.8/19.0)/tmc2209StepDiv

arm2Step = 6
arm2Dir = 5
arm2En = 16
arm2Speed = 1
arm2StepAngle = (1.8/19.0)/tmc2209StepDiv

clawStep = 24
clawDir = 25
clawEn = 18
clawSpeed = 4
clawStepAngle = 1.8/tmc2209StepDiv

GPIO.setmode(GPIO.BCM)

GPIO.setup(baseStep, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(baseDir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(baseEn, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(arm1Step, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(arm1Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(arm1En, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(arm2Step, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(arm2Dir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(arm2En, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(clawStep, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(clawDir, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(clawEn, GPIO.OUT, initial=GPIO.LOW)

class Mode(Enum):
    Normal = 1
    Fast = 2

## Initialize system parameters
curr_mode : Mode = Mode.Normal
glob_speed_mult = 1
fast_speed_mult = 1
time_between_steps = 0.0003

base_pos = 0
arm1_pos = 0
arm2_pos = 0
claw_pos = 0

base_desired_angle = 0
arm1_desired_angle = 0
arm2_desired_angle = 0
claw_desired_angle = 0
bldc_desired_rot : float = init_pos

base_last_step = time.time_ns()
arm1_last_step = time.time_ns()
arm2_last_step = time.time_ns()
claw_last_step = time.time_ns()

base_dir_mult = 1
arm1_dir_mult = 1
arm2_dir_mult = 1
claw_dir_mult = 1

base_ismoving = False
arm1_ismoving = False
arm2_ismoving = False
claw_ismoving = False

axis_lx = 0
axis_ly = 0

axis_rx = 0
axis_ry = 0

hat_x = 0
hat_y = 0

axis_lx_last_ping = time.time()
axis_ly_last_ping = time.time()
axis_rx_last_ping = time.time()
axis_ry_last_ping = time.time()

hat_x_last_ping = time.time()
hat_y_last_ping = time.time()

def button_a_pressed():
    pass
    # print("Button a pressed")

def leftJoystickMoved(axis):
    global axis_lx, axis_ly
    # print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    axis_lx = axis.x
    axis_ly = axis.y

def rightJoystickMoved(axis):
    global axis_rx, axis_ry
    # print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
    axis_rx = axis.x
    axis_ry = axis.y

def hatMoved(axis):
    global hat_x, hat_y
    hat_x = axis.x
    hat_y = axis.y

async def move_base():
    global base_pos, base_desired_angle, baseStepAngle, time_between_steps, baseSpeed, base_last_step, baseDir
    if abs(base_pos - base_desired_angle) > baseStepAngle:
        if (time.time_ns() - base_last_step)/1_000_000 > time_between_steps/baseSpeed:
            base_last_step = time.time_ns()
            if (base_pos - base_desired_angle)*base_dir_mult > 0:
                GPIO.output(baseDir, GPIO.HIGH)
                base_pos -= (baseStepAngle) * base_dir_mult
            else:
                base_pos += (baseStepAngle) * base_dir_mult
                GPIO.output(baseDir, GPIO.LOW)

            GPIO.output(baseStep, GPIO.HIGH)
            # await asyncio.sleep(time_between_steps)
            # GPIO.output(baseStep, GPIO.LOW)
            # await asyncio.sleep(time_between_steps)

async def move_claw():
    global claw_pos, claw_desired_angle, clawStepAngle, time_between_steps, clawSpeed, claw_last_step, clawDir
    if abs(claw_pos - claw_desired_angle) > clawStepAngle:
        if (time.time_ns() - claw_last_step)/1_000_000 > time_between_steps/clawSpeed:
            claw_last_step = time.time_ns()
            if (claw_pos - claw_desired_angle)*claw_dir_mult > 0:
                GPIO.output(clawDir, GPIO.HIGH)
                claw_pos -= (clawStepAngle) * claw_dir_mult
            else:
                claw_pos += (clawStepAngle) * claw_dir_mult
                GPIO.output(clawDir, GPIO.LOW)

            GPIO.output(clawStep, GPIO.HIGH)
            # await asyncio.sleep(time_between_steps)
            # GPIO.output(clawStep, GPIO.LOW)
            # await asyncio.sleep(time_between_steps)

async def move_arm1():
    global arm1_pos, arm1_desired_angle, arm1StepAngle, time_between_steps, arm1Speed, arm1_last_step, arm1Dir
    if abs(arm1_pos - arm1_desired_angle) > arm1StepAngle:
        if (time.time_ns() - arm1_last_step)/1_000_000 > time_between_steps/arm1Speed:
            arm1_last_step = time.time_ns()
            if (arm1_pos - arm1_desired_angle)*arm1_dir_mult > 0:
                GPIO.output(arm1Dir, GPIO.HIGH)
                arm1_pos -= (arm1StepAngle) * arm1_dir_mult
            else:
                arm1_pos += (arm1StepAngle) * arm1_dir_mult
                GPIO.output(arm1Dir, GPIO.LOW)

            GPIO.output(arm1Step, GPIO.HIGH)
            # await asyncio.sleep(time_between_steps)
            # GPIO.output(arm1Step, GPIO.LOW)
            # await asyncio.sleep(time_between_steps)
async def move_arm2():
    global arm2_pos, arm2_desired_angle, arm2StepAngle, time_between_steps, arm2Speed, arm2_last_step, arm2Dir
    if abs(arm2_pos - arm2_desired_angle) > arm2StepAngle:
        if (time.time_ns() - arm2_last_step)/1_000_000 > time_between_steps/arm2Speed:
            arm2_last_step = time.time_ns()
            if (arm2_pos - arm2_desired_angle)*arm2_dir_mult > 0:
                GPIO.output(arm2Dir, GPIO.HIGH)
                arm2_pos -= (arm2StepAngle) * arm2_dir_mult
            else:
                arm2_pos += (arm2StepAngle) * arm2_dir_mult
                GPIO.output(arm2Dir, GPIO.LOW)

            GPIO.output(arm2Step, GPIO.HIGH)
            # await asyncio.sleep(time_between_steps)
            # GPIO.output(arm2Step, GPIO.LOW)
            # await asyncio.sleep(time_between_steps)

async def move_bldc():
    global bldc_desired_rot, odrv0
    odrv0.axis0.controller.input_pos = bldc_desired_rot

async def update_all():
    await asyncio.create_task(move_base())
    await asyncio.create_task(move_claw())
    await asyncio.create_task(move_arm1())
    await asyncio.create_task(move_arm2())

async def update_pos():
    global axis_rx, axis_ry, axis_lx, axis_ly, glob_speed_mult
    global base_desired_angle, base_ismoving, baseSpeed
    global claw_desired_angle, claw_ismoving, clawSpeed
    global arm1_desired_angle, arm1_ismoving, arm1Speed
    global arm2_desired_angle, arm2_ismoving, arm2Speed
    global axis_lx_last_ping, axis_ly_last_ping, axis_rx_last_ping, axis_ry_last_ping
    input_update_inter = .02
    axis_spacing = 0.4


    if time.time() - axis_rx_last_ping > input_update_inter:
        axis_rx_last_ping = time.time()
        if axis_rx > axis_spacing:
            base_desired_angle += baseSpeed * glob_speed_mult * base_dir_mult
            print("Base Desired Angle: " + str(base_desired_angle))
        elif axis_rx < -axis_spacing:
            base_desired_angle -= baseSpeed * glob_speed_mult * base_dir_mult
            print("Base Desired Angle: " + str(base_desired_angle))

    if time.time() - axis_lx_last_ping > input_update_inter:
        axis_lx_last_ping = time.time()
        if axis_lx > axis_spacing:
            claw_desired_angle += clawSpeed * glob_speed_mult * claw_dir_mult
            print("Claw Desired Angle: " + str(claw_desired_angle))
        elif axis_lx < -axis_spacing:
            claw_desired_angle -= clawSpeed * glob_speed_mult * claw_dir_mult
            print("Claw Desired Angle: " + str(claw_desired_angle))

    if time.time() - axis_ry_last_ping > input_update_inter:
        axis_ry_last_ping = time.time()
        if axis_ry > axis_spacing:
            arm1_desired_angle += arm1Speed * glob_speed_mult * arm1_dir_mult
            print("Arm1 Desired Angle: " + str(arm1_desired_angle))
        elif axis_ry < -axis_spacing:
            arm1_desired_angle -= arm1Speed * glob_speed_mult * arm1_dir_mult
            print("Arm1 Desired Angle: " + str(arm1_desired_angle))

    if time.time() - axis_ly_last_ping > input_update_inter:
        axis_ly_last_ping = time.time()
        if axis_ly > axis_spacing:
            arm2_desired_angle += arm2Speed * glob_speed_mult * arm2_dir_mult
            print("Arm2 Desired Angle: " + str(arm2_desired_angle))
        elif axis_ly < -axis_spacing:
            arm2_desired_angle -= arm2Speed * glob_speed_mult * arm2_dir_mult
            print("Arm2 Desired Angle: " + str(arm2_desired_angle))

    if time.time() - hat_x > input_update_inter:
        hat_x = time.time()
        if hat_x > axis_spacing:
            bldc_desired_rot += 0.05 * glob_speed_mult
            print("Bldc Desired Rotation: " + str(bldc_desired_rot))
        elif hat_x < -axis_spacing:
            bldc_desired_rot -= 0.05 * glob_speed_mult
            print("Bldc Desired Rotation: " + str(bldc_desired_rot))



    await move_base()
    await move_claw()
    await move_arm1()
    await move_arm2()
    time.sleep(time_between_steps)
    GPIO.output(baseStep, GPIO.LOW)
    GPIO.output(clawStep, GPIO.LOW)
    GPIO.output(arm1Step, GPIO.LOW)
    GPIO.output(arm2Step, GPIO.LOW)

    # await update_all()
    # await asyncio.create_task(move_base())
    # await asyncio.create_task(move_claw())
    # await asyncio.create_task(move_arm1())
    # await asyncio.create_task(move_arm2())

async def main():
    global base_desired_angle, arm1_desired_angle, arm2_desired_angle, claw_desired_angle
    global base_pos, arm1_pos, arm2_pos, claw_pos, init_pos
    try:
        with Xbox360Controller(0, axis_threshold=0.1) as controller:
            controller.button_a.when_pressed = button_a_pressed
            controller.axis_r.when_moved = rightJoystickMoved
            controller.axis_l.when_moved = leftJoystickMoved
            controller.hat.when_moved = hatMoved

            

            base_desired_angle = 0
            arm1_desired_angle = -45
            arm2_desired_angle = -45
            claw_desired_angle = 0

            while(True):
                await update_pos()
            # signal.pause()

    except KeyboardInterrupt:
        arm1_desired_angle = 0
        arm2_desired_angle = 0
        claw_desired_angle = 0
        base_desired_angle = 0
        while(abs(base_pos) > 1 or abs(arm1_pos) > 1 or abs(arm2_pos) > 1 or abs(claw_pos) > 1):
            await update_pos()

        if odrv0:
            odrv0.axis0.controller.input_pos = init_pos
            while(abs(init_pos - odrv0.axis0.pos_vel_mapper.pos_rel) > 0.01):
                time.sleep(0.1)

            odrv0.axis0.requested_state = AXIS_STATE_IDLE


        GPIO.output(baseEn, GPIO.HIGH)
        GPIO.output(arm1En, GPIO.HIGH)
        GPIO.output(arm2En, GPIO.HIGH)
        GPIO.output(clawEn, GPIO.HIGH)

if __name__ == "__main__":
        asyncio.run(main())

