#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  leonbot.py: LeonBot trial
#  
#  Copyright 2015 Tennessee Carmel-Veilleux <veilleux@tentech.ca>
#  

from threading import Thread
import atexit
import Queue
import sys
import pygame
import time
import smbus
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from Adafruit_PWM_Servo_Driver import PWM

SERVO_HAT_I2C_ADDR = 0x41

#MODE = "joystick"
#MODE = "autonomous"

class VL6180XSuperBasicDriver(object):
    SYSRANGE__START = 0x18
    SYSRANGE__INTERMEASUREMENT_PERIOD = 0x1B 
    SYSALS__INTERMEASUREMENT_PERIOD = 0x3E
    SYSALS__START = 0x38
    SYSTEM__INTERRUPT_CLEAR = 0x15
    RESULT__INTERRUPT_STATUS_GPIO = 0x4F 
    RESULT__RANGE_VAL = 0x62
    RESULT__RANGE_STATUS = 0x4D
    RESULT__ALS_STATUS = 0x4E
    RESULT__ALS_VAL = 0x50
   
    def __init__(self, busnum=1, i2c_addr=0x29):
        self.i2c_addr = i2c_addr
        self.bus = smbus.SMBus(busnum)

    def get_register(self, reg_address):
        a1 = (reg_address >> 8) & 0xFF
        a0 = reg_address & 0xFF
        self.bus.write_i2c_block_data(self.i2c_addr, a1, [a0])
        data = self.bus.read_byte(self.i2c_addr)
        return data

    def get_register_16bit(self, reg_address):
        a1 = (reg_address >> 8) & 0xFF
        a0 = reg_address & 0xFF
        self.bus.write_i2c_block_data(self.i2c_addr, a1, [a0])
        data0 = self.bus.read_byte(self.i2c_addr)
        data1 = self.bus.read_byte(self.i2c_addr)
        return (data0 << 8) | (data1 & 0xFF)

    def set_register(self, reg_address, data):
        a1 = (reg_address >> 8) & 0xFF
        a0 = reg_address & 0xFF
        self.bus.write_i2c_block_data(self.i2c_addr, a1, [a0, (data & 0xFF)])

    def set_register_16bit(self, reg_address, data):
        a1 = (reg_address >> 8) & 0xFF
        a0 = reg_address & 0xFF
        d1 = (data >> 8) & 0xFF
        d0 = data & 0xFF
        self.bus.write_i2c_block_data(self.i2c_addr, a1, [a0, d1, d0])

    def start_ranging(self, meas_period_ms=100, continuous=False):
        self.set_register(self.SYSRANGE__INTERMEASUREMENT_PERIOD, (meas_period_ms/10))
        mode = ((1 if continuous else 0) * 2) | 1
        self.set_register(self.SYSRANGE__START, 0)
        time.sleep(0.1)
        self.set_register(self.SYSTEM__INTERRUPT_CLEAR, 0xFF)
        self.set_register(self.SYSRANGE__START, mode)

    def read_range_mm(self):
        return self.get_register(self.RESULT__RANGE_VAL)

class ControlThread(object):
    def __init__(self, main_motor_hat, params=None):
        # Motor hat for locomotion
        self.main_motor_hat = main_motor_hat
        self.motors = {}
        self.motors["b_left"] = {"motor" : self.main_motor_hat.getMotor(1), "target" : 0.0, "scaler" : 1.0}
        self.motors["f_left"] = {"motor" : self.main_motor_hat.getMotor(2), "target" : 0.0, "scaler" : -1.0}
        self.motors["b_right"] = {"motor" : self.main_motor_hat.getMotor(3), "target" : 0.0, "scaler" : -1.0}
        self.motors["f_right"] = {"motor" : self.main_motor_hat.getMotor(4), "target" : 0.0, "scaler" : 1.0}

        self.params = params if params is not None else {}

        self.servo_controller = PWM(SERVO_HAT_I2C_ADDR)

        self.servo_pwm_freq_hz = self.params.get("servo_pwm_freq_hz", 48)
        self.servo_max_angle_deg = self.params.get("servo_max_angle_deg", 45.0)
        self.servo_max_angle_us = self.params.get("servo_max_angle_us", 400)
        self.servo_neutral_us = self.params.get("servo_neutral_us", 1520)
        # Correction factor to apply to each duration to match the
        # internal oscillator of the PCA9685 on the Servo HAT. The internal
        # RC clock is supposed to be 25MHz, but it can be off
        self.servo_clock_k = self.params.get("servo_clock_k", 1.073446)
        self.servo_controller.setPWMFreq(self.servo_pwm_freq_hz)

        # Queue for input events
        self.input_queue = Queue.Queue(1)
        self.thread = Thread(target=self.process)
        
    def get_input_queue(self):
        return self.input_queue
        
    def set_motor(self, motor, target):
        true_target = target * motor["scaler"]
        
        if motor["target"] == 0.0:
            if true_target > 0.0:
                motor["motor"].run(Adafruit_MotorHAT.FORWARD)
                motor["motor"].setSpeed(int(abs(true_target) * 255))
            elif true_target < 0.0:
                motor["motor"].run(Adafruit_MotorHAT.BACKWARD)
                motor["motor"].setSpeed(int(abs(true_target) * 255))
            
        elif motor["target"] >= 0.0:
            if true_target > 0.0:
                motor["motor"].setSpeed(int(abs(true_target) * 255))
            elif true_target < 0.0:
                motor["motor"].run(Adafruit_MotorHAT.BACKWARD)
                motor["motor"].setSpeed(int(abs(true_target) * 255))
            else:
                motor["motor"].run(Adafruit_MotorHAT.RELEASE)
                motor["motor"].setSpeed(0)
                
        elif motor["target"] < 0:
            if true_target > 0.0:
                motor["motor"].run(Adafruit_MotorHAT.FORWARD)
                motor["motor"].setSpeed(int(abs(true_target) * 255))
            elif true_target < 0.0:
                motor["motor"].setSpeed(int(abs(true_target) * 255))
            else:
                motor["motor"].run(Adafruit_MotorHAT.RELEASE)
                motor["motor"].setSpeed(0)
                
        motor["target"] = true_target

    def set_servo_pulse(self, channel, angle_deg):
        pulse_len_us = float(1e6) # 1,000,000 us per second at 1Hz
        pulse_len_us /= float(self.servo_pwm_freq_hz) # us per pulse
        duration_us = self.servo_clock_k * (self.servo_neutral_us + ((float(angle_deg) / float(self.servo_max_angle_deg)) * float(self.servo_max_angle_us)))
        duration_counts = (duration_us / pulse_len_us) * 4095
        #print "pulse_len_us: %.3f, duration_us=%.3f" % (pulse_len_us, duration_us)
        self.servo_controller.setPWM(channel, 0, int(duration_counts))

    def process(self):
        running = True
        
        while running:
            event = self.input_queue.get()

            if "quit" in event:
                running = False
                for key, motor in self.motors.items():
                    motor["motor"].run(Adafruit_MotorHAT.RELEASE)
                    motor["motor"].setSpeed(0)

            elif "left_y" in event:
                self.set_motor(self.motors["f_left"], event["left_y"])
                self.set_motor(self.motors["b_left"], event["left_y"])
                
                self.set_motor(self.motors["f_right"], event["right_y"])
                self.set_motor(self.motors["b_right"], event["right_y"])
            elif "servo_chan" in event:
                servo_chan = event["servo_chan"]
                angle_deg = event["angle_deg"]
                if angle_deg > self.servo_max_angle_deg:
                    angle_deg = self.servo_max_angle_deg
                elif angle_deg < -self.servo_max_angle_deg:
                    angle_deg = -self.servo_max_angle_deg
                self.set_servo_pulse(servo_chan, angle_deg)
    
    def start(self):
        self.thread.start()
            
    
class InputThread(object):
    def __init__(self):
        self.thread = Thread(target=self.process)
        self.thread.daemon = False
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.keep_alive = True
        self.listeners = []
        
        self.left_y_axis_idx = 1
        #self.right_y_axis_idx = 2
        self.right_y_axis_idx = 3
        self.quit_button_idx = 8
        self.range_less_button_idx = 0
        self.elev_less_button_idx = 1
        self.range_more_button_idx = 2
        self.elev_more_button_idx = 3
        self.left_y_axis_multiplier = -1.0
        self.right_y_axis_multiplier = -1.0
        self.elev_servo_angle = 0.0
        self.range_servo_angle = 0.0
        self.current_range_axis = 0.0
        self.current_elev_axis = 0.0
        self.current_left_y = 0.0
        self.current_right_y = 0.0

    def process(self):
        axes = [ 0.0 ] * self.joystick.get_numaxes()
        buttons = [ False ] * self.joystick.get_numbuttons()
        last_time = 0
        interval = 0.1

        # Set an interval to kick the event loop to get latest value of axes
        pygame.time.set_timer(pygame.USEREVENT + 1, int((interval / 2.0) * 1000))

        old_buttons = []
        
        while self.keep_alive:
            event = pygame.event.wait()
            
            if event.type == pygame.QUIT:
                 self.keep_alive = False
                 self.dispatch(None, None)
            elif event.type == pygame.JOYAXISMOTION:
                e = event.dict
                axes[e['axis']] = e['value']
            elif event.type in [pygame.JOYBUTTONUP, pygame.JOYBUTTONDOWN ]:
                e = event.dict
                buttons[e['button']] ^= True
                if buttons[self.quit_button_idx]:
                    self.dispatch(None, None)
                    self.keep_alive = False
                
            # Employ time-based publishing, due to delayed system response causing queue filling-up
            if ((time.time() - last_time) > interval) or sum(buttons) != sum(old_buttons):
                old_buttons = buttons[:]
                self.dispatch(axes, buttons)
                last_time = time.time()

    def send_dispath_update(self, dispatch_update):
        print dispatch_update
        for listener in self.listeners:
            listener.put(dispatch_update)
                
    def dispatch(self, axes, buttons):
        if axes is not None:
            left_y = axes[self.left_y_axis_idx] * self.left_y_axis_multiplier
            right_y = axes[self.right_y_axis_idx] * self.right_y_axis_multiplier

            if buttons[self.range_less_button_idx]:
                range_axis = 1
            elif buttons[self.range_more_button_idx]:
                range_axis = -1
            else:
                range_axis = 0

            if buttons[self.elev_less_button_idx]:
                elev_axis = 1
            elif buttons[self.elev_more_button_idx]:
                elev_axis = -1
            else:
                elev_axis = 0

            if left_y != self.current_left_y or right_y != self.current_right_y:
                self.current_left_y = left_y
                self.current_right_y = right_y

                dispatch_update = {"left_y": left_y, "right_y": right_y}
                self.send_dispath_update(dispatch_update)
            elif range_axis != self.current_range_axis or elev_axis != self.current_elev_axis:
                if range_axis != 0:
                    self.range_servo_angle += 5.0 if range_axis > 0 else -5.0
                    dispatch_update = {"servo_chan": 0, "angle_deg": self.range_servo_angle}
                    self.send_dispath_update(dispatch_update)

                if elev_axis != 0:
                    self.elev_servo_angle += 5.0 if elev_axis > 0 else -5.0
                    dispatch_update = {"servo_chan": 1, "angle_deg": self.elev_servo_angle}
                    self.send_dispath_update(dispatch_update)
        else:
            dispatch_update = {"quit": True}
            self.send_dispath_update(dispatch_update)


    def add_listener(self, listener):
        self.listeners.append(listener)
        
    def start(self):
        self.thread.start()


class AutonomousModeController(object):
    def __init__(self, motor_controller, params):
        # Range sensor interface
        self.vl6180 = VL6180XSuperBasicDriver()
        self.vl6180.start_ranging(100, True)

        self.servo_controller = PWM(SERVO_HAT_I2C_ADDR)

        self.motor_controller = motor_controller

        self.obstacle_thresh_mm = params.get("obstacle_thresh_mm", 150.0)
        self.forward_speed_percent = params.get("forward_speed_percent", 50.0)
        self.reverse_speed_percent = params.get("reverse_speed_percent", 40.0)
        self.rotation_speed_percent = params.get("rotation_speed_percent", 60.0)
        self.rotation_duration_sec = params.get("rotation_duration_sec", 2.0)
        self.reverse_duration_sec = params.get("reverse_duration_sec", 1.0)
        self.start_time = time.time()
        self.servo_start_time = time.time()

        self.servo_pos_idx = 0
        self.servo_pos_deg = [-10.0, 0.0, 10.0, 0.0]
        #self.servo_pos_deg = [0.0]
        self.servo_interval_sec = 0.4

        self.state = "judge_obstacle"
        self.thread = Thread(target=self.process, name="AutonomousModeController")
        self.running = True

    def set_state(self, new_state):
        print "%s->%s" % (self.state, new_state)
        self.state = new_state

    def _handle_servo(self):
        # Early return if not ready to change servo position.
        if (time.time() - self.servo_start_time) < self.servo_interval_sec:
            return

        # Get next servo position
        self.servo_pos_idx += 1
        if self.servo_pos_idx >= len(self.servo_pos_deg):
            self.servo_pos_idx = 0

        servo_pos_deg = self.servo_pos_deg[self.servo_pos_idx]
        dispatch_update = {"servo_chan": 0, "angle_deg": servo_pos_deg}
        self.motor_controller.put(dispatch_update, block=True)

        self.servo_start_time = time.time()

    def process(self):
        while self.running:
            # Update scanning servo position
            self._handle_servo()

            if self.state == "judge_obstacle":
                range_mm = self.vl6180.read_range_mm()
                #print "judge_obstacle, range=%d" % range_mm
                if range_mm < 20.0:
                    self.motor_controller.put({"quit"})
                    self.running = False
                if range_mm < self.obstacle_thresh_mm:
                    # Saw obstacle, move to reverse
                    self.set_state("evade_reverse")
                    self.start_time = time.time()
                else:
                    # Forward if no obstacle
                    forward_speed = self.forward_speed_percent / 100.0
                    dispatch_update = {"left_y": forward_speed, "right_y": forward_speed}
                    self.motor_controller.put(dispatch_update, block=True)
            elif self.state == "evade_reverse":
                if (time.time() - self.start_time) >= self.reverse_duration_sec:
                    # If we have finished backing away, go to rotate
                    self.set_state("evade_rotate")
                    self.start_time = time.time()
                else:
                    # Reverse while evading
                    reverse_speed = -self.reverse_speed_percent / 100.0
                    dispatch_update = {"left_y": reverse_speed, "right_y": reverse_speed}
                    self.motor_controller.put(dispatch_update, block=True)
            elif self.state == "evade_rotate":
                # Check for being done
                if (time.time() - self.start_time) >= self.rotation_duration_sec:
                    # If we have finished backing away, go to rotate
                    self.set_state("judge_obstacle")
                    self.start_time = time.time()
                else:
                    rotate_speed = self.rotation_speed_percent / 100.0
                    # FIXME: Always rotating right
                    dispatch_update = {"left_y": rotate_speed, "right_y": -rotate_speed}
                    self.motor_controller.put(dispatch_update, block=True)
            else:
                print "INVALID STATE: %s" % self.state
                self.state = "judge_obstacle"

    def start(self):
        self.thread.start()

    def join(self):
        self.thread.join()

def turnOffMotors():
    global mh
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

def main():
    print "Starting..."
    # create a default object, no changes to I2C address or frequency
    global mh
    mh = Adafruit_MotorHAT(addr=0x62)
    # recommended for auto-disabling motors on shutdown!
    atexit.register(turnOffMotors)

    # Get mode from command line
    if len(sys.argv) >= 2 and sys.argv[1].lower() == "joystick":
       MODE = "joystick"
    else:
       MODE = "autonomous"

    if MODE == "joystick":
        pygame.init()
        pygame.joystick.init()
        #pygame.display.set_mode((1,1))
    
    print "Initialized"
    control_thread = ControlThread(mh)
    control_thread.start()

    if MODE == "joystick":
        input_thread = InputThread()
        input_thread.add_listener(control_thread.get_input_queue())
        input_thread.start()
        print "Threads started"
    elif MODE == "autonomous":
        autonomous_controller = AutonomousModeController(control_thread.get_input_queue(), {})
        autonomous_controller.start()

        print "Threads started"
        autonomous_controller.join()

    return 0

if __name__ == '__main__':
    try:
        print "Sleeping"
        time.sleep(1)
        print "Waking"
        main()
        print "Done"
    except KeyboardInterrupt:
        turnOffMotors()
        sys.exit(1)

