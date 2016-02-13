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
import pygame
import time
import smbus
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

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
        self.set_register(self.SYSRANGE__START, mode)

    def read_range_mm(self):
        return self.get_register(self.RESULT__RANGE_VAL)

class ControlThread(object):
    def __init__(self, main_motor_hat):
        # Motor hat for locomotion
        self.main_motor_hat = main_motor_hat
        self.motors = {}
        self.motors["b_left"] = {"motor" : self.main_motor_hat.getMotor(1), "target" : 0.0, "scaler" : 1.0}
        self.motors["f_left"] = {"motor" : self.main_motor_hat.getMotor(2), "target" : 0.0, "scaler" : -1.0}
        self.motors["b_right"] = {"motor" : self.main_motor_hat.getMotor(3), "target" : 0.0, "scaler" : -1.0}
        self.motors["f_right"] = {"motor" : self.main_motor_hat.getMotor(4), "target" : 0.0, "scaler" : 1.0}

        # Range sensor interface
        self.vl6180 = VL6180XSuperBasicDriver()
        self.vl6180.start_ranging(100, True)

        # Queue for input events
        self.input_queue = Queue.Queue(1)
        self.thread = Thread(target=self.process)
        
    def get_input_queue(self):
        return self.input_queue
        
    def set_motor(self, motor,target):
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
        
    def process(self):
        running = True
        
        while running:
            event = self.input_queue.get()

            if "quit" in event:
                running = False
            elif "left_y" in event:
                self.set_motor(self.motors["f_left"], event["left_y"])
                self.set_motor(self.motors["b_left"], event["left_y"])
                
                self.set_motor(self.motors["f_right"], event["right_y"])
                self.set_motor(self.motors["b_right"], event["right_y"])
            
            range_mm = self.vl6180.read_range_mm()
            print "Range mm: %d" % range_mm
    
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
        self.left_y_axis_multiplier = -1.0
        self.right_y_axis_multiplier = -1.0

        
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
                print buttons
                
                
    def dispatch(self, axes, buttons):
        if axes is not None:
            left_y = axes[self.left_y_axis_idx] * self.left_y_axis_multiplier
            right_y = axes[self.right_y_axis_idx] * self.right_y_axis_multiplier
            dispatch_update = {"left_y": left_y, "right_y": right_y}
        else:
            dispatch_update = {"quit": True}
        
        print dispatch_update
        for listener in self.listeners:
            listener.put(dispatch_update)
                
    def add_listener(self, listener):
        self.listeners.append(listener)
        
    def start(self):
        self.thread.start()


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
    
    pygame.init()
    pygame.joystick.init()
    #pygame.display.set_mode((1,1))
    
    print "Initialized"
    control_thread = ControlThread(mh)
    
    input_thread = InputThread()
    input_thread.add_listener(control_thread.get_input_queue())
    
    control_thread.start()
    input_thread.start()
    print "Threads started"
    
    return 0

if __name__ == '__main__':
    print "Sleeping"
    time.sleep(1)
    print "Waking"
    main()
    print "Done"

