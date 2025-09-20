# Automated control of spool motion functionality
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import math
import logging

class SpoolMotionControl:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Initial state
        self.enable_spool_measurement = False
        self.enable_stepper_tracking = False
        self.tracking_state = False
        self.tracking_pos = None
        self.enable_forward_assist = False
        self.enable_rewind_assist = True

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')

        # Read config section
        self.name = config.get_name().split()[1]
        self.spool_diameter = [config.getfloat('spool_diameter_min', 80.0),
                               config.getfloat('spool_diameter_max', 200.0)]
        self.poll_interval = config.getfloat('poll_interval', 0.5, minval=0.01)
        self.assist_threshold = config.getfloat('assist_threshold', 20.0, minval=0.0)


    def handle_ready(self):
        pass

    def handle_connect(self):
        # Connect hbridge_motor modules 
        hbridge_motor_name = config.get('hbridge_motor', None)
        if hbridge_motor_name:
            for hbridge_motor in self.printer.lookup_objects('hbridge_motor'):
                name = hbridge_motor.name
                if name == hbridge_motor_name:
                    self.hbridge_motor = hbridge_motor
                    return
            if self.hbridge_motor is None:
                raise self.config.error("Could not find hbridge_motor '%s'" % hbridge_motor_name)
        else:
            raise self.config.error("Missing required 'hbridge_motor' config option")

        # Connect tracked stepper
        tracked_stepper_name = config.get('tracked_stepper', None)
        if tracked_stepper_name:
            self.tracked_stepper = self.printer.lookup_object(tracked_stepper_name)
            if self.tracked_stepper is None:
                raise self.config.error("Could not find stepper '%s'" % tracked_stepper_name)
            self.enable_stepper_tracking = config.getboolean('enable_stepper_tracking', True)
        else:
            raise self.config.error("Missing required 'tracked_stepper' config option")

        # Connect vl6180 sensor
        vl6180_name = config.get('vl6180_sensor', None)
        if vl6180_name:
            for vl6180 in self.printer.lookup_objects('vl6180'):
                name = vl6180.name
                if name == vl6180_name:
                    self.vl6180 = vl6180
                    return
            if self.vl6180 is None:
                raise self.config.error("Could not find vl6180 '%s'" % vl6180_name)
            self.enable_spool_measurement = config.getboolean('enable_spool_measurement', True)
        else:
            pass
        
        # Intercept self.toolhead.move commands to monitor for print moves
        self.original_move = self.toolhead.move
        
        
    def poll_stepper_start(self):
        pass

    def poll_stepper_stop(self):
        pass

    def stepper_tracking(self, eventtime):
        if not self.tracking_state:
            self.tracking_pos = self.tracked_stepper.get_position()[0]
            self.tracking_state = True
            return eventtime + self.poll_interval

        pos = self.tracked_stepper.get_position()[0]
        delta_pos = pos - self.tracking_pos
        
        if abs(delta_pos) >= self.assist_threshold:
            self.tracking_pos = pos
            self.assist(delta_pos)



        

    
        

