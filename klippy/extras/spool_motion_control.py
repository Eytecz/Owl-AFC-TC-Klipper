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

        # MCU Tracking
        self.all_mcus = [m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]

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

        # Read config section
        self.name = config.get_name().split()[1]
        self.spool_diameter = [config.getfloat('spool_diameter_min', 80.0),
                               config.getfloat('spool_diameter_max', 200.0)]
        self.poll_interval = config.getfloat('poll_interval', 0.5, minval=0.01)
        self.assist_threshold = config.getfloat('assist_threshold', 20.0, minval=0.0)
        self.hbridge_motor_name = config.get('hbridge_motor', None)
        self.stepper_name = config.get('stepper', None)
        self.vl6180_name = config.get('vl6180_sensor', None)

        # Register g-code commands
        self.gcode.register_mux_command('SPOOL_MOTION_CONTROL', 'SPOOL', self.name,
                                        self.cmd_SPOOL_MOTION_CONTROL,
                                        desc="Control spool motion functionality.")

    def handle_ready(self):
        pass

    def handle_connect(self):
        # Connect hbridge_motor modules 
        if self.hbridge_motor_name:
            for hbridge_motor in self.printer.lookup_objects('hbridge_motor'):
                name = hbridge_motor[1].get_name()
                if name == self.hbridge_motor_name:
                    self.hbridge_motor = hbridge_motor[1]
            if self.hbridge_motor is None:
                raise self.config.error("Could not find hbridge_motor '%s'" % hbridge_motor_name)
        else:
            raise self.config.error("Missing required 'hbridge_motor' config option")

        # Connect tracked stepper
        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            name = manual_stepper[1].get_steppers()[0].get_name()
            logging.info(f'checking name {name}')
            if name == self.stepper_name:
                self.stepper = manual_stepper[1]
                logging.info(f'Found stepper: {name}')
        if self.stepper is None:
            raise self.config.error("Could not find stepper '%s'" % self.stepper_name)  
        self.enable_stepper_tracking = self.config.getboolean('enable_stepper_tracking', True)

        # Connect vl6180 sensor
        if self.vl6180_name:
            for vl6180 in self.printer.lookup_objects('vl6180'):
                name = vl6180.name
                if name == vl6180_name:
                    self.vl6180 = vl6180
                    return
            if self.vl6180 is None:
                raise self.config.error("Could not find vl6180 '%s'" % vl6180_name)
            self.enable_spool_measurement = self.config.getboolean('enable_spool_measurement', True)
        else:
            pass

        self.toolhead = self.printer.lookup_object('toolhead')
        self.trapq_append_original = self.stepper.trapq_append
        self.stepper.trapq_append = self.trapq_append_intercept

        self.wipe_trapq_original = self.stepper.motion_queuing.wipe_trapq
        self.stepper.motion_queuing.wipe_trapq = self.wipe_trapq_intercept

    def poll_stepper_start(self):
        pass

    def poll_stepper_stop(self):
        pass

    def stepper_tracking(self, eventtime):
        # Add callback to note stepper activity
        motion_queuing = self.stepper.motion_queuing
        trapq = motion_queuing.lookup_trapq_append
        logging.info(f'trapq: {trapq}')
        logging.info(f'self.trapq: {self.trapq}')

        # if not self.tracking_state:
        #     self.tracking_pos = self.tracked_stepper.get_position()[0]
        #     self.tracking_state = True
        #     return eventtime + self.poll_interval

        # pos = self.tracked_stepper.get_position()[0]
        # delta_pos = pos - self.tracking_pos
        
        # if abs(delta_pos) >= self.assist_threshold:
        #     self.tracking_pos = pos
        #     self.assist(delta_pos)
    
    def cmd_SPOOL_MOTION_CONTROL(self, gcmd):
        try:
            self.stepper_tracking(self.reactor.monotonic())
        except Exception as e:
            logging.exception(f"Error in stepper_tracking: {e}")
    
    def trapq_append_intercept(self, *args):
        self.trapq_append_original(*args)
        print_time = self.toolhead.get_last_move_time()
        reactor_time = self.reactor.monotonic()
        logging.info(f'print_time = {print_time}')
        logging.info(f'reactor_time = {reactor_time}')
        logging.info(f'*args = {args}')
        logging.info(f'self.trapq = {args[0]}')
        logging.info(f'movetime = {args[1]}')
        logging.info(f'accel_t = {args[2]}')
        logging.info(f'cruise_t = {args[3]}')
        logging.info(f'accel_t = {args[4]}')
        logging.info(f'axis_r = {args[8]}')
        logging.info(f'cruise_v = {args[12]}')
        logging.info(f'accel = {args[13]}')
        self._motion_extraction(*args)

    def wipe_trapq_intercept(self, *args):
        self.wipe_trapq_original(*args)
        logging.info('wipe_trapq called')

    def _motion_extraction(self, *args):
        curtime = self.reactor.monotonic() 
        print_time = self.mcu.estimated_print_time(curtime)
        movetime_start = args[1]
        move_duration = args[2] + args[3] + args[4]
        movetime_end = movetime_start + move_duration
        move_distance = (1/2 * args[13] * (args[2]**2) + args[12] * args[3] + 1/2 * args[13] * (args[4]**2))* args[8]
        cruise_v = args[12]
        logging.info(f'curtime = {curtime}, print_time = {print_time}, movetime_start = {movetime_start}, movetime_end = {movetime_end}, move_duration = {move_duration}, move_distance = {move_distance}, cruise_v = {cruise_v}')
        

        pwm_value = 1.0
        runtime = move_duration
        self.hbridge_motor.scheduled_motion(pwm_value, move_duration)


def load_config_prefix(config):
    return SpoolMotionControl(config)


        

    
        

