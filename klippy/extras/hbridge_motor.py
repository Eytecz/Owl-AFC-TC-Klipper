# Manual or automated control of a DC motor via H-Bridge
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging
from . import pulse_counter, output_pin

class HBridgeMotor:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Initial state
        self.last_value_in1 = self.last_value_in1 = 0.

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("gcode:request_restart", self.handle_restart)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')


        # Read config section
        self.name = self.config.get_name().split()[1]
        self.max_power = self.config.getfloat('max_power', 1., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1, minval=0.)
        self.off_below = config.getfloat('off_below', 0., minval=0., maxval=1.)
        cycle_time = config.getfloat('cycle_time', 0.1, minval=0.01)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        dir_invert = self.config.getboolean('dir_invert', False)

        # Setup pin objects
        ppins = self.printer.lookup_object('pins')
        for attr, pin_name in (('in1_pin', config.get('in1_pin')),
                               ('in2_pin', config.get('in2_pin'))):
            pin = ppins.setup_pin('pwm', pin_name)
            pin.setup_max_duration(0.)
            pin.setup_cycle_time(cycle_time, hardware_pwm)
            pin.setup_start_value(0.)
            setattr(self, attr, pin)
        
        self.enable_pin = ppins.setup_pin('digital_out', config.get('enable_pin'))
        self.enable_pin.setup_max_duration(0.)


        # Create g-code request queue
        self.gcrq = output_pin.GCodeRequestQueue(config, self.in1_pin.get_mcu(),
                                                 self._apply_speed())

        # Register g-code commands
        self.cmd_SET_DRV_MOTOR_help = "Set the speed of the H-Bridge motor. Usage: SET_DRV_MOTOR <value> "
        self.gcode.register_mux_command('SET_DRV_MOTOR', 'MOTOR', self.name,
                                        self.cmd_SET_DRV_MOTOR, desc=self.cmd_SET_DRV_MOTOR_help)
    
    
    def handle_ready(self):
        pass

    def handle_connect(self):
        pass

    def handle_restart(self):
        pass
    
    def _apply_speed(self, print_time, value):
        if value < self.off_below:
            value = 0.
        value = max(0., min(self.max_power, value * self.max_power))
        if value == self.last_value_in1:
            return "discard", 0.
        if self.enable_pin:
            if value > 0 and self.last_value_in1 == 0.:
                self.enable_pin.set_digital(print_time, 1)
            elif value == 0. and self.last_value_in1 > 0.:
                self.enable_pin.set_digital(print_time, 0)
        if (value and self.kick_start_time
            and (not self.last_value_in1 or value - self.last_value_in1 > .5)):
            self.last_req_value_in1 = value
            self.last_value_in1 = self.max_power
            self.in1_pin.set_pwm(print_time, self.max_power)
            return "delay", self.kick_start_time
        self.last_value_in1 = value
        self.in1_pin.set_pwm(print_time, value)
    
    def set_speed_from_command(self, value):
        self.gcrq.queue_gcode_request(value)

    def cmd_SET_DRV_MOTOR(self, gcmd):
        value = gcmd.get_float('VALUE', 0., minval=0., maxval=1.)
        self.set_speed_from_command(value)


def load_config_prefix(config):
    return HBridgeMotor(config)
