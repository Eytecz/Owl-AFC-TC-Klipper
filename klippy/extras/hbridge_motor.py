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
        self.last_value_in1 = self.last_value_in2 = 0.

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("gcode:request_restart", self.handle_restart)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')


        # Read config section
        self.name = config.get_name().split()[1]
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1, minval=0.)
        self.off_below = config.getfloat('off_below', 0., minval=0., maxval=1.)
        cycle_time = config.getfloat('cycle_time', 0.1, minval=0.01)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        dir_invert = config.getboolean('dir_invert', False)

        # Setup pin objects
        ppins = self.printer.lookup_object('pins')
        pin_pairs = [('in1_pin', config.get('in2_pin') if dir_invert else config.get('in1_pin')),
                     ('in2_pin', config.get('in1_pin') if dir_invert else config.get('in2_pin'))]

        for attr, pin_name in pin_pairs:
            pin = ppins.setup_pin('pwm', pin_name)
            pin.setup_max_duration(0.)
            pin.setup_cycle_time(cycle_time, hardware_pwm)
            pin.setup_start_value(0., 0.)
            setattr(self, attr, pin)

        enable_pin = config.get('enable_pin', None)
        if enable_pin:
            self.enable_pin = ppins.setup_pin('digital_out', enable_pin)
            self.enable_pin.setup_max_duration(0.)


        # Create g-code request queue
        if self.in1_pin.get_mcu() != self.in2_pin.get_mcu():
            raise config.error("in1_pin and in2_pin must be on the same MCU")
        self.gcrq = output_pin.GCodeRequestQueue(config, self.in1_pin.get_mcu(),
                                                 self._apply_speed)

        # Register g-code commands
        self.cmd_SET_DRV_MOTOR_help = "Set the speed of the H-Bridge motor. Usage: SET_DRV_MOTOR <value> "
        self.gcode.register_mux_command('SET_DRV_MOTOR', 'MOTOR', self.name,
                                        self.cmd_SET_DRV_MOTOR, desc=self.cmd_SET_DRV_MOTOR_help)
    
    
    def handle_ready(self):
        pass

    def handle_connect(self):
        pass

    def handle_restart(self, print_time):
        pass
    
    def _apply_speed(self, print_time, value):
        # Apply off threshold
        if abs(value) < self.off_below:
            value = 0.0

        # Clamp
        value = max(-self.max_power, min(self.max_power, value))

        # Track previous requested value
        last_req = getattr(self, 'last_req_value_in1', 0.0)

        # Enable pin handling
        if getattr(self, 'enable_pin', None):
            if value != 0.0 and last_req == 0.0:
                self.enable_pin.set_digital(print_time, 1)
            elif value == 0.0 and last_req != 0.0:
                self.enable_pin.set_digital(print_time, 0)

        # --- Kick-start logic ---
        # Only trigger if moving from stop or large change
        do_kick = (value != 0.0
                and self.kick_start_time > 0
                and (last_req == 0.0 or abs(value) - abs(last_req) > 0.5))
        if do_kick:
            self.last_req_value_in1 = value
            kick_pwm = self.max_power if value > 0 else -self.max_power

            # Forward kick
            if value > 0:
                self.in1_pin.set_pwm(print_time, self.max_power)
                self.in2_pin.set_pwm(print_time, 0)
            else:  # Reverse kick
                self.in1_pin.set_pwm(print_time, 0)
                self.in2_pin.set_pwm(print_time, self.max_power)

            # Update last applied values
            self.last_value_in1 = kick_pwm if value > 0 else 0.0
            self.last_value_in2 = kick_pwm if value < 0 else 0.0

            return "delay", self.kick_start_time

        # --- Normal operation ---
        self.last_req_value_in1 = value

        if value > 0.0:  # Forward
            self.in1_pin.set_pwm(print_time, value)
            self.in2_pin.set_pwm(print_time, 0)
            self.last_value_in1, self.last_value_in2 = value, 0.0

        elif value < 0.0:  # Reverse
            self.in1_pin.set_pwm(print_time, 0)
            self.in2_pin.set_pwm(print_time, -value)
            self.last_value_in1, self.last_value_in2 = 0.0, -value

        else:  # Zero: optionally apply short brake
            if getattr(self, 'brake_time', 0.0) > 0.0 and (self.last_value_in1 != 0.0 or self.last_value_in2 != 0.0):
                # Apply brake pulse: both outputs high for brake_time
                self.in1_pin.set_pwm(print_time, self.max_power)
                self.in2_pin.set_pwm(print_time, self.max_power)
                self.last_value_in1 = self.max_power
                self.last_value_in2 = self.max_power
                # After brake_time, go to coast
                def end_brake():
                    self.in1_pin.set_pwm(print_time, 0)
                    self.in2_pin.set_pwm(print_time, 0)
                    self.last_value_in1 = 0.0
                    self.last_value_in2 = 0.0
                self.reactor.register_timer(self.brake_time, end_brake)
                return "delay", self.brake_time
            else:
                # Coast immediately
                self.in1_pin.set_pwm(print_time, 0)
                self.in2_pin.set_pwm(print_time, 0)
                self.last_value_in1, self.last_value_in2 = 0.0, 0.0

    
    def set_speed_from_command(self, value):
        self.gcrq.queue_gcode_request(value)

    def cmd_SET_DRV_MOTOR(self, gcmd):
        value = gcmd.get_float('VALUE', 0., minval=-1., maxval=1.)
        self.set_speed_from_command(value)


def load_config_prefix(config):
    return HBridgeMotor(config)
