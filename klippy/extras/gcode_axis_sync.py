# Synchronize manual_stepper GCODE_AXIS with existing axes motions
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging
import string

class GCodeAxisSync:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()

        # Initial state
        self.synced_axes = {}

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')

        # Register g-code commands
        self.gcode.register_command("GCODE_AXIS_SYNC", self.cmd_GCODE_AXIS_SYNC,
                                    desc = "Synchronize manual_stepper with existing axes")


    def handle_connect(self):
        # Intercept self.toolhead.move commands and allow for synchronization later on
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.original_move = self.toolhead.move
        self.toolhead.move = self.intercept_move

    
    def handle_ready(self):
        pass
        

    def cmd_GCODE_AXIS_SYNC(self, gcmd):
        stepper = gcmd.get('STEPPER')
        master_axis_id = gcmd.get('AXIS', None)
        absolute = bool(gcmd.get_int('ABSOLUTE', 0))
        limited = bool(gcmd.get_int('LIMITED', 0))
        offset = gcmd.get_float('OFFSET', 0.0)

        try:
            if master_axis_id is None:
                self.unsync_manual_stepper(stepper)
                gcmd.respond_info(f"Unsynced manual_stepper '{stepper}'")
            else:
                self.sync_manual_stepper(stepper, master_axis_id, absolute=absolute, limited=limited, offset=offset)
                gcmd.respond_info(f"Synced manual_stepper '{stepper}' with axis '{master_axis_id}'")
        except ValueError as e:
            raise gcmd.error(str(e))
        except Exception as e:
            logging.exception(f"Exception: {e}")
            raise gcmd.error(f"Failed to process GCODE_AXIS_SYNC command: {str(e)}")


    def sync_manual_stepper(self, stepper, master_axis_id, absolute=False, limited=False, offset=0.0):
        if isinstance(stepper, tuple) and len(stepper) == 2:
            stepper_name = stepper[1].get_steppers()[0].get_name().split()[1]
            stepper_object = stepper[1]
        elif isinstance(stepper, str):
            stepper_name = stepper
            stepper_object = None
            for manual_stepper in self.printer.lookup_objects('manual_stepper'):
                rail_name = manual_stepper[1].get_steppers()[0].get_name().split()[1]
                if rail_name == stepper_name:
                    stepper_object = manual_stepper[1]
                    break
            if stepper_object is None:
                raise Exception(f"Could not find manual_stepper with name '{stepper_name}'")
        else:
            raise ValueError("Invalid stepper format. Expected tuple or string.")

        for sync_info in self.synced_axes.values():
            if sync_info['stepper'].get_steppers()[0].get_name() == stepper_name:
                logging.warning(f"Manual stepper '{stepper_name}' is already synced")
                return

        axis_id, axis_idx = self.allocate_axis(stepper_object)
        position_min = stepper_object.position_min if limited else None
        position_max = stepper_object.position_max if limited else None
        self.synced_axes[axis_idx] = {
            'stepper': stepper_object,
            'master_axis_id': master_axis_id,
            'axis_limits': (position_min, position_max),
            'offset': offset,
            'absolute': absolute
        }


    def unsync_manual_stepper(self, stepper):
        if isinstance(stepper, str):
            stepper_name = stepper
        elif isinstance(stepper, tuple) and len(stepper) == 2:
            stepper_name = stepper[1].get_steppers()[0].get_name().split()[1]
        else:
            raise ValueError("Invalid stepper format. Expected tuple or string.")
    
        for axis_idx, sync_info in list(self.synced_axes.items()):
            if sync_info['stepper'].get_steppers()[0].get_name().split()[1] == stepper_name:
                del self.synced_axes[axis_idx]
                self.gcode.run_script_from_command(
                    f"MANUAL_STEPPER STEPPER='{stepper_name}' GCODE_AXIS="
                )
                return
        logging.warning(f"Manual stepper '{stepper_name}' was not synced")
        return


    def allocate_axis(self, stepper):
        stepper_name = stepper.get_steppers()[0].get_name().split()[1]
        axis_map = self.gcode_move.axis_map
        used_axes = set(axis_map.keys())
        for axis_id in string.ascii_uppercase:
            if axis_id in ("X", "Y", "Z", "E", "F", "N"):
                continue
            if axis_id not in used_axes:
                self.gcode.run_script_from_command(
                    f"MANUAL_STEPPER STEPPER='{stepper_name}' GCODE_AXIS={axis_id}"
                )
                axis_idx = self.gcode_move.axis_map[axis_id]
                logging.info(f"Allocated axis '{axis_id}' for manual stepper '{stepper_name}'")
                return axis_id, axis_idx

        raise ValueError("No available axis found")

    
    def intercept_move(self, newpos, speed):
        base_pos = self.gcode_move.base_position
        for axis_idx, sync_info in self.synced_axes.items():
            stepper = sync_info['stepper']
            master_axis_id = sync_info['master_axis_id']
            pos_min, pos_max = sync_info['axis_limits']
            offset = sync_info['offset']
            absolute = sync_info['absolute']

            master_axis_idx = self.gcode_move.axis_map.get(master_axis_id)
            if master_axis_idx is None:
                raise ValueError(f"Master axis '{master_axis_id}' not found in axis map")
            if absolute:
                target = newpos[master_axis_idx] + offset
            else:
                delta_pos = newpos[master_axis_idx] - base_pos[master_axis_idx]
                target = stepper.get_position()[0] + delta_pos + offset
            newpos[axis_idx] = max(pos_min or target, min(target, pos_max or target))

        self.original_move(newpos, speed)

def load_config(config):
    return GCodeAxisSync(config)