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
        self.presync_queue = []

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register required objects
        self.gcode = self.printer.lookup_object('gcode')

        # Register g-code commands
        self.gcode.register_command("GCODE_AXIS_SYNC", self.cmd_GCODE_AXIS_SYNC,
                                    desc="Synchronize manual_stepper with existing axes")

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
        if master_axis_id is not None:
            master_axis_id = master_axis_id.upper()
        if master_axis_id == "":
            master_axis_id = None

        absolute = bool(gcmd.get_int('ABSOLUTE', 0))
        presync = bool(gcmd.get_int('PRESYNC', 0))
        limited = bool(gcmd.get_int('LIMITED', 0))
        offset = gcmd.get_float('OFFSET', 0.0)
        invert = bool(gcmd.get_int('INVERT', 0))

        try:
            if master_axis_id is None:
                self.unsync_manual_stepper(stepper)
                gcmd.respond_info(f"Unsynced manual_stepper '{stepper}'")
            else:
                if master_axis_id not in self.gcode_move.axis_map:
                    raise gcmd.error(f"Unknown axis id: {master_axis_id}")

                self.sync_manual_stepper(stepper, master_axis_id,
                                        absolute=absolute, limited=limited,
                                        invert=invert, offset=offset)
                if presync:
                    self.run_presync_queue()

                gcmd.respond_info(f"Synced manual_stepper '{stepper}' with axis '{master_axis_id}'")

        except ValueError as e:
            raise gcmd.error(str(e))
        except Exception as e:
            logging.exception(f"Exception: {e}")
            raise gcmd.error(f"Failed to process GCODE_AXIS_SYNC command: {str(e)}")

    def add_to_presync_queue(self, stepper_object, master_axis_id, limited, invert, offset, absolute):
        master_axis_idx = self.gcode_move.axis_map.get(master_axis_id)
        if master_axis_idx is None:
            raise ValueError(f"Master axis '{master_axis_id}' not found in axis map")

        direction = -1.0 if invert else 1.0
        last_position = self.gcode_move.last_position
        target = last_position[master_axis_idx] * direction + offset

        if limited:
            if stepper_object.pos_min is not None:
                target = max(stepper_object.pos_min, target)
            if stepper_object.pos_max is not None:
                target = min(stepper_object.pos_max, target)

        self.presync_queue.append({
            'stepper': stepper_object,
            'target': target,
            'master_axis_id': master_axis_id,
            'limited': limited,
            'invert': invert,
            'offset': offset,
            'absolute': absolute
        })

    def run_presync_queue(self):
        if not self.presync_queue:
            return
        try:
            n = len(self.presync_queue)
            for i, entry in enumerate(list(self.presync_queue)):
                stepper_object = entry['stepper']
                target = entry['target']
                sync_flag = 1 if i == n - 1 else 0
                stepper_object.do_move(target, stepper_object.velocity, stepper_object.accel, sync=sync_flag)

            for entry in list(self.presync_queue):
                stepper_object = entry['stepper']
                master_axis_id = entry['master_axis_id']
                limited = entry['limited']
                invert = entry['invert']
                offset = entry['offset']
                absolute = entry['absolute']

                axis_id, axis_idx = self.allocate_axis(stepper_object)
                position_min = stepper_object.pos_min if limited else None
                position_max = stepper_object.pos_max if limited else None

                self.synced_axes[axis_idx] = {
                    'stepper': stepper_object,
                    'master_axis_id': master_axis_id,
                    'axis_limits': (position_min, position_max),
                    'offset': offset,
                    'absolute': absolute,
                    'invert': invert
                }

        finally:
            self.presync_queue.clear()

    def sync_manual_stepper(self, stepper, master_axis_id, absolute=False, limited=False, invert=False, offset=0.0):
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

        if absolute:
            self.add_to_presync_queue(stepper_object, master_axis_id,
                                      limited, invert, offset, absolute)
            return

        axis_id, axis_idx = self.allocate_axis(stepper_object)
        position_min = stepper_object.pos_min if limited else None
        position_max = stepper_object.pos_max if limited else None
        self.synced_axes[axis_idx] = {
            'stepper': stepper_object,
            'master_axis_id': master_axis_id,
            'axis_limits': (position_min, position_max),
            'offset': offset,
            'absolute': absolute,
            'invert': invert
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
                try:
                    self.gcode.run_script_from_command(
                        f"MANUAL_STEPPER STEPPER='{stepper_name}' GCODE_AXIS="
                    )
                except Exception as e:
                    logging.error(f"Error unsyncing manual stepper '{stepper_name}': {e}")
                return

        logging.warning(f"Manual stepper '{stepper_name}' was not synced")

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
                return axis_id, axis_idx

        raise ValueError("No available axis found")

    def intercept_move(self, newpos, speed):
        logging.info(f"GCodeAxisSync: Intercepted move to {newpos} at speed {speed}")
        if self.presync_queue:
            self.run_presync_queue()
            
            newpos_updated_list = self.gcode_move.last_position
            for i in range(len(newpos)):
                newpos_updated_list[i] = newpos[i]
            newpos = newpos_updated_list

        if not self.synced_axes:
            self.original_move(newpos, speed)
            return

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
        logging.info(f"GCodeAxisSync: Updated move to {newpos} at speed {speed}")

def load_config(config):
    return GCodeAxisSync(config)
