# Support for a homeable stepper
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import chelper
import logging
from . import force_move
from ..stepper import MCU_stepper
from ..stepper import parse_step_distance, parse_gear_ratio


class HomeableStepper(MCU_stepper):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.can_home = True
        self.next_cmd_time = 0.0
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves

        # Register commands
        gcode = self.printer.lookup_object("gcode")
        self.endstops = []
        self.endstop_map = {}
        self.add_endstop(config)

    def setup_itersolve_for_homing(self):
        self.setup_itersolve("cartesian_stepper_alloc", b"x")
        self.set_trapq(self.trapq)

    def sync_print_time(self):
        toolhead = self.printer.lookup_object("toolhead")
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def do_enable(self, enable):
        self.sync_print_time()
        stepper_enable = self.printer.lookup_object("stepper_enable")
        if enable:
            se = stepper_enable.lookup_enable(self.get_name())
            se.motor_enable(self.next_cmd_time)
        else:
            se = stepper_enable.lookup_enable(self.get_name())
            se.motor_disable(self.next_cmd_time)
        self.sync_print_time()

    def do_set_position(self, setpos):
        self.set_position([setpos, 0.0, 0.0])

    def do_manual_move(self, movepos, speed, accel, sync=True):
        self.sync_print_time()
        cp = self.get_commanded_position()
        # always in radians
        speed = math.radians(speed)
        accel = math.radians(accel)
        dist = movepos - cp
        logging.info("dist: %s", dist)
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            dist, speed, accel
        )
        self.trapq_append(
            self.trapq,
            self.next_cmd_time,
            accel_t,
            cruise_t,
            accel_t,
            cp,
            0.0,
            0.0,
            axis_r,
            0.0,
            0.0,
            0.0,
            cruise_v,
            accel,
        )
        self.next_cmd_time = self.next_cmd_time + accel_t + cruise_t + accel_t
        self.generate_steps(self.next_cmd_time)
        self.trapq_finalize_moves(self.trapq, self.next_cmd_time + 99999.9)
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.note_kinematic_activity(self.next_cmd_time)
        if sync:
            self.sync_print_time()

    def home(self, accel):
        self.do_homing_move(
            360,
            self.homing_speed,
            accel,
            True,
            True,
        )

    def do_homing_move(self, movepos, speed, accel, triggered, check_trigger):
        if not self.can_home:
            raise self.printer.command_error(
                "No endstop for this manual stepper"
            )
        self.homing_accel = accel
        pos = [movepos, 0.0, 0.0, 0.0]
        endstops = self.endstops
        phoming = self.printer.lookup_object("homing")
        phoming.manual_home(
            self, endstops, pos, speed, triggered, check_trigger
        )

    def add_endstop(self, config):
        stepper = self
        if self.endstops and config.get("endstop_pin", None) is None:
            # No endstop defined - use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        endstop_pin = config.get("endstop_pin")
        printer = config.get_printer()
        ppins = printer.lookup_object("pins")
        pin_params = ppins.parse_pin(endstop_pin, True, True)
        # Normalize pin name
        pin_name = "%s:%s" % (pin_params["chip_name"], pin_params["pin"])
        # Look for already-registered endstop
        # New endstop, register it
        mcu_endstop = ppins.setup_pin("endstop", endstop_pin)
        self.endstop_map[pin_name] = {
            "endstop": mcu_endstop,
            "invert": pin_params["invert"],
            "pullup": pin_params["pullup"],
        }
        name = stepper.get_name(short=True)
        self.endstops.append((mcu_endstop, name))
        query_endstops = printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(mcu_endstop, name)

        self.position_endstop = config.getfloat("position_endstop")
        self.homing_speed = config.getfloat("homing_speed", 5.0, above=0.0)
        self.second_homing_speed = config.getfloat(
            "second_homing_speed", self.homing_speed / 2.0, above=0.0
        )
        self.homing_retract_speed = config.getfloat(
            "homing_retract_speed", self.homing_speed, above=0.0
        )
        self.homing_retract_dist = config.getfloat(
            "homing_retract_dist", 5.0, minval=0.0
        )
        self.homing_positive_dir = config.getboolean(
            "homing_positive_dir", False
        )
        # self.axis = "X"

    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        self.sync_print_time()

    def get_position(self):
        # axis_index = "XYZ".find(self.axis)
        # pos = [0.0, 0.0, 0.0, 0.0]
        # [pos[axis_index]] = self.get_commanded_position()
        # return pos

        return [self.get_commanded_position(), 0.0, 0.0]

    def set_position(self, newpos, homing_axes=()):
        self.do_set_position(newpos[0])

    def get_last_move_time(self):
        self.sync_print_time()
        return self.next_cmd_time

    def dwell(self, delay):
        self.next_cmd_time += max(0.0, delay)

    def drip_move(self, newpos, speed, drip_completion):
        self.do_manual_move(newpos[0], speed, self.homing_accel)

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return [self]

    def calc_position(self, stepper_positions):
        return [stepper_positions[self.get_name()], 0.0, 0.0]


def HomeablePrinterStepper(config):
    units_in_radians = True
    printer = config.get_printer()
    name = config.get_name()
    # Stepper definition
    ppins = printer.lookup_object("pins")
    step_pin = config.get("step_pin")
    step_pin_params = ppins.lookup_pin(step_pin, can_invert=True)
    dir_pin = config.get("dir_pin")
    dir_pin_params = ppins.lookup_pin(dir_pin, can_invert=True)
    rotation_dist, steps_per_rotation = parse_step_distance(
        config, units_in_radians, True
    )
    step_pulse_duration = config.getfloat(
        "step_pulse_duration", None, minval=0.0, maxval=0.001
    )
    mcu_stepper = HomeableStepper(
        name,
        step_pin_params,
        dir_pin_params,
        rotation_dist,
        steps_per_rotation,
        step_pulse_duration,
        units_in_radians,
    )
    # Register with helper modules
    for mname in ["stepper_enable", "force_move", "motion_report"]:
        m = printer.load_object(config, mname)
        m.register_stepper(config, mcu_stepper)
    return mcu_stepper
