# Utility for manually moving a stepper for diagnostic purposes
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import chelper

BUZZ_DISTANCE = 1.
BUZZ_VELOCITY = BUZZ_DISTANCE / .250
BUZZ_RADIANS_DISTANCE = math.radians(1.)
BUZZ_RADIANS_VELOCITY = BUZZ_RADIANS_DISTANCE / .250
STALL_TIME = 0.100

# Calculate a move's accel_t, cruise_t, and cruise_v
def calc_move_time(dist, speed, accel):
    axis_r = 1.
    if dist < 0.:
        axis_r = -1.
        dist = -dist
    if not accel or not dist:
        return axis_r, 0., dist / speed, speed
    max_cruise_v2 = dist * accel
    if max_cruise_v2 < speed**2:
        speed = math.sqrt(max_cruise_v2)
    accel_t = speed / accel
    accel_decel_d = accel_t * speed
    cruise_t = (dist - accel_decel_d) / speed
    return axis_r, accel_t, cruise_t, speed

def distance(p1, p2):
    return math.sqrt(((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2))

def cartesian_to_polar(x, y):
    return (math.sqrt(x**2 + y**2), math.atan2(y, x))


def polar_to_cartesian(r, theta):
    return (r * math.cos(theta), r * math.sin(theta))



def calc_move_time_polar(angle, speed, accel):
    #dist in degs, speed in deg/s and accel in deg/s/s
    #same as calc_move_time_polar, but axis_r (normalized move vector) needs to match such that 
    #   only the bed moves the given distance
    moves = []
    if not angle:
        angle = 0
    ending_angle = math.radians(angle)
    cartesian_start = (10,0)
    cartesian_end = polar_to_cartesian(10, ending_angle)
    x_move = cartesian_end[0] - cartesian_start[0]
    y_move = cartesian_end[1] - cartesian_start[1]
    dist = math.sqrt(x_move**2 + y_move**2)
    inv_dist = 1. / dist
    x_ratio = x_move * inv_dist
    y_ratio = y_move * inv_dist
    # x moves 1, y moves 1. ratio is 50 for x, 50 for y
    # x moves 1, y moves 0. ratio is 100 for x, 0 for y
    # x_ratio = round(abs(x_move) / (abs(x_move) + abs(y_move)), 10)
    # y_ratio = round(abs(y_move) / (abs(x_move) + abs(y_move)), 10)
    # if x_move < 0:
    #     x_ratio = -x_ratio
    # if y_move < 0:
    #     y_ratio = -y_ratio
    normalized_x = round(x_move / math.sqrt(x_move**2 + y_move**2), 10)
    normalized_y = round(y_move / math.sqrt(x_move**2 + y_move**2), 10)
    logging.info("force move calculated pos, unnormalized: %s", (x_move, y_move))
    logging.info("force move calced pos: %s", (normalized_x, normalized_y))

    if not accel:
        return (x_ratio, y_ratio), 0., angle / speed, speed
    #dist = 90, accel = 10, velocity=5
    # max_cruise_v2 = 900
    #accel_t = 5 / 10 = .5
    #accel_decel_d = .5 * 5 = 2.5
    #cruise_t = (90 - 2.5) / 5 = 16.5
    max_cruise_v2 = angle * accel
    if max_cruise_v2 < speed**2:
        speed = math.sqrt(max_cruise_v2)
    accel_t = speed / accel
    accel_decel_d = accel_t * speed
    cruise_t = (angle - accel_decel_d) / speed
    move = cartesian_end[0], cartesian_end[1], x_ratio, y_ratio, accel_t, cruise_t, accel_t, speed
    moves.append(move)
    return moves

class ForceMove:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        self.polar_bed_stepper_kinematics = ffi_main.gc(
            ffi_lib.polarbed_stepper_alloc(b'a'), ffi_lib.free)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('STEPPER_BUZZ', self.cmd_STEPPER_BUZZ,
                               desc=self.cmd_STEPPER_BUZZ_help)
        if config.getboolean("enable_force_move", False):
            gcode.register_command('FORCE_MOVE', self.cmd_FORCE_MOVE,
                                   desc=self.cmd_FORCE_MOVE_help)
            gcode.register_command('SET_KINEMATIC_POSITION',
                                   self.cmd_SET_KINEMATIC_POSITION,
                                   desc=self.cmd_SET_KINEMATIC_POSITION_help)
    def register_stepper(self, config, mcu_stepper):
        self.steppers[mcu_stepper.get_name()] = mcu_stepper
    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]
    def _force_enable(self, stepper):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        enable = stepper_enable.lookup_enable(stepper.get_name())
        was_enable = enable.is_motor_enabled()
        if not was_enable:
            enable.motor_enable(print_time)
            toolhead.dwell(STALL_TIME)
        return was_enable
    def _restore_enable(self, stepper, was_enable):
        if not was_enable:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.dwell(STALL_TIME)
            print_time = toolhead.get_last_move_time()
            stepper_enable = self.printer.lookup_object('stepper_enable')
            enable = stepper_enable.lookup_enable(stepper.get_name())
            enable.motor_disable(print_time)
            toolhead.dwell(STALL_TIME)
    def manual_move(self, stepper, dist, speed, accel=0.):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        is_polar_bed = 'stepper_bed' in stepper.get_name()
        logging.info("manual move for: %s, is_polar_bed: %s" % (stepper.get_name(), is_polar_bed))
        if is_polar_bed:
            prev_sk = stepper.set_stepper_kinematics(
                self.polar_bed_stepper_kinematics)
        else:
            prev_sk = stepper.set_stepper_kinematics(self.stepper_kinematics)
        
        prev_trapq = stepper.set_trapq(self.trapq)
        if is_polar_bed:
            stepper.set_position((10., 0., 0.))
        else:
            stepper.set_position((0., 0., 0.))
        
        if is_polar_bed:
            moves = calc_move_time_polar(dist, speed, accel)
            start_pos = (10., 0., 0.)
            print_time = toolhead.get_last_move_time()
            total_time = print_time
            for move in moves:
                end_x, end_y, axis_r_x, axis_r_y, accel_t, cruise_t, decel_t, cruise_v = move
                if accel_t != 0 or decel_t != 0:
                    my_accel = accel
                else:
                    my_accel = 0.
                self.trapq_append(self.trapq, total_time, accel_t, cruise_t, decel_t,
                            start_pos[0], start_pos[1], 0., axis_r_x, axis_r_y, 0., 0., cruise_v, my_accel)
                total_time += accel_t + cruise_t + decel_t
                logging.info("accel_t: %s, cruise_t: %s, decel_t: %s, cruise_v: %s" % (accel_t, cruise_t, decel_t, cruise_v))
                logging.info("moved from %s to %s in %s" % (start_pos[:-1], (end_x, end_y), total_time))
                stepper.generate_steps(total_time)
                start_pos = (end_x, end_y, 0.)
            logging.info("total time: %s" % total_time)
        else:
            axis_r, accel_t, cruise_t, cruise_v = calc_move_time(dist, speed, accel)
            print_time = toolhead.get_last_move_time()
            self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
                            0., 0., 0., axis_r, 0., 0., 0., cruise_v, accel)
            total_time = print_time + accel_t + cruise_t + accel_t
            stepper.generate_steps(total_time)
        self.trapq_finalize_moves(self.trapq, total_time + 99999.9)
        stepper.set_trapq(prev_trapq)
        stepper.set_stepper_kinematics(prev_sk)
        toolhead.note_kinematic_activity(total_time)
        toolhead.dwell(accel_t + cruise_t + accel_t)
        
    def _lookup_stepper(self, gcmd):
        name = gcmd.get('STEPPER')
        if name not in self.steppers:
            raise gcmd.error("Unknown stepper %s" % (name,))
        return self.steppers[name]
    cmd_STEPPER_BUZZ_help = "Oscillate a given stepper to help id it"
    def cmd_STEPPER_BUZZ(self, gcmd):
        stepper = self._lookup_stepper(gcmd)
        logging.info("Stepper buzz %s", stepper.get_name())
        was_enable = self._force_enable(stepper)
        toolhead = self.printer.lookup_object('toolhead')
        dist, speed = BUZZ_DISTANCE, BUZZ_VELOCITY
        if stepper.units_in_radians():
            dist, speed = BUZZ_RADIANS_DISTANCE, BUZZ_RADIANS_VELOCITY
        for i in range(10):
            self.manual_move(stepper, dist, speed)
            toolhead.dwell(.050)
            self.manual_move(stepper, -dist, speed)
            toolhead.dwell(.450)
        self._restore_enable(stepper, was_enable)
    cmd_FORCE_MOVE_help = "Manually move a stepper; invalidates kinematics"
    def cmd_FORCE_MOVE(self, gcmd):
        stepper = self._lookup_stepper(gcmd)
        distance = gcmd.get_float('DISTANCE')
        speed = gcmd.get_float('VELOCITY', above=0.)
        accel = gcmd.get_float('ACCEL', 0., minval=0.)
        logging.info("FORCE_MOVE %s distance=%.3f velocity=%.3f accel=%.3f",
                     stepper.get_name(), distance, speed, accel)
        self._force_enable(stepper)
        self.manual_move(stepper, distance, speed, accel)
    cmd_SET_KINEMATIC_POSITION_help = "Force a low-level kinematic position"
    def cmd_SET_KINEMATIC_POSITION(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_last_move_time()
        curpos = toolhead.get_position()
        x = gcmd.get_float('X', curpos[0])
        y = gcmd.get_float('Y', curpos[1])
        z = gcmd.get_float('Z', curpos[2])
        logging.info("SET_KINEMATIC_POSITION pos=%.3f,%.3f,%.3f", x, y, z)
        toolhead.set_position([x, y, z, curpos[3]], homing_axes=(0, 1, 2))

def load_config(config):
    return ForceMove(config)
