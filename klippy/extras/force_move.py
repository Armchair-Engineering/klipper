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
    #angle in degs, speed in deg/s and accel in deg/s/s
    #same as calc_move_time_polar, but axis_r (normalized move vector) needs to match such that 
    #   only the bed moves the given distance
    RADIUS = 10
    if not angle:
        angle = 0
    if accel == 0:
        accel = 10
    segmentation_angle_degs = 10
    num_segments = int(angle / float(segmentation_angle_degs))
    if angle % segmentation_angle_degs != 0:
        num_segments += 1
    cartesian_start = (RADIUS,0)
    max_cruise_v2 = angle * accel
    moves = []
    if max_cruise_v2 < speed**2:
        speed = math.sqrt(max_cruise_v2)
    cur_speed = 0
    state = "accelerating"
    for i in range(num_segments):
        is_last = i == num_segments - 1
        start_angle_degs = i * segmentation_angle_degs
        end_angle_degs = (i + 1) * segmentation_angle_degs
        if end_angle_degs > angle:
            end_angle_degs = angle
        start_angle = math.radians(start_angle_degs)
        ending_angle = math.radians(end_angle_degs)
        angle_delta = ending_angle - start_angle
        cartesian_end = polar_to_cartesian(RADIUS, ending_angle)
        cartesian_end = (round(cartesian_end[0],10), round(cartesian_end[1],10))
        x_move = cartesian_end[0] - cartesian_start[0]
        y_move = cartesian_end[1] - cartesian_start[1]
        total_move_dist = math.sqrt(x_move**2 + y_move**2)
        inv_dist = 1. / total_move_dist
        x_ratio = x_move * inv_dist
        y_ratio = y_move * inv_dist
        print("moving from %s to %s" % (cartesian_start, cartesian_end))
        #how long it takes to get up to cruising speed
        angle_delta_degs = math.degrees(angle_delta)
        decel_t = 0
        accel_t = 0
        if state == "cruising":
            decel_t = cur_speed / accel
            accel_t = 0
            speed_left = 0 - cur_speed
            accel_decel_d = decel_t * speed_left #how far we have to spin to get up to speed
            if is_last:
                state = "decelerating"
                accel_t = decel_t
        else:
            speed_left = speed - cur_speed
            accel_t = speed_left / accel
            accel_decel_d = accel_t * speed_left #how far we have to spin to get up to speed
        
        if abs(accel_decel_d) > angle_delta_degs: #if we won't get up to speed before we hit the end of the move
            if state == "cruising":
                state = "decelerating"
            cruise_t = 0 # we won't be cruising at all
            accel_t = math.sqrt(angle_delta_degs / accel)
            if state == "accelerating":
                cur_speed = cur_speed + (accel * accel_t) #add acceled speed
            else:
                cur_speed = cur_speed - (accel * accel_t) #substract acceled speed
        else:
            if state == "accelerating":
                state = "cruising"
                cruise_t = (angle_delta_degs - abs(accel_decel_d)) / speed
            elif state == "cruising":
                cruise_t = (angle_delta_degs) / speed
            elif state == "decelerating":
                cruise_t = (angle_delta_degs - abs(accel_decel_d)) / speed
            cur_speed = speed
        if state == "decelerating":
            decel_t = accel_t
        move = (cartesian_end[0], cartesian_end[1], x_ratio, y_ratio, round(accel_t,10), round(cruise_t,10), decel_t, speed)
        moves.append(move)
        cartesian_start = cartesian_end
   
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
                self.trapq_append(self.trapq, total_time, accel_t, cruise_t, decel_t,
                            start_pos[0], start_pos[1], 0., axis_r_x, axis_r_y, 0., 0., cruise_v, accel)
                total_time += accel_t + cruise_t + decel_t
                logging.info("accel_t: %s, cruise_t: %s, decel_t: %s, cruise_v: %s" % (accel_t, cruise_t, decel_t, cruise_v))
                logging.info("moved from %s to %s in %s" % (start_pos[:-1], (end_x, end_y), total_time))
                start_pos = (end_x, end_y, 0.)
                stepper.generate_steps(total_time)
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
