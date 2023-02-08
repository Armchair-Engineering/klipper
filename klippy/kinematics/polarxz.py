# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper
import sys
import logging, math
from collections import OrderedDict

EPSILON = 0.001
HALF_PI = math.pi * 0.5
BED_CENTER = (0, 0)


def distance(p1, p2):
    return math.sqrt(((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2))


def sqrdistance(p1, p2):
    return ((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2)


def cartesian_to_polar(x, y):
    return (math.sqrt(x**2 + y**2), math.atan2(y, x))


def polar_to_cartesian(r, theta):
    return (r * math.cos(theta), r * math.sin(theta))


def polar_crosses_zero(p1, p2):
    return abs(p1[1] - p2[2]) - math.pi < EPSILON


def get_quadrant_crosses(p1, p2):
    return int(abs(p2[1] - p1[1]) // HALF_PI)


# def alignment_wip():
#     # GCODE: set current xy position as 0,0
#     angle_1 = 0
#     angle_2 = 120
#     angle_3 = 240

#     dist1 = probe_bearing(angle_1)
#     dist2 = probe_bearing(angle_2)
#     dist3 = probe_bearing(angle_3)

#     p1 = (dist1, angle_1)
#     p2 = (dist2, angle_2)
#     p3 = (dist3, angle_3)
#     # p1, p2 and p3 are polar coordinates
#     cartesian_p1 = polar_to_cartesian(p1[0], p1[1])
#     cartesian_p2 = polar_to_cartesian(p2[0], p2[1])
#     cartesian_p3 = polar_to_cartesian(p3[0], p3[1])
#     circle = circle_from_3_points(cartesian_p1, cartesian_p2, cartesian_p3)
#     circle_center = circle[0]
#     # it's assumed that we started at 0,0.
#     # so the circle center is 0 - the offset in x and y.
#     x_offset = -circle_center[0]
#     y_offset = -circle_center[1]
#     # GCODE: move radius arm to (x_offset)
#     # GCODE: spit back Y offset to calibration command


def circle_from_3_points(p1, p2, p3):
    z1 = complex(p1[0], p1[1])
    z2 = complex(p2[0], p2[1])
    z3 = complex(p3[0], p3[1])

    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError("Duplicate points: %s, %s, %s" % (z1, z2, z3))

    w = (z3 - z1) / (z2 - z1)

    if abs(w.imag) <= 0.0001:
        raise ValueError("Points are collinear: %s, %s, %s" % (z1, z2, z3))

    c = (z2 - z1) * (w - abs(w) ** 2) / (2j * w.imag) + z1
    # Simplified denominator
    r = abs(z1 - c)
    r = round(r, 10)
    c = (round(c.real, 10), round(c.imag, 10))

    return c, r


def get_circle_line_intersections_new(p1, p2, dist):
    # find points on a line that are a given distance from origin
    intersections = []
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]

    m = (y2 - y1) / (x2 - x1)
    # calculate y intercept
    b = y1 - m * x1
    new_x1 = (-(m * x1 + b) + math.sqrt((m * x1 + b) ** 2 - dist**2)) / 2
    new_x2 = (-(m * x1 + b) - math.sqrt((m * x1 + b) ** 2 - dist**2)) / 2
    new_y1 = m * new_x1 + b
    new_y2 = m * new_x2 + b

    intersection1 = (new_x1, new_y1)
    intersection2 = (new_x2, new_y2)

    if (
        p1[0] <= intersection1[0] <= p2[0] or p2[0] <= intersection1[0] <= p1[0]
    ) and (
        p1[1] <= intersection1[1] <= p2[1] or p2[1] <= intersection1[1] <= p1[1]
    ):
        intersections.append(intersection1)
    if (
        p1[0] <= intersection2[0] <= p2[0] or p2[0] <= intersection2[0] <= p1[0]
    ) and (
        p1[1] <= intersection2[1] <= p2[1] or p2[1] <= intersection2[1] <= p1[1]
    ):
        intersections.append(intersection2)

    return intersections


def get_circle_line_intersections(p1, p2, radius):
    # calculate the intersection points of a line and circle
    # https://stackoverflow.com/questions/1073336/circle-line-collision-detection
    # p1, p2 are the endpoints of the line
    # radius is the radius of the circle
    intersections = []
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dr = math.sqrt(dx**2 + dy**2)
    D = (p1[0] * p2[1]) - (p2[0] * p1[1])
    disc = (radius**2) * (dr**2) - (D**2)
    if disc < 0:
        return intersections
    elif disc == 0:
        x = (D * dy) / (dr**2)
        y = (-D * dx) / (dr**2)
        intersections.append((x, y))
    else:
        x1 = ((D * dy) + (math.copysign(1, dy) * dx * math.sqrt(disc))) / (
            dr**2
        )
        x2 = ((D * dy) - (math.copysign(1, dy) * dx * math.sqrt(disc))) / (
            dr**2
        )
        y1 = ((-D * dx) + (abs(dy) * math.sqrt(disc))) / (dr**2)
        y2 = ((-D * dx) - (abs(dy) * math.sqrt(disc))) / (dr**2)
        x1 = round(x1, 14)
        x2 = round(x2, 14)
        y1 = round(y1, 14)
        y2 = round(y2, 14)
        intersection1 = (x1, y1)
        intersection2 = (x2, y2)
        if (
            p1[0] <= intersection1[0] <= p2[0]
            or p2[0] <= intersection1[0] <= p1[0]
        ) and (
            p1[1] <= intersection1[1] <= p2[1]
            or p2[1] <= intersection1[1] <= p1[1]
        ):
            intersections.append(intersection1)
        if (
            p1[0] <= intersection2[0] <= p2[0]
            or p2[0] <= intersection2[0] <= p1[0]
        ) and (
            p1[1] <= intersection2[1] <= p2[1]
            or p2[1] <= intersection2[1] <= p1[1]
        ):
            intersections.append(intersection2)

    return intersections


def get_quadrant_info(p1, p2):
    angles_crossed = []
    start_quadrant = int(p1[1] // HALF_PI)
    end_quadrant = int(p2[1] // HALF_PI)
    num_quadrants_crossed = abs(start_quadrant - end_quadrant)
    if num_quadrants_crossed == 0:
        return angles_crossed
    else:
        angles_crossed = list(
            range(
                (start_quadrant + 1) * HALF_PI,
                1 + (end_quadrant * HALF_PI),
                HALF_PI,
            )
        )
    return num_quadrants_crossed, angles_crossed


def generate_velocity_milestones(offset_dist, rate):
    dist = 100
    nums = []
    while dist > offset_dist:
        newdist = dist * rate
        if dist != 100:
            nums.append(dist)
        dist = newdist
    if nums[-1] != offset_dist:
        nums.append(offset_dist)
    return nums


def crosses_origin(p1, p2):
    return (p1[0] * p2[0]) < 0 and (p1[1] * p2[1]) < 0


def distance_point_to_line(p0, p1, p2):
    return abs(
        ((p2[0] - p1[0]) * (p1[1] - p0[1]))
        - ((p1[0] - p0[0]) * (p2[1] - p1[1]))
    ) / distance(p1, p2)


class PolarXZKinematics:
    def __init__(self, toolhead, config):
        # Setup axis steppers
        self.stepper_bed = stepper.PrinterStepper(
            config.getsection("stepper_bed"), units_in_radians=True
        )
        rail_x = stepper.PrinterRail(config.getsection("stepper_x"))
        rail_z = stepper.PrinterRail(config.getsection("stepper_z"))
        logging.info("rail x endstops: %s", rail_x.get_endstops())
        logging.info("rail z endstops: %s", rail_z.get_endstops())

        rail_x.get_endstops()[0][0].add_stepper(rail_z.get_steppers()[0])
        rail_z.get_endstops()[0][0].add_stepper(rail_x.get_steppers()[0])
        self.stepper_bed.setup_itersolve("polarxz_stepper_alloc", b"a")
        rail_x.setup_itersolve("polarxz_stepper_alloc", b"+")
        rail_z.setup_itersolve("polarxz_stepper_alloc", b"-")
        self.rails = [rail_x, rail_z]
        self.rail_lookup = {"x": rail_x, "z": rail_z}
        self.steppers = [self.stepper_bed] + [
            s for r in self.rails for s in r.get_steppers()
        ]
        self.toolhead = toolhead
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )
        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        # in deg/s
        self.max_rotational_velocity = config.getfloat(
            "max_rotational_velocity", 360, above=0.0
        )
        # convert max_rotational_velocity to radians per second
        self.max_rotational_velocity = math.radians(
            self.max_rotational_velocity
        )
        self.max_rotational_accel = config.getfloat(
            "max_rotational_accel",
            self.max_accel,
            above=0.0,
            maxval=self.max_accel,
        )
        self.max_z_velocity = config.getfloat(
            "max_z_velocity",
            self.max_velocity,
            above=0.0,
            maxval=self.max_velocity,
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", self.max_accel, above=0.0, maxval=self.max_accel
        )
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.0
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.0)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.0)
        self.zero_crossing_radius = config.getfloat(
            "zero_crossing_radius", 0.1, minval=0.0001, above=0.0
        )
        self.min_segment_length = config.getfloat(
            "min_segment_length", 0.2, minval=0.0001, above=0.0
        )
        self.segments_per_second = config.getfloat(
            "segments_per_second", 100, minval=0.0001, above=0.0
        )
        self.two_move_crossing = config.getboolean("two_move_crossing", False)
        self.time_segmentation = config.getboolean("time_segmentation", False)
        self.bed_radius = self.axes_min[0]

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        x_pos = stepper_positions[self.rails[0].get_name()]
        z_pos = stepper_positions[self.rails[1].get_name()]
        radius = 0.5 * (x_pos + z_pos)
        z_pos = 0.5 * (x_pos - z_pos)
        x_pos = radius * math.cos(bed_angle)
        y_pos = radius * math.sin(bed_angle)
        return [x_pos, y_pos, z_pos]

    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if 0 in homing_axes and 1 in homing_axes:
            self.limit_xy2 = self.rails[0].get_range()[1] ** 2

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)

    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.0
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    # def home(self, homing_state):
    #     # Always home XY together
    #     homing_axes = homing_state.get_axes()
    #     home_xy = 0 in homing_axes or 1 in homing_axes
    #     home_z = 2 in homing_axes
    #     updated_axes = []
    #     if home_xy:
    #         updated_axes = [0, 1]
    #     if home_z:
    #         updated_axes.append(2)
    #     homing_state.set_axes(updated_axes)
    #     # Do actual homing
    #     if home_xy:
    #         self._home_axis(homing_state, 0, self.rails[0])
    #         # self._home_axis(homing_state, 1, self.rails[0])
    #     if home_z:
    #         self._home_axis(homing_state, 2, self.rails[1])
    def home(self, homing_state):
        # Each axis is homed independently and in order
        homing_axes = homing_state.get_axes()
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_xy:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        for axis in updated_axes:
            if axis == 2:  # if we're homing z, get the z rail
                rail = self.rails[1]
            elif axis == 1:  # y doesn't do shit
                continue
            elif axis == 0:
                rail = self.rails[0]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            if axis == 0:
                homepos[1] = 0.0
            logging.info("homepos: %s", homepos)
            forcepos = list(homepos)
            logging.info("forcepos: %s", forcepos)
            logging.info("hi.positive_dir: %s", hi.positive_dir)
            logging.info("hi.position_endstop: %s", hi.position_endstop)
            logging.info("position_min: %s", position_min)
            logging.info("position_max: %s", position_max)

            if hi.positive_dir:
                # klipper dies if we do a move at 0,0, so offset position by microstep distance
                # TODO - maybe only offset if it's an x move
                forcepos[axis] -= (
                    hi.position_endstop
                    - position_min
                    - self.zero_crossing_radius
                )
                if forcepos[axis] < position_min:
                    forcepos[axis] = position_min + self.zero_crossing_radius
            else:
                forcepos[axis] += (
                    position_max
                    - hi.position_endstop
                    + self.zero_crossing_radius
                )
                if forcepos[axis] > position_max:
                    forcepos[axis] = position_max - self.zero_crossing_radius
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.0

    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0] ** 2 + end_pos[1] ** 2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.0:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        # Limit the maximum acceleration against the rotational distance theta
        # TODO: Optimize with code from the chelper?
        if move.axes_d[0] or move.axes_d[1]:
            start_xy = move.start_pos
            end_xy = move.end_pos
            # accel = move.accel
            # delta_x = end_xy[0] - start_xy[0]
            # delta_y = end_xy[1] - start_xy[1]
            # calculate accel components of x and y from deltas
            # move_hypot = math.sqrt(delta_x**2 + delta_y**2)
            # accel_x = accel * delta_x / move_hypot
            # accel_y = accel * delta_y / move_hypot
            # convert accel x and accel y to polar components
            # accel_theta = math.atan2(accel_y, accel_x)
            # accel_r = math.sqrt(accel_x**2 + accel_y**2)
            polar_start = cartesian_to_polar(start_xy[0], start_xy[1])
            polar_end = cartesian_to_polar(end_xy[0], end_xy[1])
            # dr = polar_end[0] - polar_start[0]

            # dt = move.min_move_t

            dtheta = polar_end[1] - polar_start[1]

            delta_degrees = math.degrees(dtheta)
            if abs(delta_degrees) > 180:
                if delta_degrees > 0:
                    delta_degrees = delta_degrees - 360
                else:
                    delta_degrees = delta_degrees + 360
            if delta_degrees == 0:
                step_ratio = self.max_accel / self.max_rotational_accel
            else:
                steps_per_degree = 360 / 1.8 * (16 / 120.0)
                delta_distance = distance(move.start_pos, move.end_pos)
                step_ratio = (
                    delta_distance * steps_per_degree / abs(delta_degrees)
                )
                logging.info("delta_degrees: %s" % delta_degrees)
                logging.info("delta_distance: %s" % delta_distance)
                logging.info("steps_per_degree: %s" % steps_per_degree)

            # rotational_velocity = dtheta / dt
            # radial_velocity = dr / dt
            # if radial_velocity == 0:
            #     r = polar_start[0]
            #     #calculate sagitta
            #     sagitta = r - math.sqrt((r**2) - ((distance(move.start_pos, move.end_pos)/2)**2))
            #     radial_velocity = sagitta / dt

            # radius_scale = min(polar_start[0], polar_end[0]) / self.bed_radius

            # rotational_velocity = rotational_velocity * radius_scale
            # if rotational_velocity > self.max_rotational_velocity:
            #     rotational_velocity = self.max_rotational_velocity

            # vx = (radial_velocity * math.cos(polar_start[1])) - (polar_start[0] * rotational_velocity * math.sin(polar_start[1]))
            # vy = (radial_velocity * math.sin(polar_start[1])) + (polar_start[0] * rotational_velocity * math.cos(polar_start[1]))

            # adjusted_velocity = math.sqrt(vx**2 + vy**2)
            # logging.info("adjusted velocity: %s", adjusted_velocity)
            # logging.info("step_ratio: %s" % step_ratio)

            # move.limit_speed(adjusted_velocity, self.max_rotational_accel)
            move.limit_speed(
                abs(step_ratio * self.max_rotational_velocity),
                abs(step_ratio * self.max_rotational_accel),
            )

        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio
            )

    def zero_cross(self, move, total_move_dist):
        cart_start_x = move.start_pos[0]
        cart_start_y = move.start_pos[1]
        cart_end_x = move.end_pos[0]
        cart_end_y = move.end_pos[1]

        zero_radius_intersections = get_circle_line_intersections(
            (cart_start_x, cart_start_y),
            (cart_end_x, cart_end_y),
            self.zero_crossing_radius,
        )

        if len(zero_radius_intersections) == 0:
            return [((move.start_pos, move.end_pos), 1)]
        elif len(zero_radius_intersections) == 1:
            intersection = zero_radius_intersections[0]
            if (
                abs(intersection[0] - cart_start_x) < EPSILON
                and abs(intersection[1] - cart_start_y) < EPSILON
            ):
                return [((move.start_pos, move.end_pos), 1)]
            return [
                (
                    (
                        move.start_pos,
                        (
                            intersection[0],
                            intersection[1],
                            move.end_pos[2],
                            move.end_pos[3],
                        ),
                    ),
                    1,
                )
            ]
        # 2 intersections means moving through 0
        # 2 MOVE ZERO CROSSING
        elif self.two_move_crossing:

            # moving through our zero radius, move 90 deg to incoming line
            # if we know we're zero crossing, the angle to start and to end will always be pi (180deg) apart
            # we want to find a point perpendicular to the line between start and end, at a distance of self.zero_crossing_radius
            incoming_angle = math.atan2(
                zero_radius_intersections[0][1], zero_radius_intersections[0][0]
            )
            incoming_point = polar_to_cartesian(
                self.zero_crossing_radius, incoming_angle
            )
            outgoing_angle = incoming_angle + math.pi
            outgoing_point = polar_to_cartesian(
                self.zero_crossing_radius, outgoing_angle
            )
            side_angle = (outgoing_angle - incoming_angle) / 2
            side_point = polar_to_cartesian(
                self.zero_crossing_radius, side_angle
            )

            half_z = move.axes_d[2] / 2

            half_e = move.axes_d[3] / 2
            side_pos = side_point + (
                move.start_pos[2] + half_z,
                move.start_pos[3] + half_e,
            )
            return [
                ((move.start_pos, side_pos), 0.5),
                ((side_pos, move.end_pos), 0.5),
            ]

        # 4 MOVE ZERO CROSSING
        else:
            # sort zero_radius_intersections by distance from start_pos
            zero_radius_intersections.sort(
                key=lambda x: sqrdistance((x), move.start_pos)
            )
            # moving through our zero radius, move 90 deg to incoming line
            # if we know we're zero crossing, the angle to start and to end will always be pi (180deg) apart
            # we want to find a point perpendicular to the line between start and end, at a distance of self.zero_crossing_radius
            incoming_angle = math.atan2(
                zero_radius_intersections[0][1], zero_radius_intersections[0][0]
            )
            incoming_point = polar_to_cartesian(
                self.zero_crossing_radius, incoming_angle
            )
            incoming_point = (
                round(incoming_point[0], 10),
                round(incoming_point[1], 10),
            )
            outgoing_angle = incoming_angle + math.pi
            outgoing_point = polar_to_cartesian(
                self.zero_crossing_radius, outgoing_angle
            )
            outgoing_point = (
                round(outgoing_point[0], 10),
                round(outgoing_point[1], 10),
            )
            side_angle = (outgoing_angle - incoming_angle) / 2.0
            side_point = polar_to_cartesian(
                self.zero_crossing_radius, side_angle
            )
            side_point = (round(side_point[0], 10), round(side_point[1], 10))
            crossmove_1 = distance(
                (incoming_point[0], incoming_point[1]),
                (side_point[0], side_point[1]),
            )
            crossmove_2 = distance(
                (side_point[0], side_point[1]),
                (outgoing_point[0], outgoing_point[1]),
            )
            total_move_dist = total_move_dist
            total_outermove_dist = total_move_dist - crossmove_1 - crossmove_2
            start_move_dist = end_move_dist = total_outermove_dist / 2.0

            total_z = move.axes_d[2]
            total_e = move.axes_d[3]
            start_move_ratio = start_move_dist / float(total_move_dist)
            move_1_z_dist = total_z * (start_move_ratio)
            move_1_e_dist = total_e * (start_move_ratio)

            crossmove_1_ratio = crossmove_1 / float(total_move_dist)
            move_2_z_dist = total_z * (crossmove_1_ratio)
            move_2_e_dist = total_e * (crossmove_1_ratio)

            crossmove_2_ratio = crossmove_2 / float(total_move_dist)
            move_3_z_dist = total_z * (crossmove_2_ratio)
            move_3_e_dist = total_e * (crossmove_2_ratio)

            end_move_ratio = end_move_dist / float(total_move_dist)
            move_4_z_dist = total_z * (end_move_ratio)
            move_4_e_dist = total_e * (end_move_ratio)

            start_z = move.start_pos[2]
            start_e = move.start_pos[3]

            start_pos = move.start_pos
            new_z = start_z + move_1_z_dist
            new_e = start_e + move_1_e_dist
            incoming_pos = (incoming_point[0], incoming_point[1], new_z, new_e)
            new_z = new_z + move_2_z_dist
            new_e = new_e + move_2_e_dist
            side_pos = (side_point[0], side_point[1], new_z, new_e)
            new_z = new_z + move_3_z_dist
            new_e = new_e + move_3_e_dist
            outgoing_pos = (outgoing_point[0], outgoing_point[1], new_z, new_e)
            new_z = new_z + move_4_z_dist
            new_e = new_e + move_4_e_dist
            end_pos = move.end_pos
            out_moves = [
                ((start_pos, incoming_pos), start_move_ratio),
                ((incoming_pos, side_pos), crossmove_1_ratio),
                ((side_pos, outgoing_pos), crossmove_2_ratio),
                ((outgoing_pos, end_pos), end_move_ratio),
            ]
            if (
                abs(out_moves[0][0][0][0] - start_pos[0]) < EPSILON
                and abs(out_moves[0][0][0][1] - start_pos[1]) < EPSILON
            ):
                out_moves.pop(0)
                out_moves[0] = (
                    (start_pos, out_moves[0][0][1]),
                    out_moves[0][1],
                )
            if (
                abs(out_moves[-1][0][0][0] - end_pos[0]) < EPSILON
                and abs(out_moves[-1][0][0][1] - end_pos[1]) < EPSILON
            ):
                out_moves.pop(-1)
                out_moves[-1] = (
                    (out_moves[-1][0][0], end_pos),
                    out_moves[-1][1],
                )
            return out_moves

    def segment_move(self, move):
        if self.time_segmentation:
            return self.segment_move_by_time(move)
        else:
            return self.segment_move_by_circles(move)

    def segment_move_by_time(self, move):
        logging.info(
            "special_queuing_state: %s", self.toolhead.special_queuing_state
        )
        if self.toolhead.special_queuing_state == "Drip":
            return []
        if move.axes_d[0] or move.axes_d[1]:
            logging.info("parent move: %s", (move.start_pos, move.end_pos))
            move_time = move.min_move_t
            total_segments = self.segments_per_second * move_time
            total_move_dist = distance(move.start_pos, move.end_pos)
            moves_and_ratios = self.zero_cross(move, total_move_dist)
            out_moves = []
            logging.info("moves_and_ratios: %s", moves_and_ratios)
            for tup in moves_and_ratios:

                move, ratio = tup
                segment_count = int(total_segments * ratio)
                move_dist = total_move_dist * ratio
                out_moves += self._segment_move(move, segment_count, move_dist)
            return out_moves
        else:
            return []

    def _segment_move(self, move, num_segments, move_dist):
        move_start_pos = move[0]
        move_end_pos = move[1]
        logging.info("segmenting move!")
        logging.info("move: %s, ", (move_start_pos, move_end_pos))
        dx = move_end_pos[0] - move_start_pos[0]
        dy = move_end_pos[1] - move_start_pos[1]

        if num_segments <= 1:
            return [move]
        segment_dist = move_dist / num_segments
        if segment_dist < self.min_segment_length:
            num_segments = move_dist / self.min_segment_length

        points = []
        stepx = dx / num_segments
        stepy = dy / num_segments
        px = move_start_pos[0] + stepx
        py = move_start_pos[1] + stepy
        logging.info("num_segments: %s" % num_segments)
        for i in range(int(num_segments)):

            points.append((round(px, 10), round(py, 10)))
            px += stepx
            py += stepy

        points = [(move_start_pos[0], move_start_pos[1])] + points
        logging.info("points: %s" % points)
        xy_moves = []
        while len(points) != 1:
            start = points.pop(0)
            end = points[0]
            xy_moves.append((start, end))
        logging.info("xy_moves: %s" % xy_moves)
        total_z_dist = move_end_pos[2] - move_start_pos[2]
        logging.info("total z dist: %s" % total_z_dist)
        total_e_dist = move_end_pos[3] - move_start_pos[3]
        actual_moves = []
        current_z_pos = move_start_pos[2]
        current_e_pos = move_start_pos[3]
        seg_ratio = segment_dist / move_dist
        for xy_move in xy_moves:
            z_dist = seg_ratio * total_z_dist
            e_dist = seg_ratio * total_e_dist
            new_z_pos = current_z_pos + z_dist
            new_e_pos = current_e_pos + e_dist
            actual_moves.append(
                (
                    (
                        round(xy_move[0][0], 10),
                        round(xy_move[0][1], 10),
                        round(current_z_pos, 10),
                        round(current_e_pos, 10),
                    ),
                    (
                        round(xy_move[1][0], 10),
                        round(xy_move[1][1], 10),
                        round(new_z_pos, 10),
                        round(new_e_pos, 10),
                    ),
                )
            )
            current_e_pos = new_e_pos
            current_z_pos = new_z_pos
        leftover = round(num_segments % 1, 10)
        if leftover > 0:
            actual_moves.append((actual_moves[-1][-1], move_end_pos))
        return actual_moves

    def segment_move_by_circles(self, move):
        logging.info(
            "special_queuing_state: %s", self.toolhead.special_queuing_state
        )
        if self.toolhead.special_queuing_state == "Drip":
            return []
        if move.axes_d[0] or move.axes_d[1]:
            # def testit(move):
            logging.info("segmenting move!")
            cart_start_x = move.start_pos[0]
            cart_start_y = move.start_pos[1]
            cart_end_x = move.end_pos[0]
            cart_end_y = move.end_pos[1]
            delta_x = cart_end_x - cart_start_x
            delta_y = cart_end_y - cart_start_y
            logging.info("start_pos: %s", move.start_pos)
            logging.info("end_pos: %s", move.end_pos)

            if delta_x == 0:
                riserun = 0
            else:
                riserun = delta_y / delta_x

            # calculate y intercept
            if riserun == 0:
                riserun2 = 90
            else:
                riserun2 = -1 / riserun

            y_intercept = cart_start_y - (riserun * cart_start_x)
            # calculate x intercept
            # line1 = y = riserun * x + y_intercept
            # line2 = y = riserun2 * x + 0
            # calcualate intersection of two lines
            x_intersect = (y_intercept) / (riserun2 - riserun)
            y_intersect = riserun * x_intersect + y_intercept
            closest_to_origin = (x_intersect, y_intersect)

            dist_start_sqred = sqrdistance(move.start_pos, BED_CENTER)
            dist_end_sqred = sqrdistance(move.end_pos, BED_CENTER)
            midpoint = (
                (move.start_pos[0] + move.end_pos[0]) / 2,
                (move.start_pos[1] + move.end_pos[1]) / 2,
            )
            dist_midpoint_sqred = sqrdistance(midpoint, BED_CENTER)
            dist_min_sqred = sqrdistance(closest_to_origin, BED_CENTER)

            use_min = False
            if (
                (cart_start_x <= closest_to_origin[0] <= cart_end_x)
                or (cart_start_x >= closest_to_origin[0] >= cart_end_x)
            ) and (
                (cart_start_y <= closest_to_origin[1] <= cart_end_y)
                or (cart_start_y >= closest_to_origin[1] >= cart_end_y)
            ):
                use_min = True

            velocity_milestones = generate_velocity_milestones(
                self.zero_crossing_radius, 0.5
            )
            # velocity milestones are sorted by distance, descending
            start_circle_index = None
            end_circle_index = None
            mid_circle_index = None
            for index, radius in enumerate(velocity_milestones):
                sqred_radius = radius**2
                if (
                    (
                        dist_start_sqred > sqred_radius
                        or abs(dist_start_sqred - sqred_radius) < EPSILON
                    )
                    and start_circle_index is None
                    or dist_start_sqred == 0.0
                ):
                    start_circle_index = index
                if (
                    (
                        dist_end_sqred > sqred_radius
                        or abs(dist_end_sqred - sqred_radius) < EPSILON
                    )
                    and end_circle_index is None
                    or dist_end_sqred == 0.0
                ):
                    end_circle_index = index
                if use_min:
                    if (
                        (
                            dist_min_sqred > sqred_radius
                            or abs(dist_min_sqred - sqred_radius) < EPSILON
                        )
                        and mid_circle_index is None
                        or dist_min_sqred == 0.0
                    ):
                        mid_circle_index = index
                else:
                    if (
                        (
                            dist_midpoint_sqred > sqred_radius
                            or abs(dist_midpoint_sqred - sqred_radius) < EPSILON
                        )
                        and mid_circle_index is None
                        or dist_midpoint_sqred == 0.0
                    ):
                        mid_circle_index = index

            if (
                start_circle_index == mid_circle_index == end_circle_index
            ):  # if we don't cross a velocity milestone
                end_pos = move.end_pos
                if distance(move.end_pos, (0, 0)) < self.zero_crossing_radius:
                    logging.info("trying to move to 0! get outta here.")
                    intersections = get_circle_line_intersections(
                        move.start_pos, move.end_pos, self.zero_crossing_radius
                    )
                    if len(intersections) == 0:
                        logging.error("no intersections found!")
                    logging.info("intersections: %s", intersections)
                    logging.info("new end pos: %s", move.end_pos)
                    end_pos = (
                        intersections[-1][0],
                        intersections[-1][1],
                        move.end_pos[2],
                        move.end_pos[3],
                    )
                end_pos = (
                    round(end_pos[0], 14),
                    round(end_pos[1], 14),
                    round(end_pos[2], 14),
                    round(end_pos[3], 14),
                )
                start_pos = (
                    round(move.start_pos[0], 14),
                    round(move.start_pos[1], 14),
                    round(move.start_pos[2], 14),
                    round(move.start_pos[3], 14),
                )
                return ((start_pos, end_pos),)

            logging.info("start_circle_index: %s", start_circle_index)
            logging.info("velocity_milestones: %s", velocity_milestones)

            intersections = OrderedDict()
            indices_to_traverse = []
            if start_circle_index >= mid_circle_index >= end_circle_index:
                # 8 5 2 for example
                # moving from inside to outside
                indices_to_traverse = list(
                    range(start_circle_index, end_circle_index - 1, -1)
                )
            elif start_circle_index <= mid_circle_index <= end_circle_index:
                # 2 5 8 for example
                # moving from outside to inside
                indices_to_traverse = list(
                    range(start_circle_index, end_circle_index + 1)
                )
            elif mid_circle_index >= start_circle_index:
                indices_to_traverse = []
                # 4 8 2 for example.
                # moving past center, further from inside than outside
                for i in range(start_circle_index, mid_circle_index + 1):
                    indices_to_traverse.append(i)
                for i in range(mid_circle_index, end_circle_index - 1, -1):
                    indices_to_traverse.append(i)
                # we can dedupe because we an intersection calc will get both intersection points if 2 exist
                indices_to_traverse = list(set(indices_to_traverse))
            handled_zero = False
            for i in indices_to_traverse:
                radius = velocity_milestones[i]
                if radius not in intersections:
                    intersection_subset = get_circle_line_intersections(
                        move.start_pos, move.end_pos, radius
                    )
                    if i == len(velocity_milestones) - 1 and not handled_zero:
                        # we're in the zero radius
                        print("zero radius!")

                        if len(intersection_subset) == 2:

                            print("zero crossing with two intersections")
                            # moving through our zero radius, move 90 deg to incoming line
                            # if we know we're zero crossing, the angle to start and to end will always be pi (180deg) apart
                            # we want to find a point perpendicular to the line between start and end, at a distance of self.zero_crossing_radius
                            incoming_angle = math.atan2(
                                intersection_subset[0][1],
                                intersection_subset[0][0],
                            )
                            perpendicular_angle = incoming_angle + (math.pi / 2)
                            offset_position = polar_to_cartesian(
                                self.zero_crossing_radius, perpendicular_angle
                            )
                            intersections[radius] = (offset_position,)
                        elif len(intersection_subset) == 1:
                            # moving within our zero radius, stop at radius
                            intersections[radius] = (intersection_subset[0],)
                        else:
                            # somehow we found ourselves in the lowest radius, but we don't intersect with it?
                            logging.info(
                                "looking at smallest radius, but no intersection"
                            )
                        handled_zero = True
                        continue
                    if len(intersection_subset):
                        intersections[radius] = intersection_subset

            # intersections is an ordered dict, descending,
            # radius -> [closest_intersection, furthest_intersection]
            # we traverse by radius. if there are two intersections,
            #   grab [0] and put it at the end of a start list, then
            #   grab [1] and put it at the front of an end list
            # if there's only one, put it to the end of start
            # at the end, we'll put start + end together for the full list of points
            # flatten values of intersections
            flattened_intersections = []
            for radius, intersection_subset in intersections.items():
                flattened_intersections.extend(intersection_subset)
            total_intersections = flattened_intersections
            total_intersections = sorted(
                total_intersections,
                key=lambda x: sqrdistance(x, move.start_pos),
            )
            if (
                abs(move.start_pos[0] - total_intersections[0][0]) < EPSILON
                and abs(move.start_pos[1] - total_intersections[0][1]) < EPSILON
            ):
                total_intersections = [move.start_pos] + total_intersections[1:]
            else:
                total_intersections = [move.start_pos] + total_intersections
            if (
                abs(move.end_pos[0] - total_intersections[-1][0]) < EPSILON
                and abs(move.end_pos[1] - total_intersections[-1][1]) < EPSILON
            ):
                total_intersections = total_intersections[:-1] + [move.end_pos]
            else:
                total_intersections = total_intersections + [move.end_pos]

            # sort total intersections by distance from start
            if move.end_pos[0] == 0 and move.end_pos[1] == 0:
                total_intersections.pop(-1)
            if move.start_pos[0] == 0 and move.start_pos[1] == 0:
                total_intersections.pop(0)
            xy_moves = []
            while len(total_intersections) != 1:
                start = total_intersections.pop(0)
                end = total_intersections[0]
                xy_moves.append((start, end))
            total_move_dist = distance(move.start_pos, move.end_pos)
            total_z_dist = move.end_pos[2] - move.start_pos[2]
            total_e_dist = move.end_pos[3] - move.start_pos[3]
            actual_moves = []
            current_z_pos = move.start_pos[2]
            current_e_pos = move.start_pos[3]
            for move in xy_moves:
                move_dist = distance(move[0], move[1])
                z_dist = move_dist / total_move_dist * total_z_dist
                e_dist = move_dist / total_move_dist * total_e_dist
                new_z_pos = current_z_pos + z_dist
                new_e_pos = current_e_pos + e_dist
                actual_moves.append(
                    (
                        (
                            round(move[0][0], 14),
                            round(move[0][1], 14),
                            round(current_z_pos, 14),
                            round(current_e_pos, 14),
                        ),
                        (
                            round(move[1][0], 14),
                            round(move[1][1], 14),
                            round(new_z_pos, 14),
                            round(new_e_pos, 14),
                        ),
                    )
                )
                current_e_pos = new_e_pos
                current_z_pos = new_z_pos
            print(actual_moves)
            return actual_moves
        else:
            return []

    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0.0 else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            "homed_axes": xy_home + z_home,
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return PolarXZKinematics(toolhead, config)
