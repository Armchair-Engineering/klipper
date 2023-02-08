#include <math.h>      // sqrt
#include <stdlib.h>    // malloc
#include <string.h>    // memset
#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h"     // move_get_coord

static double
polarbed_stepper_angle_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    double angle = atan2(c.y, c.x);
    if (angle == 0)
        angle = M_PI;
    return angle;
}

struct stepper_kinematics *__visible
polarbed_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    sk->calc_position_cb = polarbed_stepper_angle_calc_position;
    sk->active_flags = AF_X | AF_Y;

    return sk;
}
