#include <math.h> // sqrt
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

static double
polarxz_stepper_angle_calc_position(struct stepper_kinematics *sk
                                    , struct move *m, double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    // XXX - handle x==y==0
    double angle = atan2(c.y, c.x);
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2.f * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2.f * M_PI;
    return angle;
}

static void
polarxz_stepper_angle_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the stepper_bed angle
    if (sk->commanded_pos < -M_PI)
        sk->commanded_pos += 2 * M_PI;
    else if (sk->commanded_pos > M_PI)
        sk->commanded_pos -= 2 * M_PI;
}

struct stepper_kinematics * __visible
polarxz_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (type == 'a') {
        sk->calc_position_cb = polarxz_stepper_angle_calc_position;
        sk->post_cb = polarxz_stepper_angle_post_fixup;
        sk->active_flags = AF_X | AF_Y;
    }
    
    return sk;
}
