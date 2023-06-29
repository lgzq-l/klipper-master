// Scara kinematics stepper pulse time generation
//
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

static double
scara_stepper_angle1_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    c_2=(c.x*c.x+ c.y*c.y-L_1*L_1-L_2*L_2)/2*L_1*L_2
    s_2=sqrt(1-c_2*c_2) //右手系
    double angle= atan2(c.y,c.x)-atan2(L_2*s_2,L_1+L_2*c_2)
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2. * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2. * M_PI;
    return angle;
}

static double
scara_stepper_angle2_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    // XXX - handle x==y==0
    c_2=(c.x*c.x+ c.y*c.y-L_1*L_1-L_2*L_2)/2*L_1*L_2
    s_2=sqrt(1-c_2*c_2) //右手系
    double angle = atan2(s_2, c_2);
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2. * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2. * M_PI;
    return angle;
}

static void
scara_stepper_angle_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the stepper_bed angle
    if (sk->commanded_pos < -M_PI)
        sk->commanded_pos += 2 * M_PI;
    else if (sk->commanded_pos > M_PI)
        sk->commanded_pos -= 2 * M_PI;
}

struct stepper_kinematics * __visible
scara_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (type == 'a1') {
        sk->calc_position_cb = polar_stepper_angle1_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    } else if (type == 'a2') {
        sk->calc_position_cb = polar_stepper_angle2_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    }
    sk->active_flags = AF_X | AF_Y;
    return sk;
}
