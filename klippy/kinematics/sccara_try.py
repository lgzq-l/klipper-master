# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper

class Scarakinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        rail_arm1 = stepper.PrinterRail(config.getsection('stepper_arm1'))
        rail_arm2 = stepper.PrinterRail(config.getsection('stepper_arm2'))
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        
        arm = self.arm = [None] * 2
        arm[0] = stepper_configs[0].getfloat('arm_1_length')
        arm[1] = stepper_configs[1].getfloat('arm_2_length')

        rail_arm1.setup_itersolve('scara_stepper_alloc', b'r1')
        rail_arm1.setup_itersolve('scara_stepper_alloc', b'r2')
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails = [rail_arm1, rail_arm2,rail_z]
        self.steppers =[ s for r in self.rails
                                          for s in r.get_steppers() ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.)

    def get_steppers(self):
        #4
    def calc_position(self, stepper_positions):
        #通过各个步进电机的位置计算笛卡尔坐标系下的打印头位置；同时该方法通常只在回零和z探测时使用，因此无需过分追求效率。（正运动学）
        angle1 = stepper_positions[self.steppers[0].get_name()]
        angle2 = stepper_positions[self.steppers[1].get_name()]
        z_pos=stepper_positions[self.steppers[2].get_name()]
        x=self.arm[0]*math.cos(angle1)+self.arm[1]*math.cos(angle1+angle2)
        y=self.arm[0]*math.sin(angle1)+self.arm[1]*math.sin(angle1+angle2)
        return[x,y,z_pos]
    
    def set_position(self, newpos, homing_axes):
        #6
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        
    def home(self, homing_state):
        #5
    def _motor_off(self, print_time):
        
    def check_move(self, move):
        #2
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        #3
    

def load_kinematics(toolhead, config):
    return Scarakinematics(toolhead, config)
