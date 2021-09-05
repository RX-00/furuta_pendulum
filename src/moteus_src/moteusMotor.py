#!/usr/bin/python3 -B

# moteus motor controller class
# encoder readings are also from here
# the moteus controller is acting as the pwr, microcontroller, and foc ctrlr


import asyncio
import math
import moteus
import time
import argparse
import sys
import enum
import numpy as np
from . import multiplex as mp

'''
NOTE:
servopos.position_min = TBD
servopos.position_max = TBD

Distance travel maximum = TBD

Will use set_*() functions since there is only one moteus motor in the
furuta pendulum. No need for sending multiple commands in one transport
cycle (since there's only one moteus ctrlr).

TODO:
[ ] return [x, theta, xdot, thetadot] for furuta pendulum
    [ ] figure out velocity calc for thetadot

'''


class QueryResolutionABS:
    mode = mp.INT8
    position = mp.F32
    velocity = mp.F32
    torque = mp.F32
    q_current = mp.F32
    d_current = mp.F32
    abs_position = mp.F32
    rezero_state = mp.IGNORE
    voltage = mp.INT8
    temperature = mp.INT8
    fault = mp.INT8



# class to control moteus for the real world
# Furuta Pendulum
# responsible for getting sensor readings
# responsible for sending angular acceleration cmds (torque)
#             after converting the accelerations to angular
#             and then figuring out the Inertia of the motor
#             to get:
#                    Torque = Inertia * angular_accel
class moteusMotor:

    def __init__(self):
        #qr = QueryResolutionABS()
        # NOTE: proper way to set qr
        qr = moteus.QueryResolution()
        qr.abs_position = mp.F32
        self.actuator = moteus.Controller(id=1, query_resolution=qr)
        self.state = None
        self.input_u = 0
        self.pend_vel = 0
        # [motor pos, pend pos, motor vel, pend vel]
        self.state_vector = np.array([])
        self.state = None

    async def set_state_vector(self):
        self.state = await self.actuator.query()
        self.state_vector = np.array([
            self.state.values[moteus.Register.POSITION],
            self.state.values[moteus.Register.ABS_POSITION],
            self.state.values[moteus.Register.VELOCITY],
            self.calc_enc_vel(self.state.values[moteus.Register.ABS_POSITION])
        ])

    async def stop(self):
        # NOTE: - moteus.Controller.make_*() functions must be manually sent
        #       through a transport cycle. Allowing multiple make_ fxn cmds
        #       from multiple moteus controllers be sent at the same time.
        #       - moteus.Controller.set_*() functions are sent through a
        #       transport cycle automatically by itself
        self.state = await self.actuator.set_stop()
        return self.state


    async def clear_faults(self):
        print("moteus init")
        # clearing any faults
        await self.stop()


    async def get_pos(self):
        self.state = await self.actuator.query()
        return self.state.values[moteus.Register.POSITION]


    async def get_vel(self):
        self.state = await self.actuator.query()
        return self.state.values[moteus.Register.VELOCITY]


    async def get_aux_enc_raw(self):
        # return the auxiliary encoder readings
        self.state = await self.actuator.query()
        return self.state.values[moteus.Register.ABS_POSITION]

    async def get_aux_enc_filt(self):
        pass


    def calc_enc_vel(self, pos):

        pass

    async def get_aux_vel(self):
        pos = self.get_aux_enc()
        self.calc_enc_vel(pos)
        # TODO: read Josh's blog about velocity calculations


    async def move_torque(self, max_torque, ffd_torque=-0.01):
        self.state = await self.actuator.set_position(
            position = math.nan,
            maximum_torque = max_torque,
            feedforward_torque = ffd_torque,
            query = True)
        # TODO: return [x, theta, xdot, thetadot] for furuta pendulum here
        # TODO: also have calc_env_vel here


    async def move_to_pos(self, pos, vel=0.2, max_torque=0.2, ffd_torque=-0.01):
        '''
        NOTE: about stop_position in set/make_position()
              Through the use of the optional "stop_position" of the position controller,
              moteus can set the velocity to zero when a specific control position is
              achieved. This feature can be used in applications where the host commands
              the controller at a low update rate.
        '''
        # move motor to max position/rotation cw
        self.state = await self.actuator.set_position(
            position = pos, # this is where the position tries to hold itself in
            velocity = vel,
            maximum_torque = max_torque,
            stop_position = math.nan,
            feedforward_torque = ffd_torque,
            watchdog_timeout = math.nan,
            query = True)




async def main():
    print("disabling moteus motor...")
    motor = moteusMotor()
    await motor.clear_faults()
    #motor.state = await motor.actuator.set_position(
    #    position = math.nan,
    #    velocity = 0.0,
    #    maximum_torque = 0.0,
    #    stop_position = math.nan,
    #    feedforward_torque = -0.01,
    #    watchdog_timeout = math.nan,
    #    query = True)
    #print(motor.state.values[moteus.Register.ABS_POSITION])
    print(await motor.get_aux_enc())

    print("disable termination script finished")



#if __name__ == '__main__':
#    asyncio.run(main())
