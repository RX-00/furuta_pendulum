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
import multiplex as mp

'''
NOTE:
servopos.position_min = TBD
servopos.position_max = TBD

Distance travel maximum = TBD

Will use set_*() functions since there is only one moteus motor in the
furuta pendulum. No need for sending multiple commands in one transport
cycle (since there's only one moteus ctrlr).
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


class moteusMotor:

    def __init__(self):
        qr = QueryResolutionABS()
        self.actuator = moteus.Controller(id=1, query_resolution=qr)
        self.state = None


    async def stop(self):
        # NOTE: moteus.Controller.make_*() functions must be manually sent
        #       through a transport cycle. Allowing multiple make_ fxn cmds
        #       from multiple moteus controllers be sent at the same time.
        #
        #       moteus.Controller.set_*() functions are sent through a
        #       transport cycle automatically by itself
        self.state = await self.actuator.set_stop()
        return self.state


    async def clear_faults(self):
        print("moteus init")
        # clearing any faults
        await self.stop()


    async def mv_max_pos(self):
        '''
        NOTE: about stop_position in set/make_position()
              Through the use of the optional "stop_position" of the position controller,
              moteus can set the velocity to zero when a specific control position is
              achieved. This feature can be used in applications where the host commands
              the controller at a low update rate.
        '''
        # move motor to max position/rotation cw
        self.state = await self.actuator.set_position(
            position = math.nan, # this is where the position tries to hold itself in
            velocity = 0.2,
            maximum_torque = 0.2,
            stop_position = math.nan,
            feedforward_torque = -0.01,
            watchdog_timeout = math.nan,
            query = True)


    async def mv_min_pos(self):
        # move motor to min position/rotation ccw
        self.state = await self.actuator.set_position(
            position = math.nan,
            velocity = 0.2,
            maximum_torque = 0.2,
            stop_position = math.nan,
            feedforward_torque = -0.01,
            watchdog_timeout = math.nan,
            query = True)


    async def mv_mid_pos(self):
        # move motor to center position (pendulum in front of base weight compartment)
        self.state = await self.actuator.set_position(
            position = math.nan,
            velocity = 0.2,
            maximum_torque = 0.2,
            stop_position = math.nan,
            feedforward_torque = -0.01,
            watchdog_timeout = math.nan,
            query = True)


    async def get_aux_enc(self):
        # return the auxiliary encoder readings
        self.state = await self.actuator.set_position(
            position = math.nan,
            velocity = 0.0,
            maximum_torque = 0.0,
            stop_position = math.nan,
            feedforward_torque = 0.0,
            query = True)
        return self.state.values[moteus.Register.ABS_POSITION]



#async def main():
#    print("disabling moteus motor...")
#    motor = moteusMotor()
#    await motor.clear_faults()

#    motor.state = await motor.actuator.set_position(
#        position = math.nan,
#        velocity = 0.0,
#        maximum_torque = 0.0,
#        stop_position = math.nan,
#        feedforward_torque = -0.01,
#        watchdog_timeout = math.nan,
#        query = True)
#    #print(motor.state.values[moteus.Register.ABS_POSITION])
#    print(await motor.get_aux_enc())

#    print("disable termination script finished")



#if __name__ == '__main__':
#    asyncio.run(main())
