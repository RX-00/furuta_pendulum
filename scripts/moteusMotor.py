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

'''
NOTE:
servopos.position_min = TBD
servopos.position_max = TBD

Distance travel maximum = TBD

Will use set_*() functions since there is only one moteus motor in the
furuta pendulum. No need for sending multiple commands in one transport
cycle (since there's only one moteus ctrlr).
'''

class moteusMotor:
    def __init__(self):
        qr = moteus.QueryResolution() # default query resolution of F32
        self.actuator = moteus.Controller(id = 1)
        self.state = None

    async def stop(self):
        # NOTE: moteus.Controller.make_*() functions must be manually sent
        #       through a transport cycle. Allowing multiple make_ fxn cmds
        #       from multiple moteus controllers be sent at the same time.
        #
        #       moteus.Controller.set_*() functions are sent through a
        #       transport cycle automatically by itself
        await self.actuator.set_stop()

    async def init(self):
        print("moteus init")
        # clearing any faults
        await self.stop()

    async def mv_max_pos(self):
        # move motor to max position/rotation cw
        self.state = await self.actuator.set_position()

    async def mv_min_pos(self):
        # move motor to min position/rotation ccw
        self.state = await self.actuator.set_position()

    async def mv_mid_pos(self):
        # move motor to center position (pendulum in front of base weight compartment)
        self.state = await self.actuator.set_position()

    async def get_aux_enc(self):
        # return the auxiliary encoder readings
        self.state = # TODO: figure out how to get aux enc reading

    
