#!/usr/bin/python3 -B

# script to move the furuta pendulum
# from max pos to min pos in an oscillating
# sequence of sinusoidal varying small velocity

from moteusMotor import moteusMotor
import asyncio
import math
import moteus
import numpy as np
import time


async def main():
    print("moving moteus motor velocity...")

    motor = moteusMotor()
    await motor.clear_faults()

    while (True):
        now = time.time()
        vel = np.sin(now)

        motor.state = await motor.actuator.set_position(
            position = math.nan,
            velocity = 0.2 * np.sin(now),
            maximum_torque = 0.2,
            stop_position = math.nan,
            feedforward_torque = -0.01,
            watchdog_timeout = math.nan,
            query = True)

        print("mtr pos: ", motor.state.values[moteus.Register.POSITION])
        print("aux enc: ", motor.state.values[moteus.Register.ABS_POSITION])

        # wait 20 ms b/w iterations
        # NOTE: by default over when cmd over CAN,
        #       there is a watchdog which requires cmds
        #       to be sent at least every 100ms or the ctrlr
        #       will enter a latched fault state
        await asyncio.sleep(0.02)



if __name__ == '__main__':
    asyncio.run(main())
