#!/usr/bin/python3 -B

# stop command sent to motors thus disabling the motors
# for the single moteus & motor furuta pendulum


from moteusMotor import moteusMotor

import asyncio
import math
import moteus


async def main():
    print("disabling moteus motor...")
    motor = moteusMotor()
    await motor.clear_faults()

    '''
    motor.state = await motor.actuator.set_position(
        position = math.nan,
        velocity = 0.0,
        maximum_torque = 0.0,
        stop_position = math.nan,
        feedforward_torque = -0.01,
        watchdog_timeout = math.nan,
        query = True)
    print(motor.state.values[moteus.Register.ABS_POSITION])
    '''
    #print(await motor.get_aux_enc())

    print("disable termination script finished")



if __name__ == '__main__':
    asyncio.run(main())
