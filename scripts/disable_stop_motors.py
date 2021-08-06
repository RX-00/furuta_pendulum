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
    await motor.init()
    print("disable termination script finished")



if __name__ == '__main__':
    asyncio.run(main())
