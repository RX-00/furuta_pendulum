#!/usr/bin/python3 -B

# script to move the furuta pendulum
# from max pos to min pos in an oscillating
# sequence of sinusoidal varying small velocity

import asyncio
import math
import moteus
import numpy as np
import time

async def main():
    print("moving moteus motor velocity...")
    while (True):
        now = time.time()
        vel = np.sin(now)

        # TODO: moving around
        # TODO: print aux encoder values

        # wait 20 ms b/w iterations
        # NOTE: by default over when cmd over CAN,
        #       there is a watchdog which requires cmds
        #       to be sent at least every 100ms or the ctrlr
        #       will enter a latched fault state
        await asyncio.sleep(0.02)



if __name__ == '__main__':
    asyncio.run(main())
