#!/usr/bin/env python

import numpy as np
import random
from pycrazyswarm import *


Z = 1.0
sleepRate = 30


def goCircle(timeHelper, cflist, kPosition):
    startTime = timeHelper.time()
    radii = [random.random() * 2.5 + .5 for _ in range(4)] # range of numbers is 0.5 to 3
    times = [random.random() * 8 + 2 for _ in range(4)] # range of numbers is 2 to 10
    # radii = [1, 1, 1, 1]
    # times = [4, 4, 4, 4]
    revs = [-1 if random.randint(0, 1) == 0 else 1 for _ in range(4)]
    while True: 
        for i, cf in enumerate(cflist):
            radius = radii[i]
            totalTime = times[i]
            rev = revs[i]

            startPos = cf.initialPosition + np.array([0, 0, Z])
            center_circle = startPos - np.array([radius, 0, 0])
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time) * rev
            vy = radius * omega * np.cos(omega * time) * rev
            desiredPos = center_circle + rev * radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
            timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies, kPosition=1)
