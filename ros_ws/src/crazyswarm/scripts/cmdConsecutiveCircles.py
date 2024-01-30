#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


Z = 1.0
sleepRate = 30


def goCircle(timeHelper, cfs, totalTime, radius, kPosition):
    for cf in cfs.crazyflies:
        startTime = timeHelper.time()
        pos = cf.position()
        startPos = cf.initialPosition + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        time = 0
        while time < totalTime:
            time = timeHelper.time() - startTime 
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
            timeHelper.sleepForRate(sleepRate)
        cf.cmdVelocityWorld([0, 0, 0], yawRate=0)
    # timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs, totalTime=4, radius=1, kPosition=1)