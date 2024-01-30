#!/usr/bin/env python

import numpy as np
import random
from qpsolvers import solve_qp
from pycrazyswarm import *
import matplotlib.pyplot as plt

Z = 1.0
# sleepRate = 30
sleepRate = 30
ds = 1.0
kPosition = 1.0


def goAvoidCircle(timeHelper, cflist):
    """
    crazyflies go in random circles. maintain safety based on barrier function
    """
    N = len(cflist)
    startTime = timeHelper.time()
    radii = [random.random() * 2.5 + .5 for _ in range(N)] # range of numbers is 0.5 to 3
    times = [random.random() * 8 + 2 for _ in range(N)] # range of numbers is 2 to 10
    # radii = [1, 1, 1, 1]
    # times = [4, 4, 4, 4]
    p = np.eye(N*3)
    ctrs = np.array([cf.initialPosition - np.array([radii[i], 0, -Z]) for i, cf in enumerate(cflist)])
    # print(ctrs)
    revs = [-1 if random.randint(0, 1) == 0 else 1 for _ in range(N)]

    while True:
        plt_pos = np.array([])
        plt_error = np.array([])
        plt_verror = np.array([])
        pairs = N * (N - 1) // 2
        h = np.array([0.0 for _ in range(pairs)]) 
        G = np.array([[0.0 for _ in range(N * 3)] for __ in range(pairs)]) # 
        q = np.array([0.0 for i in range(N*3)])

        # command velocities for each crazyflie
        for i, cf in enumerate(cflist):
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / times[i]
            vx = -radii[i] * omega * np.sin(omega * time) # * revs[i]
            vy = radii[i] * omega * np.cos(omega * time) # * revs[i]
            desiredPos = ctrs[i] + radii[i] * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            plt_pos = np.append(plt_pos, cf.position())
            plt_error = np.append(plt_error, np.linalg.norm(errorX))
            q[3*i] = vx + kPosition * errorX[0]
            q[3*i + 1] = vy + kPosition * errorX[1]
            q[3*i + 2] = 0 + kPosition * errorX[2]
        # print("q: ", q)

        

        row = 0
        for i in range(N):
            for j in range(i + 1, N):
                posi = cflist[i].position()
                posj = cflist[j].position()
                diff = posi - posj
                h[row] = np.linalg.norm(diff) ** 2 - ds ** 2
                # make G by position vector
                for k in range(3):
                    G[row][3*i + k] = posi[k] - posj[k]
                    G[row][3*j + k] = posj[k] - posi[k]
                row += 1
        # G = 0*G
        # h = 0*h
        x = solve_qp(p, -q, -2 * G, h, solver="daqp")
        # print("x: ", x)
        

        # setting the velocities for each crazyflie
        for i, cf in enumerate(cflist):
            cf.cmdVelocityWorld(np.array([x[3*i], x[3*i+1], x[3*i+2]]), yawRate=0)
            plt_verror = np.append(plt_verror, np.linalg.norm(np.array([x[3*i+k] - q[3*i+k] for k in range(3)])))
            # cf.cmdVelocityWorld(np.array([float(q[3*i]), float(q[3*i+1]), float(q[3*i+2])]), yawRate=0)
        
        # print("pos", plt_pos)
        # print("error", plt_error)
        update_plots(plt_pos, plt_error, plt_verror, timeHelper.time())
        
        timeHelper.sleepForRate(sleepRate // 3)

        if timeHelper.time() > 20:
            plot_show(N)
            break

safety_data = {
    'time': []
}

error_data = {
    'time': []
}

verror_data = {
    'time': []
}

def init_data(cflist):
    """
    initializes the data for error and safety
    """
    N = len(cflist)

    # initialize error, verror data
    for i in range(N):
        error_data[f'{i}'] = []
        verror_data[f'{i}'] = []

    # initialize safety data
    for i in range(N):
        for j in range(i + 1, N):
            safety_data[f'{i}{j}'] = []

def update_plots(pos, error, verror, time):
    """
    updates the plots with new data on error / safety.
    params: position, desired position of each crazyflie, as well as the time 
    that these data points were taken. should only be called in goAvoidCircle
    """
    # print("updating")
    N = len(pos) // 3
    
    newpos = []
    for i in range(N):
        newpos.append(np.array([pos[3 * i + k] for k in range(3)]))

    # update error data
    error_data['time'].append(time)
    for i in range(N):
        error_data[f'{i}'].append(error)
    
    # update safety data
    safety_data['time'].append(time)
    for i in range(N):
        for j in range(i + 1, N):
            safety_data[f'{i}{j}'].append(np.linalg.norm(newpos[i] - newpos[j]))

    # update velocity error data
    verror_data['time'].append(time)
    for i in range(N):
        verror_data[f'{i}'].append(verror)

def plot_show(N):
    print("plotting")

    fig, ax = plt.subplots(1, 3)
    for i in range(N):
        ax[0].plot(error_data['time'], error_data[f'{i}'])
    ax[0].set_title('Error')

    for i in range(N):
        for j in range(i + 1, N):
            ax[1].plot(safety_data['time'], safety_data[f'{i}{j}'])
    ax[1].set_title('Safety')
    ax[1].axhline(y=ds, linestyle = "--")

    for i in range(N):
        ax[2].plot(verror_data['time'], verror_data[f'{i}'])
    ax[2].set_title('Velocity Error')

    plt.show()


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    init_data(allcfs.crazyflies)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goAvoidCircle(timeHelper, allcfs.crazyflies)
    # goCircle(timeHelper, allcfs.crazyflies, kPosition=1)
