#!/usr/bin/python

import rospy
from simulator.msg import Observation
from simulator.msg import Landmark

from simulation import Simulation
import numpy as np
import math


class Node:
    def __init__(self):

        #initialize ros node
        rospy.init_node('Simulator')
        self.pub = rospy.Publisher('landmark_observations', Observation)

        #initialize simulator
        self.sim = Simulation(
            self.publish_observation,
            20, (500, 500),
            start_param=(250, 250, math.pi / 2),
            magnitude_stddev=0,
            argument_stddev=0)

    def run(self):
        self.sim.run()

    def publish_observation(self, observation):
        # observation -> array of landmarks
        # landmark -> [index, x, y]
        landmarks = []
        for l in observation:
            index = np.int32(l[0])
            r = np.float64(l[1])
            theta = np.float64(l[2])
            landmarks.append(Landmark(index=index, r=r, theta=theta))

        self.pub.publish(Observation(landmarks=landmarks))


if __name__ == '__main__':
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
