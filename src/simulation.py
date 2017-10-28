import numpy as np
import math, cmath
from renderer import Renderer


class Simulation:
    def __init__(self,
                 ros_callback,
                 landmarks,
                 map_dims,
                 start_param=(0, 0, math.pi / 2),
                 sonar_min_range=20,
                 sonar_max_range=100,
                 sonar_sweep_angle=math.pi * 2 / 3,
                 magnitude_stddev=0,
                 argument_stddev=0,
                 miss_prob=0):
        self.sonar_min_range = sonar_min_range
        self.sonar_max_range = sonar_max_range
        self.sonar_sweep_angle = sonar_sweep_angle
        self.miss_prob = miss_prob
        self.height, self.width = map_dims
        self.magnitude_stddev = magnitude_stddev
        self.argument_stddev = argument_stddev
        self.ros_callback = ros_callback
        assert start_param[0] >= 0 and start_param[0] <= self.height
        assert start_param[1] >= 0 and start_param[1] <= self.width

        # generate random landmarks
        # landmarks have the form [index, x, y]
        if isinstance(landmarks, int):
            self.generate_random_landmarks(landmarks)
        else:
            assert landmarks[:, 1].max() <= self.height
            assert landmarks[:, 2].max() <= self.width
            assert landmarks[:, 1].min() >= 0
            assert landmarks[:, 2].min() >= 0
            self.landmarks = landmarks

        self.auv_pos = list(start_param)
        self.renderer = Renderer(
            map_dims,
            self.sonar_min_range,
            self.sonar_max_range,
            self.sonar_sweep_angle,
            self.move,
            start_param,
            self.landmarks)

    def run(self):
        self.renderer.start()
        self.move(0, 0, 0)  # taking initial observation
        while True:
            self.renderer.update_auv_pos(*self.auv_pos)
            self.renderer.render()

    def add_gaussian_noise(self, measurement, std):
        noise = np.random.normal(0, std)
        return measurement + noise

    # move in intrinsic coordinates
    def move(self, down, right, clockwise):

        dx = math.sin(self.auv_pos[2]) * right - math.cos(
            self.auv_pos[2]) * down
        dy = math.sin(self.auv_pos[2]) * down + math.cos(
            self.auv_pos[2]) * right

        self.auv_pos[0] += dx
        self.auv_pos[1] += dy
        self.auv_pos[2] += clockwise

        observed_landmarks = self.observe()

        if len(observed_landmarks) > 0:
            # give the indices of landmarks to update
            self.renderer.update_observed_landmarks(observed_landmarks[:, 0])

            # publish to ROS
            self.ros_callback(observed_landmarks)
        else:
            self.renderer.update_observed_landmarks(np.array([]))

    def generate_random_landmarks(self, n_landmarks):
        landmarks = []
        for i in range(n_landmarks):
            x = np.random.rand() * self.height
            y = np.random.rand() * self.width
            landmarks.append([int(i), x, y])
        self.landmarks = np.array(landmarks)

    def normalize_angle(self, angle):
        if angle < 0:
            return self.normalize_angle(angle + math.pi * 2)
        elif angle > math.pi * 2:
            return self.normalize_angle(angle - math.pi * 2)
        return angle

    # returns an array of observations,
    # each element is a landmark [relative_x_pos, relative_y_pos, landmark_index]
    def observe(self):

        # normalize angles to 0 to 2pi
        bearing_max = self.normalize_angle(self.auv_pos[2] +
                                           self.sonar_sweep_angle / 2)
        bearing_min = self.normalize_angle(self.auv_pos[2] -
                                           self.sonar_sweep_angle / 2)

        observations = []

        for lm in self.landmarks:

            index = lm[0]
            dx = lm[1] - self.auv_pos[0]
            dy = self.auv_pos[1] - lm[2]

            displacement = cmath.polar(complex(dx, dy))
            mag, arg = displacement  # mag from -pi to pi
            arg = self.normalize_angle(arg)

            # check if distance within range
            if mag <= self.sonar_max_range and mag >= self.sonar_min_range:

                angle_from_min_bearing = self.normalize_angle(
                    arg - bearing_min)

                if angle_from_min_bearing <= self.sonar_sweep_angle and np.random.rand(
                ) > self.miss_prob:
                    if self.magnitude_stddev > 0:
                        mag = self.add_gaussian_noise(mag, self.magnitude_stddev)
                    if self.argument_stddev > 0:
                        arg = self.add_gaussian_noise(arg, self.argument_stddev)
                    # calculate the bearing relative to the heading of the AUV
                    # Anti-clockwise
                    angle_rel_heading = self.normalize_angle(arg - self.auv_pos[2])
                    observations.append([index, mag, angle_rel_heading])

        return np.array(observations)
