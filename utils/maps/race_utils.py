import math
import numpy as np
import pygame

import sys
import os
current_script_directory = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.abspath(os.path.join(current_script_directory, os.pardir))
sys.path.append(parent_directory)


class RaceTrack(object):

    def __init__(self, width, height, POINT_RADIUS, screen, POINT_COLOR, LINE_COLOR, ROAD_THICKNESS, seed):
        self.width = width
        self.height = height
        self.POINT_RADIUS = POINT_RADIUS
        self.path_points = []
        self.left_cones = []
        self.right_cones = []
        self.screen = screen
        self.POINT_COLOR = POINT_COLOR
        self.LINE_COLOR = LINE_COLOR
        self.YELLOW = (226, 221, 36)
        self.BLUE = (0, 171, 197)
        self.RED = (255, 51, 153)
        # self.STARTCOLOR = (128,245,0)
        self.STARTCOLOR = self.RED
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        self.ROAD_THICKNESS = ROAD_THICKNESS
        self.start_directon = (1, 0)  # arbitrary value, should be overwritten when generating course
        self.start_directon_scalar = 15

        np.random.seed(seed)

    def get_start(self):
        return self.path_points[0]

    def draw_start(self):
        if len(self.path_points) > 0:
            pygame.draw.circle(self.screen,
                               self.STARTCOLOR,
                               self.path_points[0],
                               self.POINT_RADIUS
                               )

    def draw_start_directon(self):
        if len(self.path_points) > 0:
            pygame.draw.line(self.screen,
                             self.RED,
                             self.path_points[0],
                             self.path_points[0] + self.start_directon,
                             2
                             )

    # Function to generate random points
    def generate_random_points(self, number_of_points):
        self.path_points = []
        for _ in range(number_of_points):  # Generate 10 random points
            x = np.random.randint(self.POINT_RADIUS, self.width - self.POINT_RADIUS)
            y = np.random.randint(self.POINT_RADIUS, self.height - self.POINT_RADIUS)
            self.path_points.append((x, y))
        return self.path_points

    # THIS IS UNUSED!
    # generates a course clockwise. (samples a random radius for every angle, and returns it)
    def generate_points_counterclockwise(self, number_of_points):
        max_radius = min(self.center_x, self.center_y)

        self.path_points = []
        increment_angle = 360//number_of_points
        for theta in range(0, 360, increment_angle):  # Generate points for angles from 0 to 360 degrees
            radius = np.random.randint(self.ROAD_THICKNESS, max_radius - self.ROAD_THICKNESS)
            x = self.center_x + radius * math.cos(math.radians(theta))
            y = self.center_y + radius * math.sin(math.radians(theta))
            x = int(x)
            y = int(y)
            self.path_points.append((x, y))
        # print(points)
        return self.path_points

    def generate_random_angles(x):
        # Generate x-1 random angles between 0 and 360
        angles = [np.random.uniform(0, 360) for _ in range(x - 1)]

        # Sort the angles
        angles.sort()

        return angles

    def generate_race_course_midpath(self, number_of_points):
        max_radius = min(self.center_x, self.center_y)

        self.path_points = []
        min_angle_increment = 200//number_of_points  # Minimum angle increment between points
        max_angle_increment = 360//number_of_points  # Maximum angle increment between points

        current_angle = 0
        for _ in range(number_of_points):
            radius = np.random.randint(self.ROAD_THICKNESS + max_radius//4, max_radius - self.ROAD_THICKNESS)
            x = self.center_x + radius * math.cos(math.radians(current_angle))
            y = self.center_y + radius * math.sin(math.radians(current_angle))
            x = int(x)
            y = int(y)
            self.path_points.append((x, y))

            # Randomly adjust the current angle
            angle_increment = np.random.randint(min_angle_increment, max_angle_increment)
            current_angle += angle_increment

        self.start_directon = (self.path_points[1][0] - self.path_points[0][0], self.path_points[1][1] - self.path_points[0][1])
        self.start_directon /= np.linalg.norm(self.start_directon)
        self.start_directon *= self.start_directon_scalar
        return self.path_points

    def generate_left_and_right_cones(self):

        if len(self.path_points) <= 0:
            # print(f"invalid number of path points: {self.path_points}")
            return None

        self.left_cones = []
        self.right_cones = []

        # Iterate through each line segment in the road
        for i in range(len(self.path_points)):
            x1, y1 = self.path_points[i]
            x2, y2 = self.path_points[(i + 1) % len(self.path_points)]
            x3, y3 = self.path_points[(i + 2) % len(self.path_points)]

            # Calculate the direction vector of the line segment
            direction1 = np.array([x1 - x2, y1 - y2]).astype(np.float64)
            direction2 = np.array([x3 - x2, y3 - y2]).astype(np.float64)
            direction1 /= np.linalg.norm(direction1)
            direction2 /= np.linalg.norm(direction2)

            # Calculate the bisecting vector (perpendicular to the direction vector)
            bisecting_vector = (direction1 + direction2).astype(np.float64)

            # Normalize the bisecting vector
            bisecting_vector /= np.linalg.norm(bisecting_vector)

            # Calculate the offset vectors for left and right boundaries
            offset = bisecting_vector * (self.ROAD_THICKNESS / 2)

            # Calculate left and right boundary points as NumPy arrays
            left_point = np.array([int(x2 + offset[0]), int(y2 + offset[1])])
            right_point = np.array([int(x2 - offset[0]), int(y2 - offset[1])])

            # Calculate Euclidean distances using np.linalg.norm
            left_distance = np.linalg.norm(left_point - np.array([self.center_x, self.center_y]))
            right_distance = np.linalg.norm(right_point - np.array([self.center_x, self.center_y]))

            # Compare distances
            if left_distance > right_distance:
                # swap because cones are inside out
                temp = left_point
                left_point = right_point
                right_point = temp

            self.left_cones.append(left_point)
            self.right_cones.append(right_point)

        return self.left_cones, self.right_cones

    def get_widths(self):
        if len(self.left_cones) != len(self.right_cones):
            raise ValueError("Number of left and right cones must be the same.")

        num_cones = len(self.left_cones)
        widths = np.zeros(num_cones)

        for i in range(num_cones):
            # Calculate the Euclidean distance between left and right cones
            width = np.linalg.norm(self.right_cones[i] - self.left_cones[i])
            widths[i] = width

        return widths

    def get_midpoints(self):
        return self.path_points

    def draw_midpoints(self):
        # Draw the points
        for point in self.path_points:
            pygame.draw.circle(self.screen, self.POINT_COLOR, point, self.POINT_RADIUS)

        # Draw the path
        if len(self.path_points) >= 2:
            pygame.draw.lines(self.screen, self.LINE_COLOR, False, self.path_points, 2)

        # pygame.display.flip()

    def draw_yellow_cones(self):
        # Draw the path
        if len(self.right_cones) >= 2:
            pygame.draw.lines(self.screen, self.YELLOW, True, self.right_cones, 2)

        # Draw the points
        for point in self.right_cones:
            pygame.draw.circle(self.screen, self.YELLOW, point, self.POINT_RADIUS)

        # pygame.display.flip()

    def draw_path(self, path, velocities):
        # Draw the path
        if len(path) >= 2:
            pygame.draw.lines(self.screen, (30, 225, 112), False, path, 2)

        # Draw the points
        for i in range(len(path)):
            pygame.draw.circle(self.screen, (0, velocities[i]*9, 0), path[i], self.POINT_RADIUS)

    def draw_blue_cones(self):
        # Draw the path
        if len(self.left_cones) >= 2:
            pygame.draw.lines(self.screen, self.BLUE, True, self.left_cones, 2)
        # pygame.draw.line(self.screen, self.LINE_COLOR, , 2)

        # Draw the points
        for point in self.left_cones:
            pygame.draw.circle(self.screen, self.BLUE, point, self.POINT_RADIUS)

        # pygame.display.flip()

    def flip(self):
        pygame.display.flip()










