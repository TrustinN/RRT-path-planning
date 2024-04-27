import time
import numpy as np
from utils.maps.map_utils import get_angle
from PyQt6.QtGui import QVector3D


class Camera():

    def set_view(self, view):
        self.view = view

    def connect(self, slider, button):
        self.slider = slider
        self.play_button = button

    def reset(self):
        self.slider.setValue(50)
        self.view.setCameraPosition(pos=QVector3D(400, 400, 400), elevation=0, azimuth=-35, distance=1200)
        self.play_button.setChecked(False)

    def hide_slider(self):
        self.slider.hide()

    def show_slider(self):
        self.slider.show()

    def animate_path(self, path):
        if self.play_button.isChecked:
            while self.slider.sliderPosition() < 100:
                self.slider.setValue(self.slider.sliderPosition() + 1)
                time.sleep(.1)

    def follow_path(self, path):
        percent = self.slider.sliderPosition() / 100
        path_lengths = [np.linalg.norm(path[i] - path[i + 1]) for i in range(len(path) - 1)]
        path_length = sum(path_lengths)

        if path:
            dist = percent * path_length
            curr_dist = 0
            iter = 0

            while True:
                curr_dist += path_lengths[iter]
                if curr_dist >= dist:
                    curr_dist -= path_lengths[iter]
                    break
                iter += 1

            start = path[iter]
            end = path[iter + 1]
            vec = end - start

            vec_dist = np.linalg.norm(vec)
            p = (dist - curr_dist) / vec_dist

            c_pos = p * vec + start

            # calculate the rotation needed for camera from x-axis
            # to be positioned behind the current vector, vec
            angle = get_angle(vec[:2], np.array([1, 0]))
            azi = 180 * (angle) / np.pi

            # We need to keep track of if the angle given is a counter clockwise
            # or clockwise rotation from x-axis
            sign = np.sign(np.dot(vec[:2], np.array([0, 1])))

            # Move to oppositie side of the unit circle
            azi = azi * sign + 180

            # calculate the elevation angle. This is the displacement angle
            # from the negative vector on the x-axis on the xz-plane

            # Spin the vector onto the xz-plane
            rot = np.array([[np.cos(-angle), -np.sin(-angle), 0],
                           [np.sin(-angle), np.cos(-angle), 0],
                           [0, 0, 1]])
            n_p = np.dot(rot, vec)

            elev = get_angle(np.array([n_p[0], n_p[2]]), np.array([1, 0]))
            sign = np.sign(np.dot(np.array([n_p[0], n_p[2]]), np.array([0, 1])))

            # Again, calculate the clockwise/counterclockwise position of the angle
            elev = -sign * elev * 180 / np.pi

            # We want to view the path from slightly above
            # If the vector points in the negative x-direction, we want to go clockwise, 
            # subtract from current angle, If vector points in positive x-direction, we
            # want to go counter-clockwise/add from current angle
            sign = np.sign(np.dot(np.array([n_p[0], n_p[2]]), np.array([1, 0])))
            elev += sign * 10

            self.view.setCameraPosition(pos=QVector3D(c_pos[0], c_pos[1], c_pos[2]), distance=100, azimuth=azi, elevation=elev)








