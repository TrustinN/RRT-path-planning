import numpy as np
from utils.maps.map_utils import get_angle
from PyQt6.QtGui import QVector3D


class Camera():

    def set_view(self, view):
        self.view = view

    def connect(self, slider, buttons):
        self.slider = slider
        self.play_button = buttons[0]
        self.reverse_button = buttons[1]

        self.play_button.clicked.connect(self.animate_path)
        self.reverse_button.clicked.connect(self.reverse_path)
        self.reverse_button.clicked.connect(self.slider.reverse)

    def reset(self):
        self.slider.setValue((self.slider.min + self.slider.max) // 2)
        self.slider.reset()
        self.view.setCameraPosition(pos=QVector3D(400, 400, 400), elevation=0, azimuth=-35, distance=1200)
        self.play_button.setChecked(False)

    def hide_slider(self):
        self.slider.hide()

    def show_slider(self):
        self.slider.show()

    def animate_path(self):
        if self.play_button.isChecked:
            self.slider.animate()

    def track_path(self, path):
        self.track = path
        self.elev = 1

    def reverse_path(self):
        if self.track:
            self.track = self.track[::-1]

    def disconnect(self):
        self.track = None

    def follow_path(self):

        percent = self.slider.sliderPosition() / self.slider.max
        if self.slider.reversed:
            percent = 1 - percent

        path_lengths = [np.linalg.norm(self.track[i] - self.track[i + 1]) for i in range(len(self.track) - 1)]
        path_length = sum(path_lengths)

        if self.track:
            dist = percent * path_length
            curr_dist = 0
            iter = 0

            while True:
                curr_dist += path_lengths[iter]
                if curr_dist >= dist:
                    curr_dist -= path_lengths[iter]
                    break
                iter += 1

            start = self.track[iter]
            end = self.track[iter + 1]
            vec = end - start

            vec_dist = np.linalg.norm(vec)
            p = (dist - curr_dist) / vec_dist

            c_pos = p * vec + start

            # calculate the rotation needed for camera from x-axis
            # to be positioned behind the current vector, vec
            angle = get_angle(vec[:2], np.array([1, 0]))

            # We need to keep track of if the angle given is a counter clockwise
            # or clockwise rotation from x-axis
            sign = np.sign(np.dot(vec[:2], np.array([0, 1])))
            angle = sign * angle

            # Move to oppositie side of the unit circle
            azi = 180 * angle / np.pi
            azi = azi + 180

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

            # Reflect angle across z-axis
            if sign < 0:
                elev = 180 - elev

            elev += 10

            self.view.setCameraPosition(pos=QVector3D(c_pos[0], c_pos[1], c_pos[2]), distance=50, azimuth=azi, elevation=elev)








