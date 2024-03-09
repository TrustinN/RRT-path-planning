from .race_utils import RaceTrack
from random import randrange

###############################################################################
# Generates race_region cones                                                 #
###############################################################################


SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800
POINT_RADIUS = 5
BACKGROUND_COLOR = (255, 255, 255)
POINT_COLOR = (255, 0, 0)
LINE_COLOR = (0, 0, 255)
ROAD_THICKNESS = 50

# Create the screen
screen = None


def new_race():
    racetrack = RaceTrack(SCREEN_WIDTH,
                          SCREEN_HEIGHT,
                          POINT_RADIUS,
                          screen,
                          POINT_COLOR=POINT_COLOR,
                          LINE_COLOR=LINE_COLOR,
                          ROAD_THICKNESS=ROAD_THICKNESS,
                          seed=randrange(1000))

    midpoints = racetrack.generate_race_course_midpath(20)
    blue_cones, yellow_cones = racetrack.generate_left_and_right_cones()
    return blue_cones, yellow_cones




