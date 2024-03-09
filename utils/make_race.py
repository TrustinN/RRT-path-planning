from .race_utils import RaceTrack

###############################################################################
# Generates race_region cones                                                 #
###############################################################################


def new_race(width, height, point_radius, screen, point_color, line_color, road_thickness, seed):
    racetrack = RaceTrack(width, height, point_radius, screen, POINT_COLOR=point_color, LINE_COLOR=line_color, ROAD_THICKNESS=road_thickness, seed=seed)

    return racetrack




