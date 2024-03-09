from random import randrange
import pygame
import sys
from planner import make_plan
from utils.make_race import new_race
from utils.maps import race_map


def main():
    # Initialize Pygame
    pygame.init()

    # Constants
    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 800
    POINT_RADIUS = 5
    BACKGROUND_COLOR = (255, 255, 255)
    POINT_COLOR = (255, 0, 0)
    LINE_COLOR = (0, 0, 255)
    ROAD_THICKNESS = 50

    # Create the screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Path Planning Simulator")

    racetrack = new_race(SCREEN_WIDTH,
                         SCREEN_HEIGHT,
                         POINT_RADIUS,
                         screen,
                         POINT_COLOR,
                         LINE_COLOR,
                         ROAD_THICKNESS,
                         randrange(1000))

    planner_path = []

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    planner_path = make_plan(race_map(racetrack=racetrack, n=20))

        screen.fill(BACKGROUND_COLOR)
        # racetrack.draw_midpoints()
        racetrack.draw_yellow_cones()
        racetrack.draw_blue_cones()
        racetrack.draw_start()
        racetrack.draw_start_directon()
        racetrack.draw_path(planner_path, [0]*len(planner_path))
        racetrack.flip()

    # Quit Pygame
    pygame.quit()
    sys.exit()

if __name__ == '__main__': main()




