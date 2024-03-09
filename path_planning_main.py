import pygame
import sys
from path_utils import RaceTrack
from gui import planner


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

    racetrack = RaceTrack(SCREEN_WIDTH, SCREEN_HEIGHT, POINT_RADIUS, screen,  POINT_COLOR=POINT_COLOR, LINE_COLOR=LINE_COLOR, ROAD_THICKNESS=ROAD_THICKNESS, seed=42)

    planner_path = []
    velocities = []

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    planner_path = planner()

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



