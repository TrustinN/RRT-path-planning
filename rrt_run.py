import timeit
import numpy as np

###############################################################################
# Make Test Regions for RRT                                                   #
###############################################################################

import matplotlib.pyplot as plt
import utils.maps as maps

###############################################################################
# Sampling                                                                    #
###############################################################################

from rrt_methods.rrt_utils import find_bounding_box

###############################################################################
# Plotting                                                                    #
###############################################################################

from utils.map_utils import plot_path

###############################################################################
# RRT Methods and Variations                                                  #
###############################################################################

from rrt_methods.rrt import rrt_run
# from rrt_methods.rrt_connect import rrt_run
# from rrt_methods.rrt_star import rrt_run
# from rrt_methods.rrt_star_connect import rrt_run
# from rrt_methods.quick_rrt_star import rrt_run
# from rrt_methods.informed_rrt_star import rrt_run
# from rrt_methods.informed_rrt_star_connect import rrt_run
# from rrt_methods.ep_rrt_star import rrt_run

###############################################################################
# Generate Regions                                                            #
###############################################################################

# start_pos, end_pos, region, obstacles = maps.race_map()
map = maps.square_obs_map(7, 100)
# start_pos, end_pos, region, obstacles = maps.make_maze(20)

# Time:  0.4796205410384573
box_scope = find_bounding_box(map.region)
map.sample_init("box", box_scope)

start = timeit.default_timer()
path = rrt_run(map.path[0], map.path[1],
               map,
               50,
               2000,
               )
stop = timeit.default_timer()

print('Time: ', stop - start)


plot_path(path, c="#000000")
plt.axis("equal")












