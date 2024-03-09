import timeit
from rrt_methods.rrt_utils import Map
from rrt_methods.rrt_utils import find_bounding_box
import utils.maps as maps
# from rrt_methods.rrt import rrt_run
# from rrt_methods.rrt_connect import rrt_run
# from rrt_methods.rrt_star import rrt_run
# from rrt_methods.rrt_star_connect import rrt_run
# from rrt_methods.informed_rrt_star import rrt_run
from rrt_methods.quick_rrt_star import rrt_run
# from rrt_methods.informed_rrt_star_connect import rrt_run
# from rrt_methods.ep_rrt_star import rrt_run


def planner():
    """plans a path through the track given the blue and yellow cones.

    Args:
        blue_cones (list): blue (left) cone locations. shape (n, 2).
        yellow_cones (list): yellow (right) cone locations. shape (n, 2).

    Returns:
        list: output path, with shape (n, 2)
    """

    # YOUR CODE HERE #
    # fill out this function to make it work
    # as described in the docstring above
    start = timeit.default_timer()

    start_pos, end_pos, region, obstacles = maps.race_map()
    map = Map(region=region, obstacles=obstacles)
    map.add_path([start_pos, end_pos])

    box_scope = find_bounding_box(region)
    map.sample_init("box", box_scope)

    path = rrt_run(start_pos, end_pos,
                   map,
                   50,
                   1000,
                   plot_tree=False)

    stop = timeit.default_timer()
    print('Time: ', stop - start)
    return path











