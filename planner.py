import timeit
from rrt_methods.rrt_utils import find_bounding_box
# from rrt_methods.rrt import rrt_run
# from rrt_methods.rrt_connect import rrt_run
# from rrt_methods.rrt_star import rrt_run
# from rrt_methods.rrt_star_connect import rrt_run
# from rrt_methods.informed_rrt_star import rrt_run
from rrt_methods.quick_rrt_star import rrt_run
# from rrt_methods.informed_rrt_star_connect import rrt_run
# from rrt_methods.ep_rrt_star import rrt_run


def make_plan(map):
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

    box_scope = find_bounding_box(map.region)
    map.sample_init("box", box_scope)

    path = rrt_run(map.path[0], map.path[1],
                   map,
                   50,
                   1000,
                   plot_tree=True)

    stop = timeit.default_timer()
    print('Time: ', stop - start)
    print(path)
    return path











