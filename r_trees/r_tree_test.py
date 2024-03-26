import numpy as np
import math
import timeit
from r_star_tree import RTree
from r_tree_utils import IndexRecord
from r_tree_utils import Rect
from r_tree_utils import Cube


test = 3


if test == 2:

    ###########################################################################
    # Testing 2d                                                              #
    ###########################################################################

    def sample_point(bounds):
        rand_x = math.floor((bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0])
        rand_y = math.floor((bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2])
        return rand_x, rand_y

    np.random.seed(123)
    rtree = RTree(10, dim=2, plotting=False)
    start = timeit.default_timer()

    for i in range(1000):

        x, y = sample_point([0, 800, 800, 0])
        ti = np.array([x, y])

        b = Rect([x, x, y, y])
        ir = IndexRecord(b, ti)

        rtree.Insert(entry=ir)

    stop = timeit.default_timer()
    print('Time: ', stop - start)

    # b = Rect([0, 800, 0, 800])
    # target = rtree.Search(b)
    # b.plot("#0000ff", rtree.ax)
    #
    # for i in range(len(target)):
    #     rtree.Delete(target[i])

    print(rtree)

else:

    ###########################################################################
    # Testing 3d                                                              #
    ###########################################################################

    def sample_point(bounds):
        rand_x = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
        rand_y = (bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2]
        rand_z = (bounds[5] - bounds[4]) * np.random.random_sample() + bounds[4]
        return rand_x, rand_y, rand_z

    # np.random.seed(123)
    rtree = RTree(10, dim=3, plotting=True)
    start = timeit.default_timer()

    for i in range(150):
        x, y, z = sample_point([0, 800, 0, 800, 0, 800])
        ti1 = np.array([x, y, z])
        b1 = Cube([x, x, y, y, z, z])
        i1 = IndexRecord(bound=b1, tuple_identifier=ti1)
        rtree.Insert(i1)

    stop = timeit.default_timer()

    print('Time: ', stop - start)

    # b = Cube([50, 400, 50, 400, 50, 400])
    # target = rtree.Search(b)
    # if rtree.plotting:
    #     b.plot("#0000ff", rtree.ax)
    #
    # for t in target:
    #     rtree.Delete(t)

    rtree.animate()
    print("Done!")






