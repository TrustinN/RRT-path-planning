import numpy as np
import math
import timeit
from r_star_tree import RTree
from r_tree_utils import IndexRecord
from r_tree_utils import Rect
from r_tree_utils import Cube
from r_tree_utils import nCircle
import matplotlib.pyplot as plt


test = 2
# np.random.seed(623)

if test == 2:

    def sample_point(bounds):
        rand_x = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
        rand_y = (bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2]
        return rand_x, rand_y

    def test_insert(rtree, n):

        p = []
        for i in range(n):
            x, y = sample_point([0, 800, 0, 800])
            p.append(np.array([x, y]))

        start = timeit.default_timer()

        for point in p:

            x, y = point[0], point[1]
            ti = point
            b = Rect([x, x, y, y])
            ir = IndexRecord(b, ti)
            rtree.Insert(entry=ir)

        stop = timeit.default_timer()
        print('Test Insert: ', stop - start)

        return p

    def test_search(rtree, scope):

        start = timeit.default_timer()
        found = rtree.Search(scope)
        scope.plot("#0000ff", rtree.ax)

        stop = timeit.default_timer()
        print('Test Search: ', stop - start)
        return found

    def test_delete(rtree, array):

        start = timeit.default_timer()

        for t in array:
            rtree.Delete(t)

        stop = timeit.default_timer()
        print('Test Delete: ', stop - start)

    def test_nearest(rtree, p):

        start = timeit.default_timer()

        b = Rect([p[0], p[0], p[1], p[1]])
        ir = IndexRecord(b, p)
        nn = rtree.NearestNeighbor(ir)
        nnti = nn.tuple_identifier

        stop = timeit.default_timer()

        plt.plot([nnti[0], p[0]], [nnti[1], p[1]])
        print('Test Nearest: ', stop - start)
        print('closest: ', nnti)

    def test_nearest_naive(array, p):

        start = timeit.default_timer()

        min_dist = math.inf
        closest = p

        for point in array:
            curr_dist = np.linalg.norm(point - p)

            if curr_dist < min_dist:

                closest = point
                min_dist = curr_dist

        stop = timeit.default_timer()
        print('Test Nearest Naive: ', stop - start)
        print('closest: ', closest)

elif test == 3:

    def sample_point(bounds):
        rand_x = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
        rand_y = (bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2]
        rand_z = (bounds[5] - bounds[4]) * np.random.random_sample() + bounds[4]
        return rand_x, rand_y, rand_z

    def test_insert(rtree, n):

        p = []
        for i in range(n):
            x, y, z = sample_point([0, 800, 0, 800, 0, 800])
            p.append(np.array([x, y, z]))

        start = timeit.default_timer()
        for point in p:
            x, y, z = point[0], point[1], point[2]
            ti = point
            b = Cube([x, x, y, y, z, z])
            ir = IndexRecord(bound=b, tuple_identifier=ti)
            rtree.Insert(ir)

        stop = timeit.default_timer()
        print('Test Insert: ', stop - start)

        return p

    def test_search(rtree, scope):

        start = timeit.default_timer()
        found = rtree.Search(scope)
        scope.plot("#0000ff", rtree.ax)

        stop = timeit.default_timer()
        print('Test Search: ', stop - start)
        return found

    def test_delete(rtree, array):

        start = timeit.default_timer()

        for t in array:
            rtree.Delete(t)

        stop = timeit.default_timer()
        print('Test Delete: ', stop - start)

    def test_nearest(rtree, p):

        start = timeit.default_timer()

        b = Cube([p[0], p[0], p[1], p[1], p[2], p[2]])
        ir = IndexRecord(b, p)
        nn = rtree.NearestNeighbor(ir)
        nnti = nn.tuple_identifier

        stop = timeit.default_timer()
        print('Test Nearest: ', stop - start)
        print('closest: ', nnti)

    def test_nearest_naive(array, p):

        start = timeit.default_timer()

        min_dist = math.inf
        closest = p

        for point in array:
            curr_dist = np.linalg.norm(point - p)

            if curr_dist < min_dist:

                closest = point
                min_dist = curr_dist

        stop = timeit.default_timer()
        print('Test Nearest Naive: ', stop - start)
        print('closest: ', closest)


rtree = RTree(40, dim=test, plotting=True)
p = test_insert(rtree, 1200)
t_point = np.array([250, 250])
found = test_search(rtree, nCircle(t_point, 200))
test_delete(rtree, found)
test_nearest(rtree, t_point)
# test_nearest_naive(p, t_point)

# rtree.animate()


print("Done!")
















