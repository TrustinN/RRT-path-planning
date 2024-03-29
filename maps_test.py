import numpy as np
from utils.maps import Obstacle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# f = plt.figure()
# ax = f.add_subplot(1, 1, 1, projection=Axes3D.name)
#
# p1 = np.array([0, 0, 0])
# p2 = np.array([np.cos(np.pi / 6), np.sin(np.pi / 6), 0])
# p3 = np.array([np.cos(np.pi / 6), -np.sin(np.pi / 6), 0])
# p4 = np.array([np.cos(np.pi / 6) / 2, 0, np.sin(np.pi / 3)])
#
# f1 = Obstacle.Facet(vertices=[p1, p2, p3], dim=3)
# f2 = Obstacle.Facet(vertices=[p1, p2, p4], dim=3)
# f3 = Obstacle.Facet(vertices=[p1, p3, p4], dim=3)
# f4 = Obstacle.Facet(vertices=[p2, p3, p4], dim=3)
#
# obs = Obstacle(faces=[f1, f2, f3, f4])
# obs.plot_bound(ax)
# obs.plot(ax)
# obs.animate(ax)
#
# print("Done!")


class Edge(tuple):
    def __new__(cls, *args):
        return super(Edge, cls).__new__(cls, tuple(args))



d = Edge()
print(d)
print([1, 2, 3, 4][1:0])
print(2 // 2)

print("Done!")


