import math
import numpy as np
import matplotlib.pyplot as plt
from rrt_methods.rrt_utils import Ellipse
from rrt_methods.rrt_utils import sample_circle
from rrt_methods.rrt_utils import sample_ellipse
from rrt_methods.rrt_utils import normalize
from rrt_methods.rrt_utils import plot_path


# def sample_ellipse(ellipse, buffer=1):
#     center = ellipse.center
#
#     r1 = np.random.random_sample()
#     r2 = np.random.random_sample()
#     x_rand = buffer * ellipse.a * (math.sqrt(r1) + r1) / 2
#     y_rand = buffer * ellipse.b * (math.sqrt(r2) + r2) / 2
#     theta = np.random.random_sample() * 2 * math.pi
#     p = np.array([x_rand * math.cos(theta),
#                   y_rand * math.sin(theta)])
#
#     cos_theta = math.cos(-ellipse.angle)
#     sin_theta = math.sin(-ellipse.angle)
#     rotate = np.array([[cos_theta, -sin_theta],
#                       [sin_theta, cos_theta]])
#
#     p = np.dot(rotate, p)
#     return np.array([p[0] + center[0], p[1] + center[1]])
#

f0 = np.array([1, 0])
f1 = np.array([-1, 1])
d = 2.7

ell = Ellipse(f0, f1, d)
f = f0 - f1
midpoint = f / 2 + f1
a = np.linalg.norm(f / 2)
c = d / 2
b = math.sqrt(pow(c, 2) - pow(a, 2))
perp = normalize(np.array([-f[1], f[0]]))
perp = perp * b + midpoint


for i in range(1000):
    p_rand = sample_ellipse(ell)
    plt.scatter(p_rand[0], p_rand[1])

plot_path([f0, perp, f1])

# for i in range(1530):
#     p_rand = sample_circle(d, f0)
#     plt.scatter(p_rand[0], p_rand[1])


plt.axis("equal")

print("Done!")



