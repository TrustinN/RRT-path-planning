import matplotlib.pyplot as plt


###############################################################################
# Plotting                                                                    #
###############################################################################


def plot_path(path, c="#000000"):
    path_x = []
    path_y = []
    for i in range(len(path)):
        idx = i % len(path)
        path_x.append(path[idx][0])
        path_y.append(path[idx][1])
    plt.plot(path_x, path_y, c=c)


def plot_poly(path, c="#000000"):
    path_x = []
    path_y = []
    for i in range(len(path) + 1):
        idx = i % len(path)
        path_x.append(path[idx][0])
        path_y.append(path[idx][1])
    plt.plot(path_x, path_y, c=c)


# Returns whether a point lies inside some polygon
def ray_cast(polygon, point):
    num_sides = len(polygon)
    inside = False
    for i in range(num_sides):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % num_sides]
        change_x = p1[0] - p2[0]
        change_y = p1[1] - p2[1]
        if (p1[1] > point[1]) != (p2[1] > point[1]) and change_y != 0:
            inv_slope = change_x / change_y
            y_scale = point[1] - p1[1]
            x_on_line = p1[0] + inv_slope * y_scale
            if x_on_line <= point[0]:
                inside = not inside
    return inside



