import numpy as np
from colorutils import Color
import pyqtgraph as pg
import pyqtgraph.opengl as gl


def plot_mesh(vertices, color):
    faces = np.arange(len(vertices)).reshape((len(vertices) // 3, 3))
    md = gl.MeshData(vertexes=np.array(vertices), faces=faces)
    c = Color(web=color)
    rgb = c.rgb
    p0, p1, p2 = rgb[0], rgb[1], rgb[2]
    colors = np.ones((md.faceCount(), 4), dtype=float)
    colors[:, 3] = 0.3
    colors[:, 2] = np.linspace(p2/255, 1, colors.shape[0])
    colors[:, 1] = np.linspace(p1/255, 1, colors.shape[0])
    colors[:, 0] = np.linspace(p0/255, 1, colors.shape[0])

    md.setFaceColors(colors=colors)
    return {
        'meshdata': md,
        'smooth': False,
        'shader': 'shaded',
    }


def plot_polygons(vertices, color):

    v = np.array(vertices)
    return {
        'x': v[:, 0],
        'y': v[:, 1],
        'connect': 'pairs',
        'pen': pg.mkPen(color),
    }


def plot_points(points, color, dim=2):
    if dim == 2:
        return {
            'pos': np.array(points),
            'size': 3,
            'brush': color,
        }

    elif dim == 3:
        return {
            'pos': np.array(points),
            'color': color,
            'size': 5
        }





