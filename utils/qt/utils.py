import numpy as np
from colorutils import Color
import pyqtgraph.opengl as gl


def plot_mesh(vertices, view, color, option='opaque'):
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
    m1 = gl.GLMeshItem(meshdata=md, smooth=False, shader='shaded')
    m1.setGLOptions(option)

    view.addItem(m1)


