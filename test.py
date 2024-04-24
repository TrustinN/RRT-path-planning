import numpy as np
from utils.quickhull.hull import QuickHull
from utils.rtree.rtree_utils import Cube
import pyqtgraph as pg
import pyqtgraph.opengl as gl

pg.mkQApp("Test")

view = gl.GLViewWidget()
view.setCameraPosition(distance=1000)
view.pan(500, 0, 200)
view.show()


g = gl.GLGridItem()
g.translate(400, 400, -200)
g.scale(100, 100, 100)
view.addItem(g)


unit = 1000
grid_length = 1000 / 3
length = grid_length / 6

start_pos = np.array([grid_length / 2 for i in range(3)])
end_pos = np.array([grid_length, grid_length / 2, grid_length / 2])
# end_pos = np.array([703.96131703, 209.0817078, 166.66666667])

min_x = grid_length - length / 2
max_x = grid_length + length / 2

min_y = 0
max_y = grid_length

min_z = 0
max_z = grid_length

wall = QuickHull([np.array([min_x, min_y, min_z]),
                  np.array([max_x, min_y, min_z]),
                  np.array([min_x, max_y, min_z]),
                  np.array([min_x, min_y, max_z]),
                  np.array([max_x, max_y, min_z]),
                  np.array([max_x, min_y, max_z]),
                  np.array([min_x, max_y, max_z]),
                  np.array([max_x, max_y, max_z]),
                  ])


wall.plot(view)

line = gl.GLLinePlotItem(pos=np.array([start_pos, end_pos]))
line.setGLOptions("opaque")
view.addItem(line)

bound = Cube.combine_points([start_pos, end_pos])

iter = 0
for f in wall.faces:
    if bound.overlap(f.bound):
        print(bound.overlap(f.bound))
        print(f.intersects_line(start_pos, end_pos))
        iter += 1

print(iter)





























pg.exec()
