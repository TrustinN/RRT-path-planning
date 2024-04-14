import pyqtgraph as pg
from rrt_methods.RRTsolver import RRTsolver
from utils.maps.MapBuilder import MapBuilder


maps = {
    0: "rom",
    1: "maze",
}

methods = {
    0: "rrt",
    1: "rrt_connect",
    2: "rrt_star",
    3: "rrt_star_connect",
    4: "informed_rrt_star",
    5: "quick_rrt_star",
    6: "informed_quick_rrt_star"
}

builder = MapBuilder(dim=2, seed=100, map=maps[0])
rrt = RRTsolver(map=builder.map, step_size=20, max_iter=1000, method=methods[4])
print(rrt.get_time())
rrt.plot(branches=False, leaves=False)


pg.exec()







