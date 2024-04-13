import math
import numpy as np
from .utils import Facet
from .utils import ConvexPoly


# Creates a tetrahedral based on given vertices
# The points are chosen so that they are far from each other
def CreateSimplex(vertices):
    dim = 0
    min_v, max_v = None, None
    hull = []

    # Choose two points farthest wrt x dim, y dim, or z dim
    while dim < len(vertices[0]):
        min, max = math.inf, -math.inf

        for v in vertices:
            val = v[dim]

            if val < min:
                min = val
                min_v = v

            elif val > max:
                max = val
                max_v = v

        if min != max:
            break

        dim += 1

    # Initialize a line with those points
    hull.append(min_v), hull.append(max_v)
    hyperplane = Facet(hull)

    # Choose the next two points of the tetrahedral by projection
    # onto the line and subsequently the plane
    iter = 0
    case = 0
    while True:

        farthest_pt = None
        found = False
        max_dist = 0.00001

        for v in vertices:
            dist = np.linalg.norm(hyperplane.get_projection(v))

            if dist > max_dist:
                max_dist = dist
                farthest_pt = v
                found = True

        iter += 1
        if iter == 2:
            if found:
                case = 3
            else:
                case = 2
            break

        hull.append(farthest_pt)
        hyperplane = Facet(hull)

    if case == 2:
        hull.pop()
        return [Facet([hull[i], hull[(i + 1) % len(hull)]]) for i in range(len(hull))]

    elif case == 3:
        # Reverse direction of the first three vertices if the last chosen point
        # is on the wrong side of oriented plane
        if hyperplane.orient(farthest_pt) < 0:
            hull = hull[::-1]

        # Returns first plane and cone faces created by new point
        # this is a tetrahedral
        return [Facet(hull)] + [Facet([farthest_pt, hull[(i + 1) % len(hull)], hull[i % len(hull)]]) for i in range(3)]


# Assigns unclaimed points to a face's set of outside vertices
def AddToOutside(face, unclaimed):

    # Backwards iteration since we delete from unclaimed
    for i in range(len(unclaimed) - 1, -1, -1):
        v = unclaimed[i]

        # If we are on the outside of the face's half plane, add it
        # to that facet's outside vertices set
        if face.orient(v) < -0.005:
            unclaimed.pop(i)
            face.outside_vertices.append(v)


# Calculates the edges formed by viewing the polyhedra from our given eyepoint
# The goal is to expand our convex poly to our eyepoint, reset unclaimed vertices
# and change around the graph we have of facets and their neighbors
def CalculateHorizon(eyepoint, crossed_edge, start_idx, curr_face, horizon_edges, horizon_faces, unclaimed):

    # Performs DFS search which will give us edges in a counterclockwise order
    if not curr_face.visited:

        # If we move onto a facet that we can't see we have crossed a
        # horizon edge. The horizon faces are kept to rearrange the graph
        # of neighbors for subsequent dfs searches
        if curr_face.orient(eyepoint) >= 0:
            horizon_faces.append(curr_face)
            horizon_edges.append(crossed_edge)

        # In a ccw manner, performs dfs on the facet's edges
        else:
            curr_face.visited = True

            # Visible faces to the eyepoint will lie insibe the convex poly
            curr_face.in_conv_poly = False

            # Get list of edges, put all elements in the visible face's
            # outside vertices in the unclaimed box
            edges = curr_face.vertices
            unclaimed += curr_face.outside_vertices

            # cycle through edges of curr_face looking to see which facet in
            # its neighbor set it is attached to
            for i in range(len(edges)):
                cr_edge = [edges[(start_idx + i) % len(edges)], edges[(start_idx + i + 1) % len(edges)]]

                n, st_idx, f, cont = 0, 0, None, True

                while cont:
                    f = curr_face.neighbors[n]

                    for j in range(len(f.vertices)):
                        l1 = [f.vertices[j], f.vertices[(j + 1) % len(f.vertices)]][::-1]
                        l2 = cr_edge
                        if np.array_equal(l1[0], l2[0]) and np.array_equal(l1[1], l2[1]):
                            st_idx = j
                            cont = False

                    n += 1

                # Recurse from our new facet denoted as f and with a starting
                # index with which the shared edge was found.
                CalculateHorizon(eyepoint, cr_edge, st_idx + 1, f, horizon_edges, horizon_faces, unclaimed)


# returns vertices on the hull and the faces
def QuickHull(vertices):

    facets = CreateSimplex(vertices)
    for f in facets:
        AddToOutside(f, vertices)

    num_points = len(facets)

    if num_points == 2:

        queue = facets[:]

        while queue:

            face = queue.pop()
            if face.in_conv_poly and len(face.outside_vertices) > 0:

                face.in_conv_poly = False

                # Get farthest point from the current facet
                max_dist = -math.inf
                farthest_pt = None

                for v in face.outside_vertices:
                    curr_dist = np.linalg.norm(face.get_projection(v))

                    if curr_dist > max_dist:
                        farthest_pt = v
                        max_dist = curr_dist

                unclaimed = face.outside_vertices

                f1 = Facet([face.vertices[0], farthest_pt])
                f2 = Facet([farthest_pt, face.vertices[1]])
                AddToOutside(f1, unclaimed), AddToOutside(f2, unclaimed)

                if len(f1.outside_vertices) > 0:
                    queue.append(f1)
                facets.append(f1)

                if len(f2.outside_vertices) > 0:
                    queue.append(f2)
                facets.append(f2)

    elif num_points == 4:

        # Create convex hull which starts a tetrahedral
        for i in range(num_points):
            f = facets[i]

            # connect faces as neighbors if they share an edge
            f.add_neighbor(facets[(i + 1) % num_points])
            f.add_neighbor(facets[(i + 2) % num_points])
            f.add_neighbor(facets[(i + 3) % num_points])

        # Run the loop until there are no more facets with outside vertices
        # left
        queue = facets[:]

        while queue:

            face = queue.pop()
            if face.in_conv_poly and len(face.outside_vertices) > 0:

                # Get farthest point from the current facet
                max_dist = -math.inf
                farthest_pt = None

                for v in face.outside_vertices:
                    curr_dist = np.linalg.norm(face.get_projection(v))

                    if curr_dist > max_dist:
                        farthest_pt = v
                        max_dist = curr_dist

                # Calculate Horizon will add to the list of unclaimed vertices
                # along with giving us the horizon edges/faces from our eyepoint
                horizon_edges, horizon_faces = [], []
                unclaimed = []
                CalculateHorizon(farthest_pt, None, 0, face, horizon_edges, horizon_faces, unclaimed)

                # Building the cone from eyepoint to the horizon edges
                first_f, prev_f = None, None
                for i in range(len(horizon_edges)):

                    # Create a facet of the cone and add unclaimed vertices
                    # to its outside vertices set
                    ne = horizon_edges[i]
                    ne.append(farthest_pt)
                    f = Facet(ne)
                    AddToOutside(f, unclaimed)

                    # Add to queue if this facet is not part of final convex poly
                    if len(f.outside_vertices) > 0:
                        queue.append(f)
                    facets.append(f)

                    # Connect the current facet and the facet connected by and edge
                    # to the current facet, but not visible from the eyepoint
                    curr_hface = horizon_faces[i]
                    curr_hface.add_neighbor(f)
                    f.add_neighbor(curr_hface)

                    # Connect current facet to the one built before, since we build
                    # in a ccw manner
                    if prev_f:
                        f.add_neighbor(prev_f)
                        prev_f.add_neighbor(f)
                    prev_f = f

                    # For when we create the last facet
                    if i == 0:
                        first_f = f

                    elif i == len(horizon_edges) - 1:
                        f.add_neighbor(first_f)
                        first_f.add_neighbor(f)

                    # Removing old neighbors from horizon facets
                    n, cont = 0, True

                    while cont:
                        f = curr_hface.neighbors[n]
                        vert = f.vertices

                        for j in range(len(vert)):
                            l1 = [vert[j], vert[(j + 1) % len(f.vertices)]]
                            l2 = ne

                            if np.array_equal(l1[0], l2[0]) and np.array_equal(l1[1], l2[1]):
                                curr_hface.neighbors.pop(n)
                                cont = False
                        n += 1

    # returns conv poly object found in utils.py
    # relevant facets are ones that were not excluded in the
    # CalculateHorizon process
    return ConvexPoly([f for f in facets if f.in_conv_poly])















