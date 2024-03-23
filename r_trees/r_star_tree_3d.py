import math
import random
import numpy as np
import matplotlib.pyplot as plt
import textwrap
import timeit
from mpl_toolkits.mplot3d import Axes3D


def get_cube():
    phi = np.arange(1, 10, 2) * np.pi / 4
    Phi, Theta = np.meshgrid(phi, phi)

    x = np.cos(Phi)*np.sin(Theta)
    y = np.sin(Phi)*np.sin(Theta)
    z = np.cos(Theta)/np.sqrt(2)
    return x, y, z


class Bound(object):

    def __init__(self, bounds=[]):
        self.bounds = bounds
        if bounds:
            self.min_x = bounds[0]
            self.max_x = bounds[1]
            self.min_y = bounds[2]
            self.max_y = bounds[3]
            self.min_z = bounds[4]
            self.max_z = bounds[5]

            self.length = self.max_x - self.min_x
            self.width = self.max_y - self.min_y
            self.height = self.max_z - self.min_z

            self.volume = self.width * self.length * self.height
            self.center_x = self.min_x + self.length / 2
            self.center_y = self.min_y + self.width / 2
            self.center_z = self.min_z + self.height / 2
            self.center = np.array([self.center_x, self.center_y, self.center_z])

            self.p_obj = None

    # returns perimeter measure
    def margin(self):
        return self.length + self.width + self.height

    # returns bounds and area
    def expand(b1, b2):
        if b1.bounds and b2.bounds:
            min_x = min(b1.min_x, b2.min_x)
            max_x = max(b1.max_x, b2.max_x)

            min_y = min(b1.min_y, b2.min_y)
            max_y = max(b1.max_y, b2.max_y)

            min_z = min(b1.min_z, b2.min_z)
            max_z = max(b1.max_z, b2.max_z)

            return [min_x, max_x, min_y, max_y, min_z, max_z]
        elif b1.bound:
            return [b1.min_x, b1.max_x, b1.min_y, b1.max_y, b1.min_z, b1.max_z]
        elif b2.bound:
            return [b2.min_x, b2.max_x, b2.min_y, b2.max_y, b2.min_z, b2.max_z]

    def combine(b1, b2):
        if b1.bounds and b2.bounds:
            bounds = Bound.expand(b1, b2)
            return Bound(bounds)
        elif b1.bounds:
            return Bound(b1.bounds)
        elif b2.bounds:
            return Bound(b2.bounds)

    def combine_l(bounds):
        min_x, max_x, min_y, max_y, min_z, max_z = math.inf, -math.inf, math.inf, -math.inf, math.inf, -math.inf
        for b in bounds:
            if b.min_x < min_x:
                min_x = b.min_x
            if b.max_x > max_x:
                max_x = b.max_x

            if b.min_y < min_y:
                min_y = b.min_y
            if b.max_y > max_y:
                max_y = b.max_y

            if b.min_z < min_z:
                min_z = b.min_z
            if b.max_z > max_z:
                max_z = b.max_z

        return [min_x, max_x, min_y, max_y, min_z, max_z]

    # returns overlap area of two bounds
    def overlap(b1, b2):
        l_sum = .5 * (b1.length + b2.length)
        w_sum = .5 * (b1.width + b2.width)
        h_sum = .5 * (b1.height + b2.height)

        x_dist = abs(b1.center_x - b2.center_x)
        y_dist = abs(b1.center_y - b2.center_y)
        z_dist = abs(b1.center_z - b2.center_z)

        overlap_x = l_sum - x_dist
        overlap_y = w_sum - y_dist
        overlap_z = h_sum - z_dist

        if overlap_x <= 0 or overlap_y <= 0 or overlap_z <= 0:
            return 0
        else:
            return overlap_x * overlap_y * overlap_z

    def plot(self, c, ax):
        x, y, z = get_cube()
        self.p_obj = ax.plot_surface(self.length * x + self.center_x,
                                     self.width * y + self.center_y,
                                     self.height * z + self.center_z,
                                     alpha=0.08,
                                     shade=False,
                                     )
        # self.p_obj = plt.plot([self.min_x, self.min_x, self.max_x, self.max_x, self.min_x],
        #                       [self.min_y, self.max_y, self.max_y, self.min_y, self.min_y],
        #                       c=c, linewidth=.5)

    def rm_plot(self):
        if self.p_obj:
            self.p_obj.remove()
        self.p_obj = None

    def __str__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z]}"

    def __repr__(self):
        return f"{[self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z]}"


class IndexRecord(object):

    def __init__(self, bound, tuple_identifier):
        self.bound = bound
        self.tuple_identifier = tuple_identifier

    def __str__(self):
        return f"val: {self.tuple_identifier}"

    def __repr__(self):
        return f"val: {self.tuple_identifier}"


class IndexPointer(object):

    def __init__(self, bound, pointer):
        self.bound = bound
        self.pointer = pointer

    def update(self, bound):
        self.bound = Bound.combine(self.bound, bound)

    def __str__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"

    def __repr__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"


class BranchNode(object):

    def __init__(self, indices=[], covering=Bound(), level=0, plotting=False, ax=None):
        self.level = level
        self.covering = covering
        self.has_overflown = False
        self.items = indices
        self.plotting = False
        self.ax = ax

        if self.plotting:
            if self.covering:
                covering.plot("#ff0000", ax)

    def add_entry(self, entry):
        self.items.append(entry)
        if self.covering:
            self.covering.rm_plot()
            self.covering = Bound.combine(self.covering, entry.bound)
            if self.plotting:
                self.covering.plot("#ff0000", ax)

    def update_bound(self, bound):
        self.covering.rm_plot()
        self.covering = Bound.combine(self.covering, bound)
        if self.plotting:
            self.covering.plot("#ff0000", ax)

    def __str__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Branch " + f"{self.level} " + "(\n" + textwrap.indent(string, "    ") + ")"

    def __repr__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Branch " + f"{self.level} " + "(\n" + textwrap.indent(string, "    ") + ")"

class LeafNode(object):

    def __init__(self, indices=[], covering=Bound(), level=0, plotting=False, ax=None):
        self.level = level
        self.items = indices
        self.covering = covering
        self.has_overflown = False
        self.plotting = plotting
        self.ax = ax

        if self.plotting:
            if self.covering:
                covering.plot("#009b00", ax)
            self.color = "#" + "".join([random.choice('ABCDEF0123456789') for i in range(6)])
            self.points = []
            for i in self.items:
                self.points.append(self.ax.scatter(i.tuple_identifier[0], i.tuple_identifier[1], i.tuple_identifier[2], c=self.color, s=10, edgecolor='none'))

    def plot(self):
        for i in self.items:
            self.points.append(self.ax.scatter(i.tuple_identifier[0], i.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

    def add_entry(self, entry):
        self.items.append(entry)
        if self.covering:
            self.covering.rm_plot()
            self.covering = Bound.combine(self.covering, entry.bound)
            if self.plotting:
                self.covering.plot("#009b00", ax)
        else:
            self.covering = entry.bound
        if self.plotting:
            self.points.append(self.ax.scatter(entry.tuple_identifier[0], entry.tuple_identifier[1], entry.tuple_identifier[2], c=self.color, s=10, edgecolor='none'))

    def rm_plot(self):
        if self.plotting:
            for h in self.points:
                h.remove()
            self.points = []

    def __str__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Leaf" + f"{self.level} (" + "\n" + textwrap.indent(string, "    ") + ")"

    def __repr__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Leaf" + f"{self.level} (" + "\n" + textwrap.indent(string, "    ") + ")"


class RTree(object):

    ###########################################################################
    # Methods                                                                 #
    ###########################################################################

    def __init__(self, M, plotting=False, ax=None):
        self.max_num = M
        self.min_num = math.floor(M * .4)
        self.height = 0
        self.p = min(math.floor(M * .3), 32)
        self.plotting = plotting
        self.ax = ax
        self.root = LeafNode(indices=[], covering=None, level=0, ax=ax)

    def __str__(self):
        return "Root:\n" + textwrap.indent(f"{self.root}", "    ")

    def FindAddedArea(self, ptr, index_entry):
        bounds = Bound.expand(ptr.bound, index_entry.bound)
        exp_area = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2]) * (bounds[5] - bounds[4])
        curr_area = ptr.bound.volume
        diff = exp_area - curr_area
        return curr_area, diff

    # returns current overlap area and the difference in overlap area when
    # adding new entry
    def FindAddedOverlap(self, ptr, ptrs, index_entry):
        curr_overlap = sum([Bound.overlap(ptr.bound, p.bound) for p in ptrs if p != ptr])
        new_bound = Bound.combine(ptr.bound, index_entry.bound)
        new_overlap = sum([Bound.overlap(new_bound, p.bound) for p in ptrs if p != ptr])
        diff = new_overlap - curr_overlap
        return curr_overlap, diff

    # choosing parent of entry to insert
    def ChooseSubTree(self, node, index_entry):

        # pick the subdirectory that leads to least expansion
        # idx_ptr is the entry and idx_ptr_pos is the index
        # of the idx_ptr in node.items
        min_exp, min_area = math.inf, math.inf
        idx_ptr, idx_ptr_pos = node.items[0], 0

        # if type(idx_ptr.pointer) is LeafNode:
        #
        #     # sort by least area needed to expand and take first p entries
        #     node.items = sorted(node.items, key=lambda x: self.FindAddedArea(x, index_entry)[1])
        #     items = node.items[:self.p]
        #
        #     # find the expansion that results in least overlap
        #     for i in range(len(items)):
        #
        #         curr_ptr = items[i]
        #         curr_area, diff = self.FindAddedOverlap(curr_ptr, node.items, index_entry)
        #
        #         if diff < min_exp:
        #             min_exp = diff
        #             idx_ptr = curr_ptr
        #             idx_ptr_pos = i
        #             min_area = curr_area
        #
        #         # tiebreaker: choose smaller bounding box
        #         elif diff == min_exp:
        #             if curr_area < min_area:
        #                 idx_ptr = curr_ptr
        #                 idx_ptr_pos = i
        #                 min_area = curr_area
        # else:

        # find the expansion that results in least area added
        for i in range(len(node.items)):

            curr_ptr = node.items[i]
            curr_area, diff = self.FindAddedArea(curr_ptr, index_entry)

            if diff < min_exp:
                min_exp = diff
                idx_ptr = curr_ptr
                idx_ptr_pos = i
                min_area = curr_area

            # tiebreaker: choose smaller bounding box
            elif diff == min_exp:

                if curr_area < min_area:

                    idx_ptr = curr_ptr
                    idx_ptr_pos = i
                    min_area = curr_area

        return idx_ptr, idx_ptr_pos

    # takes in node, could be either leafnode or branchnode
    # and finds the axis to split along based on their items
    def ChooseSplitAxis(self, node):

        # splits items along start + idx index and calculates the
        # goodness value based on the two split lists
        def helper_func(items, start, idx):

            sb1 = [items[j].bound for j in range(start + idx)]
            sb2 = [items[j + start + idx].bound for j in range(len(items) - start - idx)]

            mb1 = Bound(Bound.combine_l(sb1))
            mb2 = Bound(Bound.combine_l(sb2))

            margin_1, margin_2 = mb1.margin(), mb2.margin()

            return margin_1 + margin_2

        # Choose axis that will give us the lowest goodness value
        # calculated by the margin of the bounds
        g_value = math.inf

        # x-axis calculation
        # sort by lower value of bounding box
        x_l_sort = sorted(node.items, key=lambda x: x.bound.min_x)

        # x-axis calculation
        # sort by upper value f bounding box
        x_u_sort = sorted(node.items, key=lambda x: x.bound.max_x, reverse=True)

        # y-axis calculation
        # sort by upper value f bounding box
        y_l_sort = sorted(node.items, key=lambda x: x.bound.min_y)

        # y-axis calculation
        # sort by upper value f bounding box
        y_u_sort = sorted(node.items, key=lambda x: x.bound.max_y, reverse=True)

        for i in range(self.max_num - 2 * self.min_num + 1):

            curr_g = helper_func(x_l_sort, self.min_num, i)

            if curr_g < g_value:
                g_value = curr_g
                node.items = x_l_sort

            curr_g = helper_func(x_u_sort, self.min_num, i)

            if curr_g < g_value:
                g_value = curr_g
                node.items = x_u_sort

            curr_g = helper_func(y_l_sort, self.min_num, i)

            if curr_g < g_value:
                g_value = curr_g
                node.items = y_l_sort

            curr_g = helper_func(y_u_sort, self.min_num, i)

            if curr_g < g_value:
                g_value = curr_g
                node.items = y_u_sort

    def ChooseSplitIndex(self, items):

        split_idx, b1, b2 = None, None, None
        min_overlap = math.inf
        min_area = math.inf

        for i in range(self.max_num - 2 * self.min_num + 1):

            # find index to split along by least overlap
            s1 = [items[j].bound for j in range(self.min_num + i)]
            s2 = [items[j + self.min_num + i].bound for j in range(len(items) - self.min_num - i)]
            tmp_b1 = Bound(Bound.combine_l(s1))
            tmp_b2 = Bound(Bound.combine_l(s2))

            curr_overlap = Bound.overlap(tmp_b1, tmp_b2)

            if curr_overlap <= min_overlap:

                curr_area = tmp_b1.volume + tmp_b2.volume

                # tiebreaker: choose smaller bounding box
                if curr_overlap == min_overlap:

                    if curr_area < min_area:
                        split_idx = self.min_num + i
                        min_area = curr_area
                        b1, b2 = tmp_b1, tmp_b2

                else:

                    min_area = curr_area
                    min_overlap = curr_overlap
                    split_idx = self.min_num + i
                    b1, b2 = tmp_b1, tmp_b2

        return items[:split_idx], items[split_idx:], b1, b2

    def Split(self, node):
        self.ChooseSplitAxis(node)
        return self.ChooseSplitIndex(node.items)

    def OverflowTreatment(self, node, index_entry, level):
        # if self.root.level > 0 and not node.has_overflown:
        #     node.has_overflown = True
        #     self.Reinsert(node)
        #     return None, None, None, None
        # else:
        l1, l2, b1, b2 = self.Split(node)
        node.rm_plot()
        node.covering.rm_plot()
        n1 = LeafNode(indices=l1, covering=b1, level=level, plotting=self.plotting, ax=self.ax)
        n2 = LeafNode(indices=l2, covering=b2, level=level, plotting=self.plotting, ax=self.ax)
        b1 = n1.covering
        b2 = n2.covering
        return n1, n2, b1, b2

    # def Reinsert(self, node):
    #     node.rm_plot()
    #     node.covering.rm_plot()
    #     sort_dist = sorted(node.items, key=lambda x: np.linalg.norm(node.covering.center - x.bound.center), reverse=True)
    #     node.items = sort_dist[self.p:]
    #     node.covering = Bound()
    #     for n in node.items:
    #         node.covering = Bound.combine(node.covering, n.bound)
    #     node.plot()
    #     to_insert = sort_dist[:self.p][::-1]
    #     for n in to_insert:
    #         self.insert(n)

    def ChooseLeaf(self, node, index_entry, level):

        n1, n2, b1, b2 = None, None, None, None

        if type(node) is LeafNode:

            if len(node.items) < self.max_num:

                # if we can add the entry without exceeding leaf size
                # then add it
                node.add_entry(index_entry)

            else:

                # if node is too big, split leaf node and add the entry to
                # both each of the splits
                node.add_entry(index_entry)
                n1, n2, b1, b2 = self.OverflowTreatment(node, index_entry, level)

        else:

            # choosing parent of entry to insert
            idx_pointer, idx_pointer_pos = self.ChooseSubTree(node, index_entry)

            # index.pointer is the pointer to one of node's children nodes
            n1, n2, b1, b2 = self.ChooseLeaf(idx_pointer.pointer, index_entry, level - 1)

            # update bound
            idx_pointer.update(index_entry.bound)
            node.update_bound(index_entry.bound)

            if n2:

                # Should be creating new indexpointers for each split created
                # These indexpointers will lie in our current node.
                node.items[idx_pointer_pos] = IndexPointer(b2, n2)
                pointer_1 = IndexPointer(b1, n1)
                node.add_entry(pointer_1)
                n2 = None

            if len(node.items) > self.max_num:

                # If the branch node has too many items split
                l1, l2, b1, b2 = self.Split(node)
                node.covering.rm_plot()
                n1 = BranchNode(indices=l1, covering=b1, level=level, plotting=self.plotting, ax=self.ax)
                n2 = BranchNode(indices=l2, covering=b2, level=level, plotting=self.plotting, ax=self.ax)

        # returns either split leaf nodes, or branch nodes, depending on
        # which one is at the highest level of tree
        return n1, n2, b1, b2

    # inserts entry with propagation of changes upwards until root node
    # if root node is too big, we split
    def insert(self, index_entry):

        n1, n2, b1, b2 = self.ChooseLeaf(self.root, index_entry, self.height)

        if n2:

            self.height += 1
            p1 = IndexPointer(bound=b1, pointer=n1)
            p2 = IndexPointer(bound=b2, pointer=n2)
            self.root = BranchNode(indices=[], covering=Bound.combine(b1, b2), level=self.height, plotting=self.plotting, ax=self.ax)
            self.root.add_entry(p1)
            self.root.add_entry(p2)


###############################################################################
# Testing                                                                     #
###############################################################################


def sample_point(bounds):
    rand_x = (bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0]
    rand_y = (bounds[3] - bounds[2]) * np.random.random_sample() + bounds[3]
    rand_z = (bounds[5] - bounds[4]) * np.random.random_sample() + bounds[5]
    return rand_x, rand_y, rand_z


# np.random.seed(123)
f = plt.figure()
ax = f.add_subplot(1, 1, 1, projection=Axes3D.name)

rtree = RTree(20, plotting=True, ax=ax)
start = timeit.default_timer()

for i in range(100):
    x, y, z = sample_point([0, 800, 0, 800, 0, 800])
    ti1 = np.array([x, y, z])
    b1 = Bound([x, x, y, y, z, z])
    i1 = IndexRecord(bound=b1, tuple_identifier=ti1)
    rtree.insert(i1)

stop = timeit.default_timer()

for angle in range(0, 500, 2):
    ax.view_init(elev=30, azim=angle, roll=0)
    plt.draw()
    plt.pause(.001)

plt.show()

print('Time: ', stop - start)

# Fix overflow treatment: somehow duplicating pointers
# This is probably because Split is called twice
# when you overflow, you also want to reset directory page bound
# once in the reinsert algorithm and another outside the reinsert
# Bad performance when there are more than 2
# layers to tree
# Find added overlap operation is expensive also
# print(rtree)

print("Done!")


















