import math
import random
import numpy as np
import matplotlib.pyplot as plt
import textwrap


class Bound(object):
    def __init__(self, bounds=[]):
        self.bounds = bounds
        if bounds:
            self.min_x = bounds[0]
            self.max_x = bounds[1]
            self.max_y = bounds[2]
            self.min_y = bounds[3]
            self.length = self.max_x - self.min_x
            self.width = self.max_y - self.min_y
            self.area = self.width * self.length
            self.center_x = self.min_x + self.length / 2
            self.center_y = self.min_y + self.width / 2
            self.center = np.array([self.center_x, self.center_y])
            self.p_obj = None

    # returns perimeter
    def margin(self):
        return 2 * (self.length + self.width)

    # returns bounds and area
    def expand(b1, b2):
        if b1.bounds and b2.bounds:
            min_x = min(b1.min_x, b2.min_x)
            max_x = max(b1.max_x, b2.max_x)
            max_y = max(b1.max_y, b2.max_y)
            min_y = min(b1.min_y, b2.min_y)
            return [min_x, max_x, max_y, min_y]
        elif b1.bound:
            return [b1.min_x, b1.max_x, b1.max_y, b1.min_y]
        elif b2.bound:
            return [b2.min_x, b2.max_x, b2.max_y, b2.min_y]

    def combine(b1, b2):
        if b1.bounds and b2.bounds:
            bounds = Bound.expand(b1, b2)
            return Bound(bounds)
        elif b1.bounds:
            return Bound(b1.bounds)
        elif b2.bounds:
            return Bound(b2.bounds)

    # returns overlap area of two bounds
    def overlap(b1, b2):
        l_sum = .5 * (b1.length + b2.length)
        w_sum = .5 * (b1.width + b2.width)
        x_dist = abs(b1.center_x - b2.center_x)
        y_dist = abs(b1.center_y - b2.center_y)
        overlap_x = l_sum - x_dist
        overlap_y = w_sum - y_dist
        if overlap_x <= 0 or overlap_y <= 0:
            return 0
        else:
            return overlap_x * overlap_y

    def plot(self, c):
        self.p_obj = plt.plot([self.min_x, self.min_x, self.max_x, self.max_x, self.min_x],
                              [self.min_y, self.max_y, self.max_y, self.min_y, self.min_y],
                              c=c, linewidth=.5)

    def rm_plot(self):
        if self.p_obj:
            for handle in self.p_obj:
                handle.remove()
        self.p_obj = None

    def __repr__(self):
        return f"{[self.min_x, self.max_x, self.max_y, self.min_y]}"


class IndexRecord(object):
    def __init__(self, bound, tuple_identifier):
        self.bound = bound
        self.tuple_identifier = tuple_identifier

    def __repr__(self):
        return f"val: {self.tuple_identifier}"


class IndexPointer(object):
    def __init__(self, bound, pointer):
        self.bound = bound
        self.pointer = pointer
        # self.bound.plot("#ff0000")

    def update(self, bound):
        # self.bound.rm_plot()
        self.bound = Bound.combine(self.bound, bound)
        # self.bound.plot("#ff0000")

    def __repr__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"


class BranchNode(object):
    def __init__(self, indices=[], covering=Bound(), level=0):
        self.level = level
        self.covering = covering
        self.items = indices

    def add_entry(self, entry):
        self.items.append(entry)
        if self.covering:
            self.covering = Bound.combine(self.covering, entry.bound)

    def __repr__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Branch " + f"{self.covering} " + "(\n" + textwrap.indent(string, "    ") + ")"


class LeafNode(object):
    def __init__(self, indices=[], covering=Bound(), level=0):
        self.level = level
        self.covering = covering
        if self.covering:
            covering.plot("#009b00")
        self.items = indices
        self.color = "#" + "".join([random.choice('ABCDEF0123456789') for i in range(6)])
        self.points = []
        for i in self.items:
            self.points.append(plt.scatter(i.tuple_identifier[0], i.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

    def add_entry(self, entry):
        self.items.append(entry)
        if self.covering:
            self.covering.rm_plot()
            self.covering = Bound.combine(self.covering, entry.bound)
            self.covering.plot("#009b00")
        else:
            self.covering = entry.bound
        self.points.append(plt.scatter(entry.tuple_identifier[0], entry.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

    def rm_plot(self):
        for h in self.points:
            h.remove()
        self.points = []

    def __repr__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Leaf" + f"{self.covering} (" + "\n" + textwrap.indent(string, "    ") + ")"


class RTree(object):

    ###########################################################################
    # Methods                                                                 #
    ###########################################################################

    def __init__(self, M):
        self.root = LeafNode(indices=[], covering=None, level=0)
        self.max_num = M
        self.min_num = math.floor(M * .4)
        self.height = 0
        self.p = math.floor(M * .3)
        self.overflow_levels = set()

    def __repr__(self):
        return "Root:\n" + textwrap.indent(f"{self.root}", "    ")

    def FindAddedArea(self, ptr, index_entry):
        bounds = Bound.expand(ptr.bound, index_entry.bound)
        exp_area = (bounds[1] - bounds[0]) * (bounds[2] - bounds[3])
        curr_area = ptr.bound.area
        diff = exp_area - curr_area
        return curr_area, diff

    # returns current overlap area and the difference in overlap area when
    # adding new entry
    def FindAddedOverlap(self, ptr, ptrs, index_entry):
        curr_overlap = sum([Bound.overlap(ptr.bound, ptrs[i].bound) for i in range(len(ptrs)) if ptrs[i] != ptr])
        new_bound = Bound.combine(ptr.bound, index_entry.bound)
        new_overlap = sum([Bound.overlap(new_bound, ptrs[i].bound) for i in range(len(ptrs)) if ptrs[i] != ptr])
        diff = new_overlap - curr_overlap
        return curr_overlap, diff

    # choosing parent of entry to insert
    def ChooseSubTree(self, node, index_entry):

        # remember to fix this algorithm to be faster
        # currently, it runs at quadratic complexity
        min_exp, min_area = math.inf, math.inf
        idx_ptr, idx_ptr_pos = None, 0
        items = node.items

        for i in range(len(items)):

            curr_ptr = node.items[i]
            if type(curr_ptr.pointer) is LeafNode:
                curr_area, diff = self.FindAddedOverlap(curr_ptr, node.items, index_entry)
            else:
                curr_area, diff = self.FindAddedArea(curr_ptr, index_entry)

            if diff < min_exp:
                min_exp = diff
                idx_ptr = curr_ptr
                idx_ptr_pos = i
                if curr_area < min_area:
                    min_area = curr_area

            elif diff == min_exp:
                if curr_area < min_area:
                    idx_ptr = curr_ptr
                    idx_ptr_pos = i
                    min_area = curr_area

        return idx_ptr, idx_ptr_pos

    # takes in node, could be either leafnode or branchnode
    # and finds the axis to split along based on their items
    def ChooseSplitAxis(self, node):
        # Choose axis that will give us the lowest goodness value
        # calculated by the margin of the bounds
        g_value = math.inf
        axis = "x"

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

            xl1 = x_l_sort[:self.min_num + i]
            xl2 = x_l_sort[self.min_num + i:]
            min_y_1 = min([i.bound.min_y for i in xl1])
            min_y_2 = min([i.bound.min_y for i in xl2])
            max_y_1 = max([i.bound.max_y for i in xl1])
            max_y_2 = max([i.bound.max_y for i in xl2])
            margin_1 = max_y_1 - min_y_1 + xl1[-1].bound.min_x - xl1[0].bound.min_x
            margin_2 = max_y_2 - min_y_2 + xl2[-1].bound.min_x - xl2[0].bound.min_x
            curr_g = margin_1 + margin_2
            if curr_g < g_value:
                g_value = curr_g
                node.items = x_l_sort
                axis = "x"

            xu1 = x_u_sort[:self.min_num + i]
            xu2 = x_u_sort[self.min_num + i:]
            min_y_1 = min([i.bound.min_y for i in xu1])
            min_y_2 = min([i.bound.min_y for i in xu2])
            max_y_1 = max([i.bound.max_y for i in xu1])
            max_y_2 = max([i.bound.max_y for i in xu2])
            margin_1 = max_y_1 - min_y_1 + xu1[0].bound.max_x - xu1[-1].bound.max_x
            margin_2 = max_y_2 - min_y_2 + xu2[0].bound.max_x - xu2[-1].bound.max_x
            curr_g = margin_1 + margin_2
            if curr_g < g_value:
                g_value = curr_g
                node.items = x_u_sort
                axis = "x"

            yl1 = y_l_sort[:self.min_num + i]
            yl2 = y_l_sort[self.min_num + i:]
            min_x_1 = min([i.bound.min_x for i in yl1])
            min_x_2 = min([i.bound.min_x for i in yl2])
            max_x_1 = max([i.bound.max_x for i in yl1])
            max_x_2 = max([i.bound.max_x for i in yl2])
            margin_1 = max_x_1 - min_x_1 + yl1[-1].bound.min_y - yl1[0].bound.min_y
            margin_2 = max_x_2 - min_x_2 + yl2[-1].bound.min_y - yl2[0].bound.min_y
            curr_g = margin_1 + margin_2
            if curr_g < g_value:
                g_value = curr_g
                node.items = y_l_sort
                axis = "y"

            yu1 = y_u_sort[:self.min_num + i]
            yu2 = y_u_sort[self.min_num + i:]
            min_x_1 = min([i.bound.min_x for i in yu1])
            min_x_2 = min([i.bound.min_x for i in yu2])
            max_x_1 = max([i.bound.max_x for i in yu1])
            max_x_2 = max([i.bound.max_x for i in yu2])
            margin_1 = max_x_1 - min_x_1 + yu1[0].bound.max_y - yu1[-1].bound.max_y
            margin_2 = max_x_2 - min_x_2 + yu2[0].bound.max_y - yu2[-1].bound.max_y
            curr_g = margin_1 + margin_2
            if curr_g < g_value:
                g_value = curr_g
                node.items = y_u_sort
                axis = "y"
        return axis

    def ChooseSplitIndex(self, items):
        l1, l2, b1, b2 = None, None, None, None
        min_overlap = math.inf
        min_area = math.inf
        for i in range(self.max_num - 2 * self.min_num + 1):
            r1 = items[:self.min_num + i]
            r2 = items[self.min_num + i:]
            tmp_b1, tmp_b2 = Bound(), Bound()
            for b in r1:
                tmp_b1 = Bound.combine(tmp_b1, b.bound)
            for b in r2:
                tmp_b2 = Bound.combine(tmp_b2, b.bound)
            curr_overlap = Bound.overlap(tmp_b1, tmp_b2)
            if curr_overlap <= min_overlap:
                curr_area = tmp_b1.area + tmp_b2.area
                if curr_overlap == min_overlap:
                    if curr_area < min_area:
                        l1, l2 = r1, r2
                        min_area = curr_area
                        b1, b2 = tmp_b1, tmp_b2
                else:
                    if curr_area < min_area:
                        min_area = curr_area
                    min_overlap = curr_overlap
                    l1, l2 = r1, r2
                    b1, b2 = tmp_b1, tmp_b2
        return l1, l2, b1, b2

    def Split(self, node):
        axis = self.ChooseSplitAxis(node)
        print(axis)
        return self.ChooseSplitIndex(node.items)

    def OverflowTreatment(self, node, index_entry, level):
        if self.root.level > 0 and self.root.level not in self.overflow_levels:
            self.overflow_levels.add(self.root.level)
            self.Reinsert(node)
            return None, None, None, None
        else:
            l1, l2, b1, b2 = self.Split(node)
            node.rm_plot()
            node.covering.rm_plot()
            n1 = LeafNode(indices=l1, covering=b1, level=level)
            n2 = LeafNode(indices=l2, covering=b2, level=level)
            b1 = n1.covering
            b2 = n2.covering
            return n1, n2, b1, b2

    def Reinsert(self, node):
        node.rm_plot()
        node.covering.rm_plot()
        sort_dist = sorted(node.items, key=lambda x: np.linalg.norm(node.covering.center - x.bound.center), reverse=True)
        node.items = sort_dist[self.p:]
        node.covering = Bound()
        for n in node.items:
            node.covering = Bound.combine(node.covering, n.bound)
        to_insert = sort_dist[:self.p][::-1]
        for n in to_insert:
            self.insert(n)

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
            node.covering = Bound.combine(node.covering, index_entry.bound)

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
                n1 = BranchNode(indices=l1, covering=b1, level=level)
                n2 = BranchNode(indices=l2, covering=b2, level=level)

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
            self.root = BranchNode(indices=[], covering=Bound.combine(b1, b2), level=self.height)
            self.root.add_entry(p1)
            self.root.add_entry(p2)

###############################################################################
# Testing                                                                     #
###############################################################################


def sample_point(bounds):
    rand_x = math.floor((bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0])
    rand_y = math.floor((bounds[2] - bounds[3]) * np.random.random_sample() + bounds[3])
    return rand_x, rand_y


np.random.seed(123)
ax = plt.gca()
ax.set_xlim([-10, 810])
ax.set_ylim([-10, 810])

rtree = RTree(50)
for i in range(400):
    x, y = sample_point([0, 800, 800, 0])
    ti1 = np.array([x, y])
    b1 = Bound([x, x, y, y])
    i1 = IndexRecord(b1, ti1)
    rtree.insert(i1)

# Find out why it is always choosing the y-axis to split
print(rtree)
print("Done!")
















