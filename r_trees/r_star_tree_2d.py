import math
import random
import numpy as np
import matplotlib.pyplot as plt
import textwrap
import timeit
from r_tree_utils import Rect as Bound
# from r_tree_utils import Cube
from r_tree_utils import IndexRecord
from r_tree_utils import IndexPointer


class RTree(object):

    class Node:

        def __init__(self, items, covering, level, plotting):

            self.items = items
            self.covering = covering
            self.level = level
            self.plotting = plotting

        def add_entry(self, entry):
            """
            Add an entry to the node's list of items
            """
            return

    class BranchNode(Node):

        def __init__(self, items=[], covering=Bound(), level=0, plotting=False):

            super().__init__(items, covering, level, plotting)

            if self.plotting:
                if self.covering:
                    covering.plot("#ff0000")

        def add_entry(self, entry):

            self.items.append(entry)

            if self.covering:

                self.covering.rm_plot()
                self.covering = Bound.combine([self.covering, entry.bound])

                if self.plotting:
                    self.covering.plot("#ff0000")

        def update_bound(self, bound):

            self.covering.rm_plot()
            self.covering = bound

            if self.plotting:
                self.covering.plot("#ff0000")

        def rm_plot(self):

            self.covering.rm_plot()

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

    class LeafNode(Node):

        def __init__(self, items=[], covering=Bound(), level=0, plotting=False):

            super().__init__(items, covering, level, plotting)

            if self.plotting:
                if self.covering:
                    covering.plot("#009b00")

                self.color = "#" + "".join([random.choice('ABCDEF0123456789') for i in range(6)])
                self.points = []

                for i in self.items:
                    self.points.append(plt.scatter(i.tuple_identifier[0], i.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

        def plot(self):

            if self.plotting:
                for i in self.items:
                    self.points.append(plt.scatter(i.tuple_identifier[0], i.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

        def add_entry(self, entry):

            self.items.append(entry)

            if self.covering:

                self.covering.rm_plot()
                self.covering = Bound.combine([self.covering, entry.bound])

                if self.plotting:
                    self.covering.plot("#009b00")

            else:
                self.covering = entry.bound

            if self.plotting:
                self.points.append(plt.scatter(entry.tuple_identifier[0], entry.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

        def rm_entry(self, entry):

            for i in range(len(self.items)):

                if entry == self.items[i]:

                    # Remove index_entry, adjust leaf covering
                    self.items.pop(i)
                    point_plot = self.points.pop(i)
                    point_plot.remove()
                    self.update_bound(Bound.combine([j.bound for j in self.items]))

                    return True

            return False

        def update_bound(self, bound):

            self.covering.rm_plot()
            self.covering = bound

            if self.plotting:
                self.covering.plot("#009b00")

        def rm_plot(self):

            if self.plotting:

                for h in self.points:
                    h.remove()

                self.points = []
                self.covering.rm_plot()

        def __str__(self):
            string = ""
            for i in self.items:
                string += str(i) + "\n"
            return "Leaf " + f"{self.level} (" + "\n" + textwrap.indent(string, "    ") + ")"

        def __repr__(self):
            string = ""
            for i in self.items:
                string += str(i) + "\n"
            return "Leaf " + f"{self.level} (" + "\n" + textwrap.indent(string, "    ") + ")"

    root = LeafNode(items=[], covering=None, level=0)

    ###########################################################################
    # Methods                                                                 #
    ###########################################################################

    def __init__(self, M, dim, plotting=False):

        self.max_num = M
        self.min_num = math.floor(M * .4)

        self.height = 0
        self.p = min(math.floor(M * .3), 32)

        self.plotting = plotting

        if self.plotting:
            plt.gca()

        if dim == 2:
            return

    def __str__(self):
        return "Root:\n" + textwrap.indent(f"{self.root}", "    ")

    def __repr__(self):
        return "Root:\n" + textwrap.indent(f"{self.root}", "    ")

    def FindAddedArea(ptr, index_entry):

        exp_area = Bound.expand_area(ptr.bound, index_entry.bound)
        curr_area = ptr.bound.area
        diff = exp_area - curr_area

        return curr_area, diff

    # returns current overlap area and the difference in overlap area when
    # adding new entry
    def FindAddedOverlap(ptr, ptrs, index_entry):

        curr_overlap = sum(Bound.overlap(ptr.bound, p.bound) for p in ptrs if p != ptr)

        new_bound = Bound.combine([ptr.bound, index_entry.bound])
        new_overlap = sum(Bound.overlap(new_bound, p.bound) for p in ptrs if p != ptr)

        diff = new_overlap - curr_overlap

        return curr_overlap, diff

    # choosing parent of entry to insert
    def ChooseSubTree(self, node, index_entry, curr_lvl):

        # pick the subdirectory that leads to least expansion
        min_exp, min_area = math.inf, math.inf
        idx_ptr, ptr_pos = None, 0

        if curr_lvl == 0:

            # sort by least area needed to expand and take first p entries
            node.items = sorted(node.items, key=lambda x: RTree.FindAddedArea(x, index_entry)[1])
            items = node.items[:self.p]

        else:
            items = node.items

        # find the expansion that results in least area added
        for i in range(len(items)):

            curr_ptr = items[i]

            if curr_lvl == 0:
                curr_area, diff = RTree.FindAddedOverlap(curr_ptr, node.items, index_entry)

            else:
                curr_area, diff = RTree.FindAddedArea(curr_ptr, index_entry)

            if diff < min_exp:

                min_exp = diff
                idx_ptr = curr_ptr
                ptr_pos = i
                min_area = curr_area

            # tiebreaker: choose smaller bounding box
            elif diff == min_exp:
                if curr_area < min_area:

                    idx_ptr = curr_ptr
                    ptr_pos = i
                    min_area = curr_area

        return idx_ptr, ptr_pos

    # takes in node, could be either leafnode or branchnode
    # and finds the axis to split along based on their items
    def ChooseSplitAxis(self, node):

        # splits items along start + idx index and calculates the
        # goodness value based on the two split lists
        def helper_func(items, start, idx):

            sb1 = [items[j].bound for j in range(start + idx)]
            sb2 = [items[j + start + idx].bound for j in range(len(items) - start - idx)]

            mb1 = Bound.combine(sb1)
            mb2 = Bound.combine(sb2)

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

            tmp_b1 = Bound.combine(s1)
            tmp_b2 = Bound.combine(s2)

            curr_overlap = Bound.overlap(tmp_b1, tmp_b2)

            if curr_overlap <= min_overlap:

                curr_area = tmp_b1.area + tmp_b2.area

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
        node.rm_plot()

        return self.ChooseSplitIndex(node.items)

    def OverflowTreatment(self, node, level, overflow):

        if level != self.height and level not in overflow:

            overflow.add(level)
            to_insert = self.Reinsert(node)

            return None, None, None, None, to_insert, level

        else:
            if level == 0:

                l1, l2, b1, b2 = self.Split(node)

                n1 = RTree.LeafNode(items=l1, covering=b1, level=level, plotting=self.plotting)
                n2 = RTree.LeafNode(items=l2, covering=b2, level=level, plotting=self.plotting)

                b1 = n1.covering
                b2 = n2.covering

                return n1, n2, b1, b2, None, None

            else:
                l1, l2, b1, b2 = self.Split(node)

                return l1, l2, b1, b2, None, None

    def Reinsert(self, node):

        node.rm_plot()

        sort_dist = sorted(node.items, key=lambda x: np.linalg.norm(node.covering.center - x.bound.center), reverse=True)
        node.items = sort_dist[self.p:]

        node.update_bound(Bound.combine([n.bound for n in node.items]))

        if type(node) is RTree.LeafNode:
            node.plot()

        return sort_dist[:self.p][::-1]

    def ChooseLeaf(self, node, index_entry, curr_lvl, overflowed, ins_lvl=0):

        n1, n2, b1, b2, q, r_lvl = None, None, None, None, [], None

        if curr_lvl == ins_lvl:

            node.add_entry(index_entry)

            if len(node.items) > self.max_num:
                if curr_lvl == 0:

                    # if node is too big, split leaf node
                    n1, n2, b1, b2, r, r_lvl = self.OverflowTreatment(node,
                                                                      curr_lvl,
                                                                      overflowed,
                                                                      )

                    if r:
                        for elem in r:
                            q.append((elem, r_lvl))

        else:

            # choosing parent of entry to insert
            idx_ptr, ptr_pos = self.ChooseSubTree(node,
                                                  index_entry,
                                                  curr_lvl,
                                                  )

            # index.pointer is the pointer to one of node's children nodes
            n1, n2, b1, b2, q, r_lvl = self.ChooseLeaf(node=idx_ptr.pointer,
                                                       index_entry=index_entry,
                                                       curr_lvl=curr_lvl - 1,
                                                       ins_lvl=ins_lvl,
                                                       overflowed=overflowed,
                                                       )

            # update bound
            idx_ptr.update(idx_ptr.pointer.covering)
            node.update_bound(Bound.combine([n.bound for n in node.items]))

            if n2:

                # Should be creating new indexpointers for each split created
                # These indexpointers will lie in our current node.
                node.items[ptr_pos] = IndexPointer(b2, n2)
                other_ptr = IndexPointer(b1, n1)
                node.add_entry(other_ptr)
                n2 = None

            if len(node.items) > self.max_num:

                # If the branch node has too many items split
                l1, l2, b1, b2, r, r_lvl = self.OverflowTreatment(node,
                                                                  curr_lvl,
                                                                  overflowed,
                                                                  )

                if r:
                    for elem in r:
                        q.append((elem, r_lvl))

                if l1:

                    n1 = RTree.BranchNode(items=l1, covering=b1,
                                          level=curr_lvl, plotting=self.plotting)
                    n2 = RTree.BranchNode(items=l2, covering=b2,
                                          level=curr_lvl, plotting=self.plotting)

        # returns either split leaf nodes, or branch nodes, depending on
        # which one is at the highest level of tree
        return n1, n2, b1, b2, q, r_lvl

    # inserts entry with propagation of changes upwards until root node
    # if root node is too big, we split
    def Insert(self, entry, ins_lvl=0, overflowed=set()):

        n1, n2, b1, b2, q, r_lvl = self.ChooseLeaf(node=self.root,
                                                   index_entry=entry,
                                                   curr_lvl=self.height,
                                                   ins_lvl=ins_lvl,
                                                   overflowed=overflowed,
                                                   )

        if n2:

            p1 = IndexPointer(bound=b1, pointer=n1)
            p2 = IndexPointer(bound=b2, pointer=n2)

            self.height += 1
            self.root = RTree.BranchNode(items=[],
                                         covering=Bound.combine([b1, b2]),
                                         level=self.height,
                                         plotting=self.plotting
                                         )

            self.root.add_entry(p1)
            self.root.add_entry(p2)

        if q:
            for pairs in q:
                self.Insert(pairs[0], ins_lvl=pairs[1], overflowed=overflowed)

    # removes entry from the tree
    def Delete(self, entry):

        def FindLeaf(node, index_entry, curr_lvl):

            if curr_lvl == 0:

                # returns whether entry is removed after attempting
                # to remove it
                return node.rm_entry(index_entry)

            else:

                # Set of index records to be readded in case of
                # underfull node after deletion
                q, rm_item, ins_lvl = [], False, 0

                # Findleaf on all branches that might have index_entry
                for i in range(len(node.items) - 1, -1, -1):

                    curr_item = node.items[i]

                    if curr_item.bound.contains(index_entry.bound):

                        child_node = curr_item.pointer

                        # If we found an entry and deleted it, do whats after
                        if FindLeaf(child_node, index_entry, curr_lvl=curr_lvl - 1):
                            rm_item = True

                            # delete underfull leafnodes
                            if len(child_node.items) < self.min_num:

                                q += child_node.items
                                ins_lvl = child_node.level
                                child_node.rm_plot()
                                del node.items[i]

                            else:

                                # fix indexpointer covering
                                curr_item.bound = child_node.covering

                # points to branch, update bound if childpointer was changed
                if rm_item:
                    node.update_bound(Bound.combine([n.bound for n in node.items]))

                # reinsert here
                for elem in q:
                    self.Insert(entry=elem, ins_lvl=ins_lvl)

                return rm_item

        # call recursive function
        FindLeaf(self.root, index_entry=entry, curr_lvl=self.height)

    # given a scope (search rectangle/Bound) returns list of index records
    # contained in that scope
    def Search(self, scope):

        found = []

        def helper_func(node, found):

            if type(node) is RTree.LeafNode:
                for record in node.items:
                    if scope.contains(record.bound):
                        found.append(record)

            else:
                for b in node.items:
                    if Bound.overlap(scope, b.bound) > 0:
                        helper_func(b.pointer, found)

        helper_func(self.root, found)
        return found

###############################################################################
# Testing                                                                     #
###############################################################################


def sample_point(bounds):
    rand_x = math.floor((bounds[1] - bounds[0]) * np.random.random_sample() + bounds[0])
    rand_y = math.floor((bounds[3] - bounds[2]) * np.random.random_sample() + bounds[2])
    return rand_x, rand_y


np.random.seed(123)

rtree = RTree(20, dim=2, plotting=True)


start = timeit.default_timer()

for i in range(200):

    x, y = sample_point([0, 800, 800, 0])
    ti = np.array([x, y])

    b = Bound([x, x, y, y])
    ir = IndexRecord(b, ti)

    rtree.Insert(entry=ir)

stop = timeit.default_timer()
print('Time: ', stop - start)

b = Bound([50, 300, 50, 300])
target = rtree.Search(b)
b.plot("#0000ff")

for t in target:
    rtree.Delete(t)


print(rtree)









