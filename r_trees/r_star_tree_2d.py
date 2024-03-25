import math
import random
import numpy as np
import matplotlib.pyplot as plt
import textwrap
import timeit


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

    def contains(self, other):
        return (self.min_x <= other.min_x) and (self.max_x >= other.max_x) and (self.min_y <= other.min_y) and (self.max_y >= other.max_y)

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

    def combine_l(bounds):
        min_x, max_x, max_y, min_y = math.inf, -math.inf, -math.inf, math.inf
        for b in bounds:
            if b.min_x < min_x:
                min_x = b.min_x
            if b.max_x > max_x:
                max_x = b.max_x
            if b.max_y > max_y:
                max_y = b.max_y
            if b.min_y < min_y:
                min_y = b.min_y
        return [min_x, max_x, max_y, min_y]

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

    def __str__(self):
        return f"{[self.min_x, self.max_x, self.max_y, self.min_y]}"

    def __repr__(self):
        return f"{[self.min_x, self.max_x, self.max_y, self.min_y]}"


class IndexRecord(object):

    def __init__(self, bound, tuple_identifier):
        self.bound = bound
        self.tuple_identifier = tuple_identifier

    def __str__(self):
        return f"val: {self.tuple_identifier}"

    def __repr__(self):
        return f"val: {self.tuple_identifier}"

    def __eq__(self, other):
        if isinstance(other, self.__class__) and np.array_equal(self.tuple_identifier, other.tuple_identifier):
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)


class IndexPointer(object):

    def __init__(self, bound, pointer):
        self.bound = bound
        self.pointer = pointer

    def update(self, bound):
        self.bound = bound

    def __str__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"

    def __repr__(self):
        return "pt " + f"{self.bound} -> {self.pointer}"


class BranchNode(object):

    def __init__(self, indices=[], covering=Bound(), level=0, plotting=False):
        self.level = level
        self.covering = covering
        self.has_overflown = False
        self.items = indices
        self.plotting = plotting

        if self.plotting:
            if self.covering:
                covering.plot("#ff0000")

    def add_entry(self, entry):
        self.items.append(entry)
        if self.covering:
            self.covering.rm_plot()
            self.covering = Bound.combine(self.covering, entry.bound)
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


class LeafNode(object):

    def __init__(self, indices=[], covering=Bound(), level=0, plotting=False):
        self.level = level
        self.items = indices
        self.covering = covering
        self.has_overflown = False
        self.plotting = plotting

        if self.plotting:
            if self.covering:
                covering.plot("#009b00")
            self.color = "#" + "".join([random.choice('ABCDEF0123456789') for i in range(6)])
            self.points = []
            for i in self.items:
                self.points.append(plt.scatter(i.tuple_identifier[0], i.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

    def plot(self):
        for i in self.items:
            self.points.append(plt.scatter(i.tuple_identifier[0], i.tuple_identifier[1], c=self.color, s=10, edgecolor='none'))

    def add_entry(self, entry):
        self.items.append(entry)
        if self.covering:
            self.covering.rm_plot()
            self.covering = Bound.combine(self.covering, entry.bound)
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
                self.update_bound(Bound(Bound.combine_l([j.bound for j in self.items])))
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


class RTree(object):

    root = LeafNode(indices=[], covering=None, level=0)

    ###########################################################################
    # Methods                                                                 #
    ###########################################################################

    def __init__(self, M, plotting=False):
        self.max_num = M
        self.min_num = math.floor(M * .4)
        self.height = 0
        self.p = min(math.floor(M * .3), 32)
        self.plotting = plotting

    def __str__(self):
        return "Root:\n" + textwrap.indent(f"{self.root}", "    ")

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

        if type(idx_ptr.pointer) is LeafNode:

            # sort by least area needed to expand and take first p entries
            node.items = sorted(node.items, key=lambda x: self.FindAddedArea(x, index_entry)[1])
            items = node.items[:self.p]

            # find the expansion that results in least overlap
            for i in range(len(items)):

                curr_ptr = items[i]
                curr_area, diff = self.FindAddedOverlap(curr_ptr, node.items, index_entry)

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
        else:

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
        return self.ChooseSplitIndex(node.items)

    def OverflowTreatment(self, node, level, parent):
        if level != self.height and not node.has_overflown:
            node.has_overflown = True
            to_insert = self.Reinsert(node, parent)
            return None, None, None, None, to_insert, level
        else:
            if type(node) is LeafNode:
                l1, l2, b1, b2 = self.Split(node)
                node.rm_plot()
                n1 = LeafNode(indices=l1, covering=b1, level=level, plotting=self.plotting)
                n2 = LeafNode(indices=l2, covering=b2, level=level, plotting=self.plotting)
                b1 = n1.covering
                b2 = n2.covering
                return n1, n2, b1, b2, None, None
            else:
                l1, l2, b1, b2 = self.Split(node)
                return l1, l2, b1, b2, None, None

    def Reinsert(self, node, parent):
        node.rm_plot()

        sort_dist = sorted(node.items, key=lambda x: np.linalg.norm(parent.covering.center - x.bound.center), reverse=True)
        node.items = sort_dist[self.p:]

        node.update_bound(Bound(Bound.combine_l([n.bound for n in node.items])))
        if type(node) is LeafNode:
            node.plot()

        return sort_dist[:self.p][::-1]

    def ChooseLeaf(self, node, index_entry, curr_level, insert_level=0, parent_node=None):

        n1, n2, b1, b2, q, re_insert_lvl = None, None, None, None, [], None

        if curr_level == insert_level:

            node.add_entry(index_entry)

            if len(node.items) > self.max_num:

                if type(node) is LeafNode:
                    # if node is too big, split leaf node and add the entry to
                    # both each of the splits
                    n1, n2, b1, b2, r, re_insert_lvl = self.OverflowTreatment(node, curr_level, parent_node)
                # else:
                #     l1, l2, b1, b2, r, re_insert_lvl = self.OverflowTreatment(node, curr_level, node)

                    if r:
                        for elem in r:
                            q.append((elem, re_insert_lvl))

        else:

            # choosing parent of entry to insert
            idx_pointer, idx_pointer_pos = self.ChooseSubTree(node, index_entry)

            # index.pointer is the pointer to one of node's children nodes
            n1, n2, b1, b2, q, re_insert_lvl = self.ChooseLeaf(node=idx_pointer.pointer,
                                                               index_entry=index_entry,
                                                               curr_level=curr_level - 1,
                                                               insert_level=insert_level, 
                                                               parent_node=node
                                                               )

            # update bound
            idx_pointer.update(idx_pointer.pointer.covering)
            node.update_bound(Bound(Bound.combine_l([n.bound for n in node.items])))

            if n2:

                # Should be creating new indexpointers for each split created
                # These indexpointers will lie in our current node.
                node.items[idx_pointer_pos] = IndexPointer(b2, n2)
                pointer_1 = IndexPointer(b1, n1)
                node.add_entry(pointer_1)
                n2 = None

            if len(node.items) > self.max_num:

                # If the branch node has too many items split
                l1, l2, b1, b2, r, re_insert_lvl = self.OverflowTreatment(node, curr_level, node)

                if r:
                    for elem in r:
                        q.append((elem, re_insert_lvl))

                if l1:
                    node.rm_plot()
                    n1 = BranchNode(indices=l1, covering=b1, level=curr_level, plotting=self.plotting)
                    n2 = BranchNode(indices=l2, covering=b2, level=curr_level, plotting=self.plotting)

        # returns either split leaf nodes, or branch nodes, depending on
        # which one is at the highest level of tree
        return n1, n2, b1, b2, q, re_insert_lvl

    # inserts entry with propagation of changes upwards until root node
    # if root node is too big, we split
    def Insert(self, entry, insert_level=0):

        n1, n2, b1, b2, q, re_insert_lvl = self.ChooseLeaf(node=self.root,
                                                           index_entry=entry,
                                                           curr_level=self.height,
                                                           insert_level=insert_level,
                                                           )

        if n2:

            self.height += 1
            p1 = IndexPointer(bound=b1, pointer=n1)
            p2 = IndexPointer(bound=b2, pointer=n2)
            self.root = BranchNode(indices=[],
                                   covering=Bound.combine(b1, b2),
                                   level=self.height,
                                   plotting=self.plotting
                                   )
            self.root.add_entry(p1)
            self.root.add_entry(p2)

        if q:
            for pairs in q:
                self.Insert(pairs[0], insert_level=pairs[1])

    # removes entry from the tree
    def Delete(self, entry):

        def FindLeaf(node, index_entry, curr_level):

            if curr_level == 0:

                # returns whether entry is removed after attempting
                # to remove it
                return node.rm_entry(index_entry)

            else:

                # Set of index records to be readded in case of
                # underfull node after deletion
                q, rm_item, ins_level = [], False, 0

                # Findleaf on all branches that might have index_entry
                for i in range(len(node.items) - 1, -1, -1):

                    curr_item = node.items[i]

                    if curr_item.bound.contains(index_entry.bound):

                        child_node = curr_item.pointer

                        # If we found an entry and deleted it, do whats after
                        if FindLeaf(child_node, index_entry, curr_level=curr_level - 1):
                            rm_item = True

                            # delete underfull leafnodes
                            if len(child_node.items) < self.min_num:

                                q += child_node.items
                                ins_level = child_node.level
                                child_node.rm_plot()
                                del node.items[i]

                            else:

                                # fix indexpointer covering
                                curr_item.bound = child_node.covering

                # points to branch, update bound if childpointer was changed
                if rm_item:
                    node.update_bound(Bound(Bound.combine_l([n.bound for n in node.items])))

                # reinsert here
                for elem in q:
                    self.Insert(entry=elem, insert_level=ins_level)

                return rm_item

        # call recursive function
        FindLeaf(self.root, index_entry=entry, curr_level=self.height)


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

rtree = RTree(40, plotting=True)
items = []
start = timeit.default_timer()

for i in range(8000):
    x, y = sample_point([0, 800, 800, 0])
    ti1 = np.array([x, y])
    b1 = Bound([x, x, y, y])
    i1 = IndexRecord(b1, ti1)
    # i == 3521
    rtree.Insert(entry=i1, insert_level=0)

# for i in range(len(items)):
#     rtree.Delete(items[i])
#     rtree.Insert(entry=items[i], insert_level=0)

stop = timeit.default_timer()

print('Time: ', stop - start)
print(rtree)

# Fix overflow treatment: somehow duplicating pointers
# This is probably because Split is called twice
# when you overflow, you also want to reset directory page bound
# once in the reinsert algorithm and another outside the reinsert
# Bad performance when there are more than 2
# layers to tree
# Find added overlap operation is expensive also
# print(rtree)

print("Done!")































