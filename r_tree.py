import math
import numpy as np
import textwrap


class Bound(object):
    def __init__(self, bounds=[0, 0, 0, 0]):
        self.min_x = bounds[0]
        self.max_x = bounds[1]
        self.max_y = bounds[2]
        self.min_y = bounds[3]
        self.length = self.max_x - self.min_x
        self.width = self.max_x - self.min_x
        self.area = self.width * self.length

    # returns bounds and area
    def expand(self, other):
        min_x = min(self.min_x, other.min_x)
        max_x = max(self.max_x, other.max_x)
        max_y = max(self.max_y, other.max_y)
        min_y = min(self.min_y, other.min_y)
        return [min_x, max_x, max_y, min_y]

    def combine(self, other):
        bounds = self.expand(other)
        return Bound(bounds)

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

    def __repr__(self):
        return f"(pt -> {self.pointer})"


class BranchNode(object):
    def __init__(self, indices=[]):
        self.items = indices

    def add_entry(self, entry):
        self.items.append(entry)

    def __repr__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Branch:\n" + textwrap.indent(string, "    ")


class LeafNode(object):
    def __init__(self, indices=[], covering=Bound()):
        self.covering = covering
        self.items = indices

    def add_entry(self, entry):
        self.items.append(entry)
        self.covering = self.covering.combine(entry.bound)

    def __repr__(self):
        string = ""
        for i in self.items:
            string += str(i) + "\n"
        return "Leaf\n" + textwrap.indent(string, "    ")


class RTree(object):

    ###########################################################################
    # Methods                                                                 #
    ###########################################################################

    def __init__(self, M):
        self.root = LeafNode(indices=[], covering=Bound())
        self.max_num = M
        self.min_num = math.floor(M // 2)

    def __repr__(self):
        return "Root:\n" + textwrap.indent(f"{self.root}", "    ")

    def ChooseLeaf(self, node, index_entry):
        n1, n2, b1, b2 = None, None, None, None
        if type(node) is LeafNode:
            if len(node.items) < self.max_num:
                node.items.append(index_entry)
                node.covering = node.covering.combine(index_entry.bound)
            else:
                l1, l2, b1, b2 = self.SplitNode(node, self.min_num)
                n1 = LeafNode(indices=l1, covering=b1)
                n2 = LeafNode(indices=l2, covering=b2)
                n1.add_entry(index_entry)
                n2.add_entry(index_entry)
        else:
            min_expansion = math.inf
            min_area = math.inf
            child_node = None
            child_node_idx = 0
            items = node.items
            for i in range(len(items)):
                curr_node = node.items[i]
                bounds = curr_node.bound.expand(index_entry.bound)
                exp_area = (bounds[1] - bounds[0]) * (bounds[2] - bounds[3])
                curr_area = curr_node.bound.area
                diff = exp_area - curr_area
                if diff <= min_expansion:
                    min_expansion = diff
                    if curr_area < min_area:
                        child_node = curr_node
                        child_node_idx = i
                        min_area = curr_area

            # child_node.pointer is the pointer of one of node's children
            n1, n2, b1, b2 = self.ChooseLeaf(child_node.pointer, index_entry)

            # update bound
            child_node.bound = child_node.bound.combine(index_entry.bound)

            if n2:
                # Should be creating new indexpointers for each split created
                # These indexpointers will lie in our current node.

                # in general, n2 might not be a leaf node, so what do we mean
                # by covering fix this part.
                node.items[child_node_idx] = IndexPointer(b2, n2)
                pointer_1 = IndexPointer(b1, n1)
                node.add_entry(pointer_1)
                n2 = None

            if len(node.items) > self.max_num:
                l1, l2, b1, b2 = self.SplitNode(node, self.min_num)
                n1 = BranchNode(indices=l1)
                n2 = BranchNode(indices=l2)
                return n1, n2, b1, b2

        return n1, n2, b1, b2

    # takes in a node to split, and minimum number of entries
    # required in each new node.
    # Returns two lists of nodes split from node.items
    # along with their bounding box.
    def SplitNode(self, node, m):
        remaining = set(node.items)

        # pick first element of our each new group
        n1, n2, b1, b2 = [], [], Bound(), Bound()
        r1, r2 = self.PickSeeds(node.items)
        n1.append(r1), n2.append(r2)

        remaining.remove(r1), remaining.remove(r2)
        b1, b2 = r1.bound, r2.bound
        while True:
            if len(remaining) == 0:
                break
            if len(n1) >= m:
                for s in remaining:
                    n2.append(s)
                    b2 = b2.combine(s.bound)
                break
            if len(n2) >= m:
                for s in remaining:
                    n1.append(s)
                    b1 = b1.combine(s.bound)
                break

            r_new, pref = self.PickNext(remaining, b1, b2)
            if pref == 1:
                n1.append(r_new)
                b1 = b1.combine(r_new.bound)
            else:
                n2.append(r_new)
                b2 = b2.combine(r_new.bound)

            remaining.remove(r_new)
        return n1, n2, b1, b2

    def PickSeeds(self, entries):
        iefficieny = -math.inf
        length = len(entries)
        r1 = None
        r2 = None
        for i in range(length):
            for j in range(length - i - 1):
                bounds = entries[i].bound.expand(entries[i + j + 1].bound)
                exp_area = (bounds[1] - bounds[0]) * (bounds[2] - bounds[3])
                curr_ieff = exp_area - entries[i].bound.area - entries[i + j + 1].bound.area
                if curr_ieff > iefficieny:
                    iefficieny = curr_ieff
                    r1 = entries[i]
                    r2 = entries[i + j + 1]
        return r1, r2

    def PickNext(self, entries, b1, b2):
        max_diff = -1
        r = None
        r_pref = 0
        for entry in entries:
            bound_1 = entry.bound.expand(b1)
            exp_area_1 = (bound_1[1] - bound_1[0]) * (bound_1[2] - bound_1[3])
            bound_2 = entry.bound.expand(b2)
            exp_area_2 = (bound_2[1] - bound_2[0]) * (bound_2[2] - bound_2[3])
            pref = 0
            if exp_area_1 < exp_area_2:
                pref = 1
            else:
                pref = 2
            diff = abs(exp_area_1 - exp_area_2)
            if diff > max_diff:
                max_diff = diff
                r = entry
                r_pref = pref
        return r, r_pref

    # def AdjustTree(self, node):

    def insert(self, index_entry):
        n1, n2, b1, b2 = self.ChooseLeaf(self.root, index_entry)
        if n2:
            self.root = BranchNode(indices=[])
            p1 = IndexPointer(b1, pointer=n1)
            p2 = IndexPointer(b2, pointer=n2)
            self.root.add_entry(p1)
            self.root.add_entry(p2)

###############################################################################
# Testing                                                                     #
###############################################################################


rtree = RTree(5)
for i in range(4000):
    b1 = Bound([0, i, i, 0])
    ti1 = np.array([.5 * i, .5 * i])
    i1 = IndexRecord(b1, ti1)
    rtree.insert(i1)

print(rtree)
print("Done!")





















