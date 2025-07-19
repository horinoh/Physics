import sys
import numpy as np

class AABB:
    """AABB"""

    def __init__(self, Mn = np.full(3, sys.float_info.max), Mx = np.full(3, -sys.float_info.max)):
        self.Min = Mn
        self.Max = Mx

    def GetCenter(self):
        return (np.array(self.Min) + self.Max) * 0.5

    def GetExtent(self):
         return np.array(self.Max) - self.Min
    
    def Expand(self, rhs):
        self.Min = [ min(self.Min[0], rhs[0]), min(self.Min[1], rhs[1]), min(self.Min[2], rhs[2]) ]
        self.Max = [ max(self.Max[0], rhs[0]), max(self.Max[1], rhs[1]), max(self.Max[2], rhs[2]) ]

class BoundEdge:
    """BoundEdge"""

    def __init__(self, Idx, Val, IsLower):
        self.Index = Idx
        self.Value = Val
        self.IsLower = IsLower
    