import numpy as np

class AABB:
    """AABB"""

    def __init__(self, Mn = np.zeros(3), Mx = np.zeros(3)):
        self.Min = Mn
        self.Max = Mx

    @property
    def Center(self):
        return (self.Min + self.Max) * 0.5
    @property    
    def Extent(self):
         return self.Max - self.Min
    
    def Expand(self, rhs):
        self.Min = [ min(self.Min[0], rhs[0]), min(self.Min[1], rhs[1]), min(self.Min[2], rhs[2]) ]
        self.Max = [ max(self.Max[0], rhs[0]), max(self.Max[1], rhs[1]), max(self.Max[2], rhs[2]) ]

class BoundEdge:
    """BoundEdge"""

    def __init__(self, Index, ProjValue, IsLower):
        self.Index = Index
        self.Value = ProjValue
        self.IsLower = IsLower
    