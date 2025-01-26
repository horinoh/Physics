import numpy as np

import enum

class ShapeType(enum.Enum):
    NONE = enum.auto(),
    SPHERE = enum.auto(),
    BOX = enum.auto(),

class ShapeBase:
    """Shape"""

    def __init__(self):
        self.CenterOfMass = np.zeros(3)
    def __del__(self):
        pass
    def GetType(self):
        return ShapeType.NONE

class ShapeSphere(ShapeBase):
    """ShapeSphere"""
    
    def __init__(self, Rad = 1.0):
        super(ShapeSphere, self).__init__()
        self.Radius = Rad
        self.InertiaTensor = np.diag(np.full(3, 2.0 / 5.0 * pow(self.Radius, 2.0)))
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)
    def __del__(self):
        super(ShapeSphere, self).__del__()
    def GetType(self):
        return ShapeType.SPHERE
