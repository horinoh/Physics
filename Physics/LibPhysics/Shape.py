import enum
import numpy as np

class ShapeType(enum.Enum):
    BOX = enum.auto()

class ShapeBase:
    """Shape"""

    def __init__(self):
        self.CenterOfMass = np.zeros(3)

    def __del__(self):
        pass

    def GetType(self):
        return None
    
    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
        return None
    
    def GetFastestPointSpeed(self, AngVel, UDir):
        return 0.0

class ShapeBox(ShapeBase):
    """ShapeBox"""
    
    def __init__(self, Ext):
        super(ShapeBox, self).__init__()
        WR = Ext * 0.5
        HR = Ext * 0.5
        DR = Ext * 0.5
        W2 = WR * WR
        H2 = HR * HR
        D2 = DR * DR;
        self.InertiaTensor = np.array([[(H2 + D2), 0.0, 0.0], [0.0, (W2 + D2), 0.0], [0.0, 0.0, (W2 + H2) ]]) / 12.0
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

    def __del__(self):
        super(ShapeBox, self).__del__()

    def GetType(self):
        return ShapeType.BOX
