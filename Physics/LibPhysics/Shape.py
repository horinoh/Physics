import enum
import numpy as np
import quaternion

from LibPhysics import Bound

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
    
    def GetAABB(self, Pos, Rot):
        return None

    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
        return None
    
    def GetFastestPointSpeed(self, AngVel, UDir):
        return 0.0

class ShapeBox(ShapeBase):
    """ShapeBox"""
    
    def __init__(self, Ext):
        super(ShapeBox, self).__init__()
        X = Ext * 0.5
        Y = Ext * 0.5
        Z = Ext * 0.5
        W2 = X * X
        H2 = Y * Y
        D2 = Z * Z
        self.InertiaTensor = np.array([[(H2 + D2), 0.0, 0.0], [0.0, (W2 + D2), 0.0], [0.0, 0.0, (W2 + H2) ]]) / 12.0
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

        self.Points = []
        self.Points.append(np.array([-X, -Y, -Z]))
        self.Points.append(np.array([ X, -Y, -Z]))
        self.Points.append(np.array([-X,  Y, -Z]))
        self.Points.append(np.array([-X, -Y,  Z]))

        self.Points.append(np.array([ X,  Y,  Z]))
        self.Points.append(np.array([-X,  Y,  Z]))
        self.Points.append(np.array([ X, -Y,  Z]))
        self.Points.append(np.array([ X,  Y, -Z]))

    def __del__(self):
        super(ShapeBox, self).__del__()

    def GetType(self):
        return ShapeType.BOX

    def GetAABB(self, Pos, Rot):
        Ab = Bound.AABB()
        for i in self.Points:
            Ab.Expand(quaternion.rotate_vectors(Rot, i) + Pos)
        return Ab


