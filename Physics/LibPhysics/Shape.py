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

    def GetParallelAxisTheoremTensor(self):
        R = -self.CenterOfMass
        R2 = np.dot(R, R)
        XX = R[0] * R[0]
        XY = R[0] * R[1]
        XZ = R[0] * R[2]
        YY = R[1] * R[1]
        YZ = R[1] * R[2]
        ZZ = R[2] * R[2]
        return np.array([
            [R2 - XX,      XY,      XZ],
            [     XY, R2 - YY,      YZ],
            [     XZ,      YZ, R2 - ZZ]
        ])

    def CalcCenterOfMass(self):
        return np.zeros(3)

    def CalcInertiaTensor(self):
        return np.identity(3)

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

        self.Points = []
        self.Points.append(np.array([-X, -Y, -Z]))
        self.Points.append(np.array([ X, -Y, -Z]))
        self.Points.append(np.array([-X,  Y, -Z]))
        self.Points.append(np.array([-X, -Y,  Z]))

        self.Points.append(np.array([ X,  Y,  Z]))
        self.Points.append(np.array([-X,  Y,  Z]))
        self.Points.append(np.array([ X, -Y,  Z]))
        self.Points.append(np.array([ X,  Y, -Z]))

        self.InertiaTensor = self.CalcInertiaTensor() + self.GetParallelAxisTheoremTensor()
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

    def __del__(self):
        super(ShapeBox, self).__del__()

    def GetType(self):
        return ShapeType.BOX

    def GetAABB(self, Pos, Rot):
        Ab = Bound.AABB()
        for i in self.Points:
            Ab.Expand(quaternion.rotate_vectors(Rot, i) + Pos)
        return Ab

    def CalcInertiaTensor(self):
        Ext = np.array([self.Points[1][0] - self.Points[0][0], self.Points[2][1] - self.Points[1][1], self.Points[3][2] - self.Points[2][2]])
        X = Ext[0] * 0.5
        Y = Ext[1] * 0.5
        Z = Ext[2] * 0.5
        W2 = X * X
        H2 = Y * Y
        D2 = Z * Z
        return np.array([[(H2 + D2), 0.0, 0.0], [0.0, (W2 + D2), 0.0], [0.0, 0.0, (W2 + H2) ]]) / 12.0

    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
        return max(list(map(lambda rhs: (lambda Pt = quaternion.rotate_vectors(Rot, rhs) + Pos: [Pt, Pt @ UDir])(), self.Points)), key = lambda rhs: rhs[1])[0] + UDir * Bias
    
    def GetFastestPointSpeed(self, AngVel, UDir):
        return max(map(lambda rhs: UDir @ np.cross(AngVel, rhs), self.Points))


