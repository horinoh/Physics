from abc import ABCMeta, abstractmethod
import numpy as np
import quaternion
from Physics.Collision.Bound import AABB

import enum

class ShapeType(enum.Enum):
    SPHERE = enum.auto(),
    BOX = enum.auto(),
    CONVEX = enum.auto(),

class ShapeBase:
    """Shape"""

    def __init__(self):
        self.CenterOfMass = np.zeros(3)
    def __del__(self):
        pass

    @abstractmethod
    def GetType(self):
        return None

    @abstractmethod
    def GetAABB(self, Pos, Rot):
        return None
    
    # 指定の方向に最も遠い点を返す
    @abstractmethod
    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
        return None
    
    # 指定の方向に最も速く動いている点の速度
    def GetFastestPointSpeed(self, AngVel, Dir):
        return 0.0

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
    
    def GetAABB(self, Pos, Rot):
        return AABB(np.full(3, -self.Radius) + Pos, np.full(3, self.Radius) + Pos)
    
    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
	    return Pos + UDir * (self.Radius + Bias)
        
class ShapeBox(ShapeBase):
    """ShapeBox"""
    
    def __init__(self, Ext = np.ones(3)):
        super(ShapeBox, self).__init__()
        self.Extent = Ext
        X, Y, Z = np.power(self.Extent, 2.0)
        self.InertiaTensor = np.diag(np.array([Y + Z, X + Z, X + Y]) / 12.0)
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

        X, Y, Z = np.array(self.Extent) * 0.5
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
        Ab = AABB()
        for i in self.Points:
            Ab.Expand(quaternion.rotate_vectors(Rot, i) + Pos)
        return Ab

    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
        # [移動回転した点, 点と UDir の内積] のリストを返す (map())
        # 内積 [1] が最大になるような要素を見つける (max())
        # 点 [0] に UDir * Bias を加算して返す
        return max(list(map(lambda rhs: (lambda Pt = quaternion.rotate_vectors(Rot, rhs) + Pos: [Pt, Pt @ UDir])(), self.Points)), key = lambda rhs: rhs[1])[0] + UDir * Bias
    
    def GetFastestPointSpeed(self, AngVel, UDir):
        return max(self.Points, key = lambda rhs: UDir @ np.cross(AngVel, rhs))

class ShapeConvex(ShapeBase):
    """ShapeConvex"""
    
    def __init__(self):
        super(ShapeConvex, self).__init__()
        #self.InertiaTensor = 
        #self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)
        self.Points = []

    def __del__(self):
        super(ShapeConvex, self).__del__()

    def GetType(self):
        return ShapeType.CONVEX

    def GetAABB(self, Pos, Rot):
        Ab = AABB()
        for i in self.Points:
            Ab.Expand(quaternion.rotate_vectors(Rot, i) + Pos)
        return Ab

    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
        return max(self.Points, key = lambda rhs: (quaternion.rotate_vectors(Rot, rhs) + Pos) @ UDir) + UDir * Bias
    
    def GetFastestPointSpeed(self, AngVel, UDir):
        return max(self.Points, key = lambda rhs: UDir @ np.cross(AngVel, rhs))
