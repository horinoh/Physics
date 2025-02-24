from abc import ABCMeta, abstractmethod
import numpy as np
import quaternion
import Physics.Collision
from Physics.Collision.Bound import AABB
from Physics.Collision.Volume import Tetrahedron

import enum

import Physics.Collision.Volume
import Physics.InertiaTensor

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
        self.InertiaTensor = Physics.InertiaTensor.Sphere(self.Radius)
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

    def __del__(self):
        super(ShapeSphere, self).__del__()

    def GetType(self):
        return ShapeType.SPHERE
    
    def GetAABB(self, Pos, Rot):
        return AABB(np.full(3, -self.Radius) + Pos, np.full(3, self.Radius) + Pos)
    
    def GetSupportPoint(self, Pos, Rot, UDir, Bias):
	    return Pos + UDir * (self.Radius + Bias)
        
class ShapeConvexBase(ShapeBase):
    """ShapeConvexBase"""

    def __init__(self):
        super(ShapeConvexBase, self).__init__()
        self.Points = []

    def __del__(self):
        super(ShapeConvexBase, self).__del__()

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
        return max(map(lambda rhs: UDir @ np.cross(AngVel, rhs), self.Points))

class ShapeBox(ShapeConvexBase):
    """ShapeBox"""
    
    def __init__(self, Ext = np.ones(3)):
        super(ShapeBox, self).__init__()

        self.Extent = Ext
        X, Y, Z = np.power(self.Extent, 2.0)
        self.InertiaTensor = Physics.InertiaTensor.Box(X, Y, Z)
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

        X, Y, Z = np.array(self.Extent) * 0.5
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

class ShapeConvex(ShapeConvexBase):
    """ShapeConvex"""
    
    def __init__(self):
        super(ShapeConvex, self).__init__()

        Vertices = []
        Indices = []
        self.CenterOfMass = self.CalcCenterOfMass(Vertices, Indices)
        self.InertiaTensor = self.CalcInertiaTensor(Vertices, Indices, self.CenterOfMass)
        self.InvInertiaTensor = np.linalg.inv(self.InertiaTensor)

    def __del__(self):
        super(ShapeConvex, self).__del__()

    def GetType(self):
        return ShapeType.CONVEX

    def CalcCenterOfMass(self, Vertices, Indices):
        MeshCenter = sum(Vertices) / len(Vertices)
        
        TotalCenter = 0.0
        TotalVolume = 0.0
        
        Len = len(Indices) // 3
        for i in range(Len):
            A = MeshCenter
            B = Indices[i * 3 + 0]
            C = Indices[i * 3 + 1]
            D = Indices[i * 3 + 2]

            Volume = Physics.Collision.Volume.Tetrahedron(A, B, C, D)

            TotalCenter += (A + B + C + D) * 0.25 * Volume
            TotalVolume += Volume

        return TotalCenter / TotalVolume

    def CalcInertiaTensor(self, Vertices, Indices, CenterOfMass):
        TotalInertiaTensor = 0.0
        TotalVolume = 0.0

        Len = len(Indices) // 3
        for i in range(Len):
            A = np.zeros(3)
            B = Indices[i * 3 + 0] - CenterOfMass
            C = Indices[i * 3 + 1] - CenterOfMass
            D = Indices[i * 3 + 2] - CenterOfMass

            TotalInertiaTensor += Physics.InertiaTensor.Tetrahedron(A, B, C, D)
            TotalInertiaTensor += Physics.Collision.Volume.Tetrahedron(A, B, C, D)
        
        return TotalInertiaTensor / TotalVolume

    def Diamond():
        Rad = 2.0 * np.pi * 0.125
        Rot = quaternion.from_euler_angles(0, 0, Rad * 0.5)
        Pts = [
            [0.1, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.1],
            [0.4, 0.0, 0.],
        ]
        Pts.append(quaternion.rotate_vectors(Rot, [0.8, 0.0, 0.3]))
        Pts.append(quaternion.rotate_vectors(Rot, Pts[1]))
        Pts.append(quaternion.rotate_vectors(Rot, Pts[2]))

        Rot = quaternion.from_euler_angles(0, 0, Rad)
        Accm = quaternion.identity()
        for i in range(1, 8):
            Accm = Accm * Rot
            for j in range(7):
                Pts.append(quaternion.rotate_vectors(Accm, Pts[j]))
        return Pts
