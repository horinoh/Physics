
import numpy as np

from Physics import RigidBody
from Physics import Shape
from Physics.Collision import Intersection
from Physics.Collision import Contact

class Scene:
    """Scene"""
    def __init__(self):
        self.RigidBodies = []
    def __del__(self):
        pass

    def Update2(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            if np.isclose(i.InvMass, 0.0):
                continue
            i.LinearVelocity += Scene.Gravity * DeltaSec

        # 総当たりで衝突情報を収集する
        for i in range(len(self.RigidBodies)):
            for j in range(i + 1, len(self.RigidBodies)):
                RbA = self.RigidBodies[i]
                RbB = self.RigidBodies[j]
                if RbA.InvMass == 0.0 and RbB.InvMass == 0.0:
                    continue

                # 球 vs 球
                if RbA.Shape.GetType() == Shape.ShapeType.SPHERE and RbA.Shape.GetType() == Shape.ShapeType.SPHERE:
                    # 衝突検出
                    if Intersection.SphereSphere(RbA.Position, RbA.Shape.Radius,
                                                 RbB.Position, RbB.Shape.Radius):
                        # 接触情報を収集
                        Ci = Contact.Info()
                        
                        Ci.RbA = RbA
                        Ci.RbB = RbB
                        
                        # 法線
                        AB = Ci.RbB.Position - Ci.RbA.Position
                        Ci.WNrm = AB / np.linalg.norm(AB)

                        # 接点
                        Ci.WOnA = Ci.RbA.Position + Ci.WNrm * Ci.RbA.Shape.Radius
                        Ci.WOnB = Ci.RbB.Position - Ci.WNrm * Ci.RbB.Shape.Radius

                        # 解決
                        Contact.Resolve(Ci)
                        
        for i in self.RigidBodies:
            i.Update(DeltaSec)

    def Update1(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            if np.isclose(i.InvMass, 0.0):
                continue
            i.LinearVelocity += Scene.Gravity * DeltaSec

        # 総当たりで衝突情報を収集する
        for i in range(len(self.RigidBodies)):
            for j in range(i + 1, len(self.RigidBodies)):
                RbA = self.RigidBodies[i]
                RbB = self.RigidBodies[j]
                if RbA.InvMass == 0.0 and RbB.InvMass == 0.0:
                    continue

                # 球 vs 球
                if RbA.Shape.GetType() == Shape.ShapeType.SPHERE and RbA.Shape.GetType() == Shape.ShapeType.SPHERE:
                    # 衝突検出
                    [b, T] = Intersection.SphereSphereDy(RbA.Position, RbA.Shape.Radius, RbA.LinearVelocity * DeltaSec,
                                                         RbB.Position, RbB.Shape.Radius, RbB.LinearVelocity * DeltaSec)
                    if b:
                        T *= DeltaSec
                        # 接触情報を収集
                        Ci = Contact.Info()
                        
                        Ci.RbA = RbA
                        Ci.RbB = RbB
                        
                        Ci.RbA.Update(T)
                        Ci.RbB.Update(T)

                        # 法線
                        AB = Ci.RbB.Position - Ci.RbA.Position
                        Ci.WNrm = AB / np.linalg.norm(AB)

                        # 接点
                        Ci.WOnA = Ci.RbA.Position + Ci.WNrm * Ci.RbA.Shape.Radius
                        Ci.WOnB = Ci.RbB.Position - Ci.WNrm * Ci.RbB.Shape.Radius

                        Ci.RbA.Update(-T)
                        Ci.RbB.Update(-T)

                        # 解決
                        Contact.Resolve(Ci)
                        
        for i in self.RigidBodies:
            i.Update(DeltaSec)

    def Update(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            if np.isclose(i.InvMass, 0.0):
                continue
            i.LinearVelocity += Scene.Gravity * DeltaSec

        ContactInfos = []

        # 総当たりで衝突情報を収集する
        for i in range(len(self.RigidBodies)):
            for j in range(i + 1, len(self.RigidBodies)):
                RbA = self.RigidBodies[i]
                RbB = self.RigidBodies[j]
                if RbA.InvMass == 0.0 and RbB.InvMass == 0.0:
                    continue

                # 球 vs 球
                if RbA.Shape.GetType() == Shape.ShapeType.SPHERE and RbA.Shape.GetType() == Shape.ShapeType.SPHERE:
                    # 衝突検出
                    [b, T] = Intersection.SphereSphereDy(RbA.Position, RbA.Shape.Radius, RbA.LinearVelocity,
                                                         RbB.Position, RbB.Shape.Radius, RbB.LinearVelocity)
                    if b:
                        # 接触情報を収集
                        Ci = Contact.Info()
                        
                        Ci.TimeOfImpact = T * DeltaSec
                        Ci.RbA = RbA
                        Ci.RbB = RbB

                        # 法線
                        AB = Ci.RbB.Position - Ci.RbA.Position
                        Ci.WNrm = AB / np.linalg.norm(AB)

                        # 接点
                        Ci.WOnA = Ci.RbA.Position + Ci.WNrm * Ci.RbA.Shape.Radius
                        Ci.WOnB = Ci.RbB.Position - Ci.WNrm * Ci.RbB.Shape.Radius
                        
                        ContactInfos.append(Ci)

        # 衝突時間でソート
        ContactInfos = sorted(ContactInfos, key = lambda rhs: rhs.TimeOfImpact)
        #if len(ContactInfos) > 0:
         #   print("C", len(ContactInfos))

        # TOI 毎に時間をスライスして進める
        AccumTime = 0.0
        for i in ContactInfos:
            if i.RbA.InvMass == 0.0 and i.RbB.InvMass == 0.0:
                continue

            Delta = i.TimeOfImpact - AccumTime

            # 次の衝突まで進める
            for j in self.RigidBodies:
                #print("A", Delta)
                j.Update(Delta)
                        
            # 解決
            Contact.Resolve(i)

            AccumTime += Delta

        # 残りの時間分進める
        Delta = DeltaSec - AccumTime
        #print("B", Delta)
        if Delta > 0.0:
            for i in self.RigidBodies:
                i.Update(Delta)
        
    Gravity = np.array((0.0, -9.8, 0.0))
