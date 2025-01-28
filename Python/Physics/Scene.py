
import numpy as np

from Physics import RigidBody
from Physics import Shape
from Physics.Collision import Intersection
from Physics.Collision import Contact
from Physics.Collision.Bound import AABB
from Physics.Collision.Bound import BoundEdge

class Scene:
    """Scene"""
    def __init__(self):
        self.RigidBodies = []
    def __del__(self):
        pass

    def Update_(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            if np.isclose(i.InvMass, 0.0):
                continue
            i.LinearVelocity += Scene.Gravity * DeltaSec

        # 総当たりで衝突情報を収集する
        Len = len(self.RigidBodies)
        for i in range(Len):
            for j in range(i + 1, Len):
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
    
    # 総当たり
    def BruteForce(self, DeltaSec, ContactInfos):
        Len = len(self.RigidBodies)
        for i in range(Len):
            for j in range(i + 1, Len):
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

    # Sweep And Prune (SAP)
    def BroadPhase(self, DeltaSec, CollidablePairs):
        # ここでは射影軸を (1, 1, 1) とする
        Axis = np.ones(3)
        Axis /= np.linalg.norm(Axis)
        
        BoundEdges = []

        # 軸に射影した AABB の両端を収集
        Len = len(self.RigidBodies)
        for i in range(Len):
            Rb = self.RigidBodies[i]

            Ab = Rb.GetAABB()
            
            # 速度分拡張
            DVel = Rb.LinearVelocity * DeltaSec
            Ab.Expand(Ab.Min + DVel)
            Ab.Expand(Ab.Max + DVel)

            # 軸に射影して両端を収集
            BoundEdges.append(BoundEdge(i, Axis @ Ab.Min, True))
            BoundEdges.append(BoundEdge(i, Axis @ Ab.Max, False))
        
        # 射影値でソート
        BoundEdges = sorted(BoundEdges, key = lambda rhs: rhs.Value)

        # 射影 AABB 両端リストから、潜在的衝突ペアを収集
        Len = len(BoundEdges)
        for i in range(Len):
            A = BoundEdges[i]
            
            # A が下限なら
            if False == A.IsLower:
                continue

            # 対の上限を探す
            for j in range(i + 1, Len):
                B = BoundEdges[j]
                # 対の上限が見つかれば終了
                if A.Index == B.Index:
                    break

                # 対の上限が見つかる前に、別の下限が見つかった場合は潜在的衝突相手として収集
                if True == B.IsLower:
                    CollidablePairs.append([ A.Index, B.Index ])

    def NarrowPhase(self, DeltaSec, CollidablePairs, ContactInfos):
        for i in CollidablePairs:
            RbA = self.RigidBodies[i[0]]
            RbB = self.RigidBodies[i[1]]
            if RbA.InvMass == 0.0 and RbB.InvMass == 0.0:
                continue
            
            # 球 vs 球
            if RbA.Shape.GetType() == Shape.ShapeType.SPHERE and RbA.Shape.GetType() == Shape.ShapeType.SPHERE:
                # 衝突検出
                [b, T] = Intersection.SphereSphereDy(RbA.Position, RbA.Shape.Radius, RbA.LinearVelocity * DeltaSec,
                                                     RbB.Position, RbB.Shape.Radius, RbB.LinearVelocity * DeltaSec)
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
    
    def Update_BruteForce(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            if np.isclose(i.InvMass, 0.0):
                continue
            i.LinearVelocity += Scene.Gravity * DeltaSec

        # 衝突情報を収集、衝突時間でソート
        ContactInfos = []
        self.BruteForce(DeltaSec, ContactInfos)
        ContactInfos = sorted(ContactInfos, key = lambda rhs: rhs.TimeOfImpact)

        # TOI 毎に時間をスライスして進める
        AccumTime = 0.0
        for Ci in ContactInfos:
            if Ci.RbA.InvMass == 0.0 and Ci.RbB.InvMass == 0.0:
                continue

            Delta = Ci.TimeOfImpact - AccumTime

            # 次の衝突まで進める
            for i in self.RigidBodies:
                i.Update(Delta)
                        
            # 解決
            Contact.Resolve(Ci)

            AccumTime += Delta

        # 残りの時間分進める
        Delta = DeltaSec - AccumTime
        if Delta > 0.0:
            for i in self.RigidBodies:
                i.Update(Delta)

    def Update(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            if np.isclose(i.InvMass, 0.0):
                continue
            i.LinearVelocity += Scene.Gravity * DeltaSec

        # 衝突情報を収集、衝突時間でソート
        CollidablePairs = []
        self.BroadPhase(DeltaSec, CollidablePairs)
        ContactInfos = []
        self.NarrowPhase(DeltaSec, CollidablePairs, ContactInfos)
        
        #c = 0
        #Len = len(self.RigidBodies)
        #for i in range(Len):
        #    for j in range(i + 1, Len):
        #        c += 1
        #print(len(CollidablePairs), " vs ", c)

        # 衝突時間でソート
        ContactInfos = sorted(ContactInfos, key = lambda rhs: rhs.TimeOfImpact)

        # TOI 毎に時間をスライスして進める
        AccumTime = 0.0
        for Ci in ContactInfos:
            if Ci.RbA.InvMass == 0.0 and Ci.RbB.InvMass == 0.0:
                continue

            Delta = Ci.TimeOfImpact - AccumTime

            # 次の衝突まで進める
            for i in self.RigidBodies:
                i.Update(Delta)
                        
            # 解決
            Contact.Resolve(Ci)

            AccumTime += Delta

        # 残りの時間分進める
        Delta = DeltaSec - AccumTime
        if Delta > 0.0:
            for i in self.RigidBodies:
                i.Update(Delta)
        
    Gravity = np.array((0.0, -9.8, 0.0))
