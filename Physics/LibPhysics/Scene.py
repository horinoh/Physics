import numpy as np
import copy

from LibPhysics import RigidBody
from LibPhysics import Contact
from LibPhysics import Bound
from LibPhysics import GJK

class Scene:
    """Scene"""

    def __init__(self):
        self.Shapes = []
        
        self.RigidBodies = []
        
        self.Constraints = []
        self.Manifold = None

        self.SAPAxis = np.ones(3)
        self.SAPAxis /= np.linalg.norm(self.SAPAxis)

    def __del__(self):
        pass

    # SAP (Sweep And Prune)
    def BroadPhase(self, DeltaSec, CollidablePairs):
        BoundEdges = []
        for i in range(len(self.RigidBodies)):
             Rb = self.RigidBodies[i]
             Ab = Rb.GetAABB()
             
             DVel = Rb.LinearVelocity * DeltaSec
             Ab.Expand(Ab.Min + DVel)
             Ab.Expand(Ab.Max + DVel);
             
             BoundEdges.append(Bound.BoundEdge(i, self.SAPAxis @ Ab.Min, True))
             BoundEdges.append(Bound.BoundEdge(i, self.SAPAxis @ Ab.Max, False))
        BoundEdges = sorted(BoundEdges, key = lambda rhs: rhs.Value)
        
        Len = len(BoundEdges)
        for i in range(Len):
            A = BoundEdges[i]
            if not A.IsLower:
                continue
            for j in range(i + 1, Len):
                B = BoundEdges[j]
                if A.Index == B.Index:
                    break
                if B.IsLower:
                    CollidablePairs.append([ A.Index, B.Index ])

    def Intersection(DeltaSec, RbA, RbB):
        WRbA = copy.deepcopy(RbA)
        WRbB = copy.deepcopy(RbB)

        DT = DeltaSec
        TOI = 0.0
        ItCount = 0
        Bias = 0.001

        while DT > 0.0:
            b, [OnA, OnB] = GJK.GJK(WRbA.Shape, WRbA.Position, WRbA.Rotation,
                                    WRbB.Shape, WRbB.Position, WRbB.Rotation,
                                    GJK.EPA, Bias)
            if b:
                Ct = Contact()

                Ct.RigidBodyA = RbA
                Ct.RigidBodyB = RbB
                
                Ct.TimeOfImpact = TOI

                Ct.WNormal = OnB - OnA
                Ct.WNormal /= np.linalg.norm(Ct.WNormal)

                # シンプレックスを拡張しているので、その分をキャンセル
                OnA -= Ct.WNormal * Bias
                OnB += Ct.WNormal * Bias

                # 衝突点
                Ct.WPointA = OnA
                Ct.WPointB = OnB
                Ct.CalcLocal()

                return True, Ct

            # 移動せずその場で回転しているような場合、ループから抜け出さない事があるのでループに上限回数を設ける
            ItCount += 1
            if ItCount > 10:
                break
        
            AB = OnB - OnA
            SepDist = np.linalg.norm(AB)

            Dir = AB / SepDist
            
            LVel = (WRbA.LinearVelocity - WRbB.LinearVelocity) @ Dir
            AVel = WRbA.Shape.GetFastestPointSpeed(WRbA.AngularVelocity, Dir) - WRbB.Shape.GetFastestPointSpeed(WRbB.AngularVelocity, Dir)
            OrthoSpeed = LVel + AVel
            if OrthoSpeed <= 0.0:
                # 近づいていない
                break
            
            # 衝突するであろう直前までの時間を求める
            TimeToGo = SepDist / OrthoSpeed
            if TimeToGo > DT:
		        # DT 以内には存在しない
                break
            
            # 衝突するであろう直前まで時間を進める
            DT -= TimeToGo
            TOI += TimeToGo
            WRbA.Update(TimeToGo)
            WRbB.Update(TimeToGo)
        return False, None

    def NarrowPhase(self, DeltaSec, CollidablePairs, Contacts):
        for i in CollidablePairs:
            RbA = self.RigidBodies[i[0]]
            RbB = self.RigidBodies[i[1]]
            if RbA.InvMass == 0.0 and RbB.InvMass == 0.0:
                continue
            
            # b, Ct = Scene.Intersection(DeltaSec, RbA, RbB)
            # if b:
            #     #if Ct.TimeOfImpact == 0.0:
            #     #    self.Manifold.Add(Ct)
            #     #else:
            #     Contacts.append(Ct)

    def SolveConstraints(self, DeltaSec, IterationCount):
        pass

    def ResolveContact(self, Contact):
        TotalInvMass = Contact.RigidBodyA.InvMass + Contact.RigidBodyB.InvMass

        RA = Contact.WPointA - Contact.RigidBodyA.GetWorldCenterOfMass()
        RB = Contact.WPointB - Contact.RigidBodyB.GetWorldCenterOfMass()

        InvITA = Contact.RigidBodyA.GetWorldInverseInertiaTensor()
        InvITB = Contact.RigidBodyB.GetWorldInverseInertiaTensor()

        VelA = Contact.RigidBodyA.LinearVelocity + np.cross(Contact.RigidBodyA.AngularVelocity, RA)
        VelB = Contact.RigidBodyB.LinearVelocity + np.cross(Contact.RigidBodyB.AngularVelocity, RB)
        RelVelA = VelA - VelB;

        def Apply(Axis, Vel, Coef):
            AngJA = np.cross(InvITA @ np.cross(RA, Axis), RA)
            AngJB = np.cross(InvITB @ np.cross(RB, Axis), RB)
            AngFactor = (AngJA + AngJB) @ Axis
            J = Vel * Coef / (TotalInvMass + AngFactor)
            Contact.RigidBodyA.ApplyImpulse(Contact.WPointA, -J)
            Contact.RigidBodyB.ApplyImpulse(Contact.WPointB, J)

        VelN = Contact.WNormal * (RelVelA @ Contact.WNormal)
        TotalElas = 1.0 + Contact.RigidBodyA.Elasticity * Contact.RigidBodyB.Elasticity
        Apply(Contact.WNormal, VelN, TotalElas)

        VelT = RelVelA - VelN
        Tan = VelT.normalized()
        TotalFric = Contact.RigidBodyA.Friction * Contact.RigidBodyB.Friction;
        Apply(Tan, VelT, TotalFric)

    def ConservativeAdvacne(self, DeltaSec, Contacts):
        # TOI 毎に時間をスライスして進める
        AccumTime = 0.0
        for i in Contacts:
            if i.RbA.InvMass == 0.0 and i.RbB.InvMass == 0.0:
                continue
            Delta = i.TimeOfImpact - AccumTime
            
            # 次の衝突まで進める
            for j in self.RigidBodies:
                j.Update(Delta)

            # 衝突解決
            self.ResolveContact(i)

            AccumTime += Delta
      
        # 残りの時間を進める
        Delta = DeltaSec - AccumTime
        if Delta > 0.0:
            for i in self.RigidBodies:
                i.Update(Delta)

    def Update(self, DeltaSec):
        # 重力
        for i in self.RigidBodies:
            i.ApplyGravity(DeltaSec)

        # 衝突情報を収集
        Contacts = []
        CollidablePairs = []
        self.BroadPhase(DeltaSec, CollidablePairs)
        self.NarrowPhase(DeltaSec, CollidablePairs, Contacts)

        # 制約解決 (Constraints, Manifold)
        self.SolveConstraints(DeltaSec, 5)

        #
        self.ConservativeAdvacne(DeltaSec, Contacts)

