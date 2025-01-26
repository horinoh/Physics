# 接触
import numpy as np

from Physics import RigidBody

class Info:
    """Info"""

    def __init__(self):
        self.TimeOfImpact = 0.0
        # 剛体
        self.RbA = None
        self.RbB = None
        # 接点 (ローカル)
        #self.LOnA = np.zeros(3)
        #self.LOnB = np.zeros(3)
        # 接点 (ワールド)
        self.WOnA = np.zeros(3)
        self.WOnB = np.zeros(3)
        # A -> B へのワールド法線
        self.WNrm = np.zeros(3)

def ResolveLinear(Ci):
    TotalInvMass = Ci.RbA.InvMass + Ci.RbB.InvMass
    TotalElasticity = 1.0 + Ci.RbA.Elasticity * Ci.RbB.Elasticity

    J = (Ci.RbB.LinearVelocity - Ci.RbA.LinearVelocity) * TotalElasticity / (TotalInvMass + 0)
    Ci.RbA.ApplyLinearImpulse(J)
    Ci.RbB.ApplyLinearImpulse(-J)

    AB = Ci.WOnB - Ci.WOnA
    Ci.RbA.Position += AB * Ci.RbA.InvMass / TotalInvMass
    Ci.RbB.Position -= AB * Ci.RbB.InvMass / TotalInvMass

def Resolve(Ci):    
    TotalInvMass = Ci.RbA.InvMass + Ci.RbB.InvMass
    TotalElasticity = 1.0 + Ci.RbA.Elasticity * Ci.RbB.Elasticity
    TotalFriction = Ci.RbA.Friction * Ci.RbB.Friction

    # 重心と衝突点との半径
    RadA = Ci.WOnA - Ci.RbA.WorldCenterOfMass
    RadB = Ci.WOnB - Ci.RbB.WorldCenterOfMass

    # 逆慣性テンソル
    InvInerA = Ci.RbA.ToWorld(Ci.RbA.InvInertiaTensor)
    InvInerB = Ci.RbB.ToWorld(Ci.RbB.InvInertiaTensor)

    # 相対速度
    VelA = Ci.RbA.LinearVelocity + np.cross(Ci.RbA.AngularVelocity, RadA)
    VelB = Ci.RbB.LinearVelocity + np.cross(Ci.RbB.AngularVelocity, RadB)
    VelAB = VelB - VelA

    # 相対速度を法線、接線成分に分解
    NVelAB = Ci.WNrm * (VelAB @ Ci.WNrm)
    TVelAB = VelAB - NVelAB

    def Apply(Axis, Vel, Coef):
        JA = np.cross(InvInerA @ np.cross(RadA, Axis), RadA)
        JB = np.cross(InvInerB @ np.cross(RadB, Axis), RadB)
        AngFactor = (JA + JB) @ Axis
        J = Vel * Coef / (TotalInvMass + AngFactor)
        Ci.RbA.ApplyImpulse(Ci.WOnA, J)
        Ci.RbB.ApplyImpulse(Ci.WOnB, -J)

    # 力積 (法線)
    Apply(Ci.WNrm, NVelAB, TotalElasticity)

    # 力積 (接線)
    LenSq = TVelAB @ TVelAB
    if False == np.isclose(LenSq, 0.0):
        WTan = TVelAB / np.sqrt(LenSq)
        Apply(WTan, TVelAB, TotalFriction)

    # 追い出し処理
    if Ci.TimeOfImpact == 0.0:
        AB = Ci.WOnB - Ci.WOnA
        Ci.RbA.Position += AB * Ci.RbA.InvMass / TotalInvMass
        Ci.RbB.Position -= AB * Ci.RbB.InvMass / TotalInvMass
