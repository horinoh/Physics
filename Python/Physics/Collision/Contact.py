# 接触
import numpy as np
import math # math.sqrt の方が numpy.sqrt より速いらしい
import copy

from Physics import RigidBody
from Physics import Shape
from Physics.Collision import Intersection
from Physics.Collision.GJK import GJK, EPA

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

def GetContactInfoSphere(RbA, RbB, DeltaSec):
    # 衝突検出
    [b, T] = Intersection.SphereSphereDy(RbA.Position, RbA.Shape.Radius, RbA.LinearVelocity * DeltaSec,
                                         RbB.Position, RbB.Shape.Radius, RbB.LinearVelocity * DeltaSec)
    if b:
        # 接触情報を収集
        Ci = Info()
                        
        Ci.TimeOfImpact = T * DeltaSec
        Ci.RbA = RbA
        Ci.RbB = RbB

        PosA = Ci.RbA.Position + Ci.RbA.LinearVelocity * Ci.TimeOfImpact
        PosB = Ci.RbB.Position + Ci.RbB.LinearVelocity * Ci.TimeOfImpact
        
        # 法線 (A視点)
        AB = PosB - PosA
        Ci.WNrm = AB / np.linalg.norm(AB)

        # 接点
        Ci.WOnA = PosA + Ci.WNrm * Ci.RbA.Shape.Radius
        Ci.WOnB = PosB - Ci.WNrm * Ci.RbB.Shape.Radius

        return Ci
    return None

def GetContactInfo(RbA, RbB, DeltaSec):
    # 球 vs 球 の場合は専用の処理
    if RbA.Shape.GetType() == Shape.ShapeType.SPHERE and RbA.Shape.GetType() == Shape.ShapeType.SPHERE:
        return GetContactInfoSphere(RbA, RbB, DeltaSec)
    
    # TOI が直接求まらないので、シミュレーションを進めることで求める (Conservative Advance)
    
    # ワークでシミュレーションを進める
    WRbA = copy.deepcopy(RbA)
    WRbB = copy.deepcopy(RbB)

    DT = DeltaSec
    TOI = 0.0
    ItCount = 0
    while DT > 0.0:
        Res = []
        b, Res = GJK(WRbA.Shape, WRbA.Position, WRbA.Rotation,
                     WRbB.Shape, WRbB.Position, WRbB.Rotation, 
                     True)
        # GJK で衝突していれば EPA に帰着
        if b:
            IPos = []
            IPos = EPA(WRbA.Shape, WRbA.Position, WRbA.Rotation,
                       WRbB.Shape, WRbB.Position, WRbB.Rotation,
                       Res, 0.1)
            Ci = Info()
            Ci.TimeOfImpact = TOI
            Ci.RbA = RbA
            Ci.RbB = RbB
            Ci.WOnA = IPos[0]
            Ci.WOnB = IPos[1]
            AB = Ci.WOnB - Ci.WOnA
            Ci.WNrm = AB / np.linalg.norm(AB)
            return Ci

        # 移動せずその場で回転しているような場合、ループから抜け出さない事があるのでループに上限回数を設ける
        ItCount += 1
        if ItCount > 10:
            return None

        # 最近接点を結ぶ AB        
        AB = Res[1] - Res[0]
        SepDist = np.linalg.norm(AB)
        AB /= SepDist

        # 線形相対速度
        RelVel = (WRbA.LinearVelocity - WRbB.LinearVelocity) @ AB

        # 角相対速度
        RelVel += WRbA.Shape.GetFastestPointSpeed(WRbA.AngularVelocity, AB) - WRbB.Shape.GetFastestPointSpeed(WRbB.AngularVelocity, AB)

        # 近づいてなければ早期終了
        if RelVel < 0.0 or math.isclose(RelVel, 0.0):
            return None
        
        # 進めても良い時間 (衝突寸前まで)
        ToAdv = SepDist / RelVel

        # デルタ時間内に起こらないなら早期終了
        if DT > ToAdv:
            return None
        
        DT -= ToAdv
        TOI += ToAdv
        
        # シミュレーションを進める
        WRbA.Update(ToAdv)
        WRbB.Update(ToAdv)

    return None

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

    # 力積用補助関数
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
        WTan = TVelAB / math.sqrt(LenSq)
        Apply(WTan, TVelAB, TotalFriction)

    # 追い出し処理
    if Ci.TimeOfImpact == 0.0:
        AB = Ci.WOnB - Ci.WOnA
        Ci.RbA.Position += AB * Ci.RbA.InvMass / TotalInvMass
        Ci.RbB.Position -= AB * Ci.RbB.InvMass / TotalInvMass

# 2 オブジェクト間の接触情報
class Manifold:
    """Manifold"""

    def __init__(self):
        # 剛体
        self.RbA = None
        self.RbB = None
        # 接触情報と貫通コンストレイント
        self.Constraints = []