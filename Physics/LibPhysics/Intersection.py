import numpy as np
import copy

from LibPhysics import RigidBody
from LibPhysics import Contact
from LibPhysics import GJK

def ConservativeAdvance(DeltaSec, RbA, RbB):
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
        AVel = WRbA.Shape.GetFastestRotatingPointSpeed(WRbA.AngularVelocity, Dir) - WRbB.Shape.GetFastestRotatingPointSpeed(WRbB.AngularVelocity, Dir)
        OrthoSpeed = LVel + AVel
        if OrthoSpeed <= 0.0:
            # 近づいていない
            break
            
        # 衝突するであろう直前までの時間を求める
        TimeToGo = SepDist / OrthoSpeed
        if TimeToGo > DT:
            # 今フレーム中 (DeltaSec) には衝突しない
            break
            
        # 衝突するであろう直前まで時間を進める
        DT -= TimeToGo
        TOI += TimeToGo
        WRbA.Update(TimeToGo)
        WRbB.Update(TimeToGo)

    return False, None