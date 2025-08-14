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

            # �V���v���b�N�X���g�����Ă���̂ŁA���̕����L�����Z��
            OnA -= Ct.WNormal * Bias
            OnB += Ct.WNormal * Bias

            # �Փ˓_
            Ct.WPointA = OnA
            Ct.WPointB = OnB
            Ct.CalcLocal()
            
            return True, Ct

        # �ړ��������̏�ŉ�]���Ă���悤�ȏꍇ�A���[�v���甲���o���Ȃ���������̂Ń��[�v�ɏ���񐔂�݂���
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
            # �߂Â��Ă��Ȃ�
            break
            
        # �Փ˂���ł��낤���O�܂ł̎��Ԃ����߂�
        TimeToGo = SepDist / OrthoSpeed
        if TimeToGo > DT:
            # ���t���[���� (DeltaSec) �ɂ͏Փ˂��Ȃ�
            break
            
        # �Փ˂���ł��낤���O�܂Ŏ��Ԃ�i�߂�
        DT -= TimeToGo
        TOI += TimeToGo
        WRbA.Update(TimeToGo)
        WRbB.Update(TimeToGo)

    return False, None