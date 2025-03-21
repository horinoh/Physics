import sys
import numpy as np

from Physics import RigidBody
from Physics.Collision.Contact import Info
from Physics.Collision.GJK import GetOrtho

class ConstraintBase:
    """ConstraintBase"""

    def __init__(self, RbA, RbB):
        # 質量行列       
        self.InvMass = np.zeros((12, 12))
        self.Velocities = np.empty(12)

    def __del__(self):
        pass         

    # 質量行列
    # M = (MA   0   0   0) ... A の質量の逆数が対角成分 (3x3)
    #     (    IA   0   0) ... A の慣性テンソルの逆行列 (3x3)
    #     ( 0   0  MB   0) ... B の質量の逆数が対角成分
    #     ( 0   0   0  IB) ... B の慣性テンソルの逆行列
    def GetInvMassMatrix(RbA, RbB):

        InvMass = np.empty((12, 12))
        
        # MA
        InvMass[0:4, 0:4] = np.diag(np.full(3, RbA.InvMass))
        # IA
        InvMass[3:7, 3:7] = RbA.InvInertiaTensor
        # MB
        InvMass[6:10, 6:10] = np.diag(np.full(3, RbB.InvMass))
        # IB
        InvMass[9:13, 9:13] = RbB.InvInertiaTensor
        
        return InvMass
    
    # V = (VA) ... A の速度
    #     (WA) ... A の角速度
    #     (VB) ... B の速度
    #     (WB) ... B の角速度
    def GetVelocities(RbA, RbB):
        Velocities = np.empty(12)
    
        Velocities[0:4] = RbA.LinearVelocity
        Velocities[3:7] = RbA.AngularVelocity
        Velocities[6:10] = RbB.LinearVelocity
        Velocities[9:13] = RbB.AngularVelocity
    
        return Velocities
    
    def ApplyImpulse(RbA, RbB, Impulse):
        RbA.ApplyLinearImpulse(Impulse[0:4])
        RbA.ApplyAngularImpulse(Impulse[3:7])
        
        RbB.ApplyLinearImpulse(Impulse[6:10])
        RbB.ApplyAngularImpulse(Impulse[9:13])

    def PreSolve(self, DeltaSec):
        pass
    def Solve(self):
        pass
    def PostSolve(self):
        pass

class Distance(ConstraintBase):
    """Distance"""
    # RbA, RbB を繋ぐアンカー位置をワールド座標で渡す
    def __init__(self, RbA, RbB, WAnchor):
        super(ConstraintBase, self).__init__()

        self.RbA = RbA
        self.RbB = RbB
        self.InvMass = self.GetInvMassMatrix(self.RbA, self.RbB)

        # 初期アンカー位置はそれぞれのローカル座標で覚えておく
        self.AnchorA = RbA.ToLocalPos(WAnchor)
        self.AnchorB = RbB.ToLocalPos(WAnchor)

        # ヤコビ行列 (n * 12) n == コンストレイント数, 12 == 6 (移動3、回転3) 軸の自由度 * 2 オブジェクト
        # ここでは n == 1
        self.Jacobian = np.empty((1, 12))
        # 前回のコンストレイントを保持しておき、新フレームのコンストレイントを計算する前に適用することで、少ないシミュレーション回数で収束させる
        self.CachedLambda = 0.0

        # 適正な位置へ戻すような力を適用する事で位置ドリフトを修正 (Baumgarte stabilization)
        # 一気にやるとシステムにエネルギーを追加しすぎる為、数フレームかけて適用する
        self.Baumgarte = 0.0

    def __del__(self):
        super(ConstraintBase, self).__del__()
    
    def PreSolve(self, DeltaSec):
        # オブジェクトが動くことを考慮して、都度ワールドへ変換
        WAnchorA = self.RbA.ToWorldPos(self.AnchorA)
        WAnchorB = self.RbB.ToWorldPos(self.AnchorB)

        # アンカーを結ぶベクトル
        AB = WAnchorB - WAnchorA
        # アンカーへの半径
        RA = WAnchorA - self.RbA.WorldCenterOfMass()
        RB = WAnchorB - self.RbB.WorldCenterOfMass()

        # ヤコビ行列を作成
        J1 = -AB * 2.0
        J2 = np.cross(RA, J1)
        J3 = -J1
        J4 = np.cross(RB, J3)
        self.Jacobian[0:4] = J1
        self.Jacobian[3:7] = J2
        self.Jacobian[6:10] = J3
        self.Jacobian[9:13] = J4

        # 前のフレームの力 (CachedLambda) を今フレームの計算前に適用することで、少ないフレームで安定状態へ持っていく (Warm starting)
        self.ApplyImpulse(self.Jacobian * self.CachedLambda)

        # 適正な位置へ戻すような力を適用する事で位置ドリフトを修正 (Baumgarte stabilization)
        C = max(AB @ AB - 0.01, 0.0)
        self.Baumgarte = 0.05 * C / DeltaSec

    def Solve(self):
        JT = np.transpose(self.Jacobian)
        A = JT @ self.InvMass @ self.Jacobian
        B = -JT @ self.Velocities - self.Baumgarte
        
        Lambda = GaussSiedel(A, B)

        self.ApplyImpulse(self.Jacobian * Lambda)

        self.CachedLambda += Lambda

    def PostSolve(self):
        self.CachedLambda = np.clip(self.CachedLambda, -sys.float_info.epsilon, sys.float_info.epsilon)

class Penetration(ConstraintBase):
    """Penetration"""
    def __init__(self, ):
        super(ConstraintBase, self, Ci).__init__()

        self.RbA = Ci.RbA
        self.RbB = Ci.RbB

        self.InvMass = self.GetInvMassMatrix(self.RbA, self.RbB)

        self.AnchorA = self.RbA.ToLocalPos(Ci.WOnA)
        self.AnchorB = self.RbB.ToLocalPos(Ci.WOnB)

        # 法線 (A視点)
        self.Normal = np.linalg.norm(self.RbA.ToLocalDir(Ci.WNrm))

        self.Friction = self.RbA.Friction * self.RbB.Friction

        # ヤコビ行列 (n * 12) n == コンストレイント数, 12 == 6 (移動3、回転3) 軸の自由度 * 2 オブジェクト
        # ここでは n == 3
        self.Jacobian = np.empty((3, 12))
        self.CachedLambda = np.zeros(3)

        self.Baumgarte = 0.0

    def __del__(self):
        super(ConstraintBase, self).__del__()
    
    def PreSolve(self, DeltaSec):
        # オブジェクトが動くことを考慮して、都度ワールドへ変換
        WAnchorA = self.RbA.ToWorldPos(self.AnchorA)
        WAnchorB = self.RbB.ToWorldPos(self.AnchorB)

        RA = WAnchorA - self.RbA.WorldCenterOfMass()
        RB = WAnchorB - self.RbB.WorldCenterOfMass()

        # オブジェクトが動くことを考慮して、都度ワールドへ変換
        WNrm = self.RbA.ToWorldDir(self.Normal)

        # ヤコビ行列を作成
        J1 = -WNrm
        J2 = np.cross(RA, J1)
        J3 = -J1
        J4 = np.cross(RB, J3)
        self.Jacobian[0][0:4] = J1
        self.Jacobian[0][3:7] = J2
        self.Jacobian[0][6:10] = J3
        self.Jacobian[0][9:13] = J4

        # 摩擦
        if self.Friction > 0.0:
            # 直交ベクトル U, V を求める
            U, V = GetOrtho(WNrm)

            J1 = -U
            J2 = np.cross(RA, J1)
            J3 = -J1
            J4 = np.cross(RB, J3)
            self.Jacobian[1][0:4] = J1
            self.Jacobian[1][3:7] = J2
            self.Jacobian[1][6:10] = J3
            self.Jacobian[1][9:13] = J4

            J1 = -V
            J2 = np.cross(RA, J1)
            J3 = -J1
            J4 = np.cross(RB, J3)
            self.Jacobian[2][0:4] = J1
            self.Jacobian[2][3:7] = J2
            self.Jacobian[2][6:10] = J3
            self.Jacobian[2][9:13] = J4
        
        self.ApplyImpulse(self.Jacobian * self.CachedLambda)

        C = (WAnchorB - WAnchorA) @ WNrm
        C = min(C + 0.02, 0.0)
        self.Baumgarte = 0.25 * C / DeltaSec

    def Solve(self):
        JT = np.transpose(self.Jacobian)
        A = JT @ self.InvMass @ self.Jacobian
        #B = -JT @ self.Velocities - np.array(self.Baumgarte, 0.0, 0.0)
        B = -JT @ self.Velocities
        B[0] -= self.Baumgarte
        
        Lambda = GaussSiedel(A, B)

        PreCL = self.CachedLambda
        self.CachedLambda += Lambda
        self.CachedLambda[0] = max(self.CachedLambda[0], 0.0)
        if self.Friction > 0.0:
            uMg = self.Friction * abs(9.8 / (self.RbA.InvMass + self.RbB.InvMass))
            NFrc = abs(Lambda[0] * self.Friction)
            Frc = max(uMg, NFrc)
            self.CachedLambda = np.clip(self.CachedLambda, -Frc, Frc)
        Lambda = self.CachedLambda - PreCL

        self.ApplyImpulse(self.Jacobian * Lambda)

# Linear Complimentary Problem (LCP)
#	A * x = b 
#	行列 A, ベクトル b が既知の時、未知のベクトル x を求める

# ガウスザイデル法では以下のいずれかの場合に LCP を解くことができる (これから扱う行列は対角優位なので、ガウスザイデル法が使用可能)
#	[1] 正定値 (positive definite) 
#		v^t * M * v > 0 
#		ベクトル v の転地と v で挟むように掛けたときに結果が正となるような行列 M
#
#	[2] 対角優位 (diagonally dominant) 
#		|xy_ij| >= Sigma_j,j!=i |xy_ij|
#		対角成分の和の絶対値が、非対角成分の和の絶対値以上となるような行列
def GaussSiedel(A, b, ItCount = 10):
    Len = len(b)
    x = np.zeros(Len)
    
    for it in range(ItCount):
        for i in range(Len):
            x[i] += (b[i] - A[i] @ x) / A[i][i]
    
    return x

def TestGaussSiedel():
    print("TestGaussSiedel")

    A = np.array([
        [3, 1],
        [2, 4],
    ])

    b = np.array([6, 7])

    x = GaussSiedel(A, b)

    # 1.7, 0.9
    print(x)