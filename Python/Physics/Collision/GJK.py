# GJK
import sys
import numpy as np

from Physics import RigidBody
from Physics import Shape

# ABC 上での 原点 の重心座標
def BaryCentric(A, B, C):
    # 法線
    N = np.cross(np.array(B) - A, np.array(C) - A)
    if np.isclose(N @ N, 0.0):
        return False, np.zeros(3)
    
    # 原点を ABC に射影したものを P とする
    P = N * (A @ N) / (N @ N)

    # XY, YZ, ZX 平面の内射影面積が最大のものを選択する
    Area = []
    for i in range(3):
        j = (i + 1) % 3
        k = (i + 2) % 3
        a = [A[j], A[k]]
        b = [B[j], B[k]]
        c = [C[j], C[k]]
        AB = np.array(b) - a
        AC = np.array(c) - a
        Area.append(np.linalg.det([AB, AC]))
    Idx = np.argmax(np.abs(Area))

    # 選択した平面へ射影
    x = (Idx + 1) % 3
    y = (Idx + 2) % 3
    PrjABC = [[A[x], A[y]], [B[x], B[y]], [C[x], C[y]]]
    PrjP = [P[x], P[y]]

    # 射影点と辺からなるサブ三角形の面積
    SubArea = []
    for i in range(3):
        j = (i + 1) % 3
        k = (i + 2) % 3
        SubArea.append(np.linalg.det([np.array(PrjABC[j]) - PrjP, np.array(PrjABC[k]) - PrjP]))

    # P が [A, B, C] の内部にある場合
    if np.array_equal(np.full(3, np.sign(Area[Idx])), np.sign(SubArea)):
        return True, SubArea / Area[Idx]
    
    return False, np.zeros(3)

# ABC 上での Pt の重心座標
def BaryCentricOfPoint(A, B, C,
                       Pt):
    return BaryCentric(A - Pt, B - Pt, C - Pt)

# 指定行、列を取り除いた部分行列の行列式
def Minor(Mat,
          Row, Col):
    return np.linalg.det(np.delete(np.delete(Mat, Row, axis = 0), Col, axis = 1))
# 指定行、列の余因子
def Cofactor(Mat,
             Row, Col):
    return (-1) ** (Row + Col) * Minor(Mat, Row, Col)

# 1-シンプレクス
# 線分を XYZ 軸に射影し、最大となる軸を求める
# その軸に A, B, P(原点) を射影 AB 上での P の重心座標を返す
def SignedVolume1(A, B):
    AB = np.array(B) - A
    # 原点を AB に射影したものを P とする
    P = np.array(A) - AB * (AB @ A) / (AB @ AB)

    # XYZ 軸の内絶対値が最大のものを選択する
    Idx = np.argmax(np.abs(AB))

    # 選択した軸へ射影
    PrjA = A[Idx]
    PrjB = B[Idx]
    PrjP = P[Idx]

    # P が AB の内側 (重心座標)
    if ((PrjP > PrjA and PrjP < PrjB) or (PrjP > PrjB and PrjP < PrjA)):
        return [PrjB - PrjP, PrjP - PrjA] / AB[Idx]
    # P が A 側の外側
    if ((PrjA <= PrjB and PrjP <= PrjA) or (PrjA >= PrjB and PrjP >= PrjA)):
        return [1.0, 0.0]
    # P が B 側の外側
    return [0.0, 1.0]

# 2-シンプレクス 
# 三角形を XY, YZ, ZX 平面に射影、最大となる平面を求める
# その平面に A, B, C, P(原点) を射影 ABC 上での P の重心座標を返す
# P が ABC 外の場合は 3 辺射影して一番近い 1-シンプレクスに帰着
def SignedVolume2(A, B, C):
    [b, BC] = BaryCentric(A, B, C)
    if b:
        return BC

    # 3 辺に射影して一番近いもの
    Edges = [np.array(A), np.array(B), np.array(C)]
    Lambda = []
    LenSq = []
    for i in range(3):
        j = (i + 1) % 3
        k = (i + 2) % 3

        BC = SignedVolume1(Edges[j], Edges[k])

        Lmd = np.empty(3)
        Lmd[i] = 0.0
        Lmd[j] = BC[0]
        Lmd[k] = BC[1]
        Lambda.append(Lmd)

        LS = Edges[j] * BC[0] + Edges[k] * BC[1]
        LenSq.append(LS @ LS) 

    # 一番近いものを返す
    return Lambda[np.argmin(np.abs(LenSq))]

# 3-シンプレクス
# 四面体 ABCD 内部に P(原点) があれば、その重心座標を返す
# P が ABCD 外の場合は 4 面に射影して一番近い 2-シンプレクスに帰着
def SignedVolume3(A, B, C, D):
    # 四面体 ABCD 内部にあれば重心座標を返す
    M = np.array([
        [A[0], B[0],C[0], D[0]],
        [A[1], B[1],C[1], D[1]],
        [A[2], B[2],C[2], D[2]],
        np.ones(4),
    ])
    Cof = [Cofactor(M, 3, 0), Cofactor(M, 3, 1), Cofactor(M, 3, 2), Cofactor(M, 3,3)]
    Det = np.sum(Cof)
    if np.array_equal(np.sign(np.full(4, Det)), np.sign(Cof)):
        return Cof / Det   
    
    # 4 面に射影して一番近い 2-シンプレクスに帰着
    Faces = [np.array(A), np.array(B), np.array(C), np.array(D)]
    Lambda = []
    LenSq = []
    for i in range(4):
        j = (i + 1) % 4
        k = (i + 2) % 4

        BC = SignedVolume2(Faces[i], Faces[j], Faces[k])

        Lmd = np.zeros(4)
        Lmd[i] = BC[0]
        Lmd[j] = BC[1]
        Lmd[k] = BC[2]
        Lambda.append(Lmd)

        LS = Faces[i] * BC[0] + Faces[j] * BC[1] + Faces[k] * BC[2]
        LenSq.append(LS @ LS) 
    
     # 一番近いものを返す
    return Lambda[np.argmin(np.abs(LenSq))]

# シンプレクス計算のテスト
def TestSignedVolume():
    Pts = [
        np.zeros(3),
        [1,0,0],
        [0,1,0],
        [0,0,1],
    ]
    
    # 1, 0, 0, 0
    print(SignedVolume3(Pts[0] + np.ones(3), Pts[1] + np.ones(3), Pts[2] + np.ones(3), Pts[3] + np.ones(3)))
    
    # 0.25, 0.25, 0.25, 0.25, 
    print(SignedVolume3(Pts[0] + np.full(3, -0.25), Pts[1] + np.full(3, -0.25), Pts[2] + np.full(3, -0.25), Pts[3] + np.full(3, -0.25)))
    
    # 0, 0.33, 0.33, 0.33
    print(SignedVolume3(Pts[0] + np.full(3, -1), Pts[1] + np.full(3, -1), Pts[2] + np.full(3, -1), Pts[3] + np.full(3, -1)))

    # 0.5, 0.0, 0.0, 0.5
    print(SignedVolume3(Pts[0] + np.array([1,1,-0.5]), Pts[1] + np.array([1,1,-0.5]), Pts[2] + np.array([1,1,-0.5]), Pts[3] + np.array([1,1,-0.5])))

    # 0.29, 0.302, 0.206, 0.202
    print(SignedVolume3([51.19, 26.19, 1.91], [-51.05, -26.05, -0.43], [50.89, -24.10, -1.04], [-49.10, 25.89, -1.04]))

# A, B の形状のミンコフスキー差の形状を C とした場合
# C のサポートポイントは A, B のサポートポイントの差となる
class SupportPoint:
    """SupportPoint"""

    def __init__(self, SpA, SpB):
        self.A = SpA
        self.B = SpB
        self.C = self.A - self.B
    
        self._Shape = None
    def __del__(self):
        pass

def GetSupportPoint(ShA, PosA, RotA,
                    ShB, PosB, RotB,
                    UDir, Bias):
    # A は UDir 方向に一番遠い点、B はその反対方向に一番遠い点を求める
    A = ShA.GetSupportPoint(PosA, RotA,  UDir, Bias)
    B = ShB.GetSupportPoint(PosB, RotB, -UDir, Bias)
    return SupportPoint(A, B)
