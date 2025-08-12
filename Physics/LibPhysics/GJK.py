import sys
from tkinter import ON
import numpy as np

from LibPhysics import RigidBody
from LibPhysics import Shape
from LibPhysics.Distance import PointTriangle, IsFrontTriangle

def GetOrtho(N):
    W = [1, 0, 0] if N[2] * N[2] > 0.9 * 0.9 else [0, 0, 1]

    V = np.cross(W, N)
    V /= np.linalg.norm(V)
    V = np.cross(N, V)
    V /= np.linalg.norm(V)

    U = np.cross(V, N)
    U /= np.linalg.norm(U)
    
    return U, V

# ABC 上での 原点 の重心座標
def BaryCentric(A, B, C):
    # 法線
    N = np.cross(np.array(B) - A, np.array(C) - A)
    if np.isclose(N @ N, 0.0):
        return False, np.zeros(3)
    
    # 原点を ABC に射影したものを P とする
    P = N * (A @ N) / (N @ N)

    # XY, YZ, ZX 各平面の内、射影面積が最大のインデックスを取得
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

    # 選択した平面へ A, B, C, P を射影
    x = (Idx + 1) % 3
    y = (Idx + 2) % 3
    PrjABC = [[A[x], A[y]], [B[x], B[y]], [C[x], C[y]]]
    PrjP = [P[x], P[y]]

    # 射影された、P と ABC の各辺からなるサブ三角形の面積を求める
    SubArea = []
    for i in range(3):
        j = (i + 1) % 3
        k = (i + 2) % 3
        SubArea.append(np.linalg.det([np.array(PrjABC[j]) - PrjP, np.array(PrjABC[k]) - PrjP]))

    # (サブ三角形の面積の符号が同じなら) P が ABC の内部にある、重心を返す
    if np.array_equal(np.full(3, np.sign(Area[Idx])), np.sign(SubArea)):
        return True, SubArea / Area[Idx]
    
    # P は ABC の外側
    return False, np.zeros(3)

# ABC 上での Pt の重心座標
def BaryCentricOfPoint(A, B, C,
                       Pt):
    # ABC 上での原点の重心座標に帰着
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
# 線分を XYZ 各軸に射影し、最大となる軸を求める
# その軸に A, B, P (原点) を射影し、AB 上での P の重心座標を返す
def SignedVolume1(A, B):
    AB = np.array(B) - A
    # 原点を AB に射影したものを P とする
    P = np.array(A) - AB * (AB @ A) / (AB @ AB)

    # XYZ 各軸の内、絶対値が最大のインデックスを取得
    Idx = np.argmax(np.abs(AB))

    # 選択した軸へ A, B, P を射影
    PrjA = A[Idx]
    PrjB = B[Idx]
    PrjP = P[Idx]

    # P が AB の内側なら重心座標を返す
    if ((PrjP > PrjA and PrjP < PrjB) or (PrjP > PrjB and PrjP < PrjA)):
        return [PrjB - PrjP, PrjP - PrjA] / AB[Idx]
    # P が A 側の外側
    if ((PrjA <= PrjB and PrjP <= PrjA) or (PrjA >= PrjB and PrjP >= PrjA)):
        return [1.0, 0.0]
    # P が B 側の外側
    return [0.0, 1.0]

# 2-シンプレクス 
# BaryCentric()
#   三角形を XY, YZ, ZX 各平面に射影、最大となる平面を求める
#   その平面に A, B, C, P (原点) を射影し、P が ABC 上にあれば P の重心座標を返す
# P が ABC 外の場合は 3 辺へ射影、一番近い 1-シンプレクスに帰着
def SignedVolume2(A, B, C):
    # P が ABC の内部にあれば重心座標を返して終了
    [b, BC] = BaryCentric(A, B, C)
    if b:
        return BC

    # P を ABC の各辺に射影して、それぞれの距離 (の2乗) を求める
    Edges = [np.array(A), np.array(B), np.array(C)]
    Lambda = []
    LenSq = []
    for i in range(3):
        j = (i + 1) % 3
        k = (i + 2) % 3

        # 1-シンプレクスに帰着
        BC = SignedVolume1(Edges[j], Edges[k])

        Lmd = np.empty(3)
        Lmd[i] = 0.0
        Lmd[j] = BC[0]
        Lmd[k] = BC[1]
        Lambda.append(Lmd)

        Pt = Edges[j] * BC[0] + Edges[k] * BC[1]
        LenSq.append(Pt @ Pt) 

    # 一番近いものを返す
    return Lambda[np.argmin(np.abs(LenSq))]

# 3-シンプレクス
# P が ABCD 内部にあれば P の重心座標を返す
# P が ABCD 外の場合は 4 面に射影、一番近い 2-シンプレクスに帰着
def SignedVolume3(A, B, C, D):
    # P が ABCD 内部にあれば重心座標を返して終了
    M = np.array([
        [A[0], B[0], C[0], D[0]],
        [A[1], B[1], C[1], D[1]],
        [A[2], B[2], C[2], D[2]],
        np.ones(4),
    ])
    Cof = [Cofactor(M, 3, 0), Cofactor(M, 3, 1), Cofactor(M, 3, 2), Cofactor(M, 3,3)]
    Det = np.sum(Cof)
    if np.array_equal(np.sign(np.full(4, Det)), np.sign(Cof)):
        return Cof / Det   
    
    # P を ABCD の各面に射影して、それぞれの距離 (の2乗) を求める
    Faces = [np.array(A), np.array(B), np.array(C), np.array(D)]
    Lambda = []
    LenSq = []
    for i in range(4):
        j = (i + 1) % 4
        k = (i + 2) % 4

        # 2-シンプレクスに帰着
        BC = SignedVolume2(Faces[i], Faces[j], Faces[k])

        Lmd = np.zeros(4)
        Lmd[i] = BC[0]
        Lmd[j] = BC[1]
        Lmd[k] = BC[2]
        Lambda.append(Lmd)

        Pt = Faces[i] * BC[0] + Faces[j] * BC[1] + Faces[k] * BC[2]
        LenSq.append(Pt @ Pt) 
    
     # 一番近いものを返す
    return Lambda[np.argmin(np.abs(LenSq))]

# ラムダ値と新しい方向ベクトルを返す (方向ベクトルの長さがほぼゼロなら交差あり)
def SignedVolume(Sps, Dir):
    match len(Sps):
        case 2:
            # シンプレクス (線分) 上へ射影したの原点の重心座標
            Lmd = SignedVolume1(Sps[0].C, Sps[1].C)
            # 原点へのベクトル
            Dir = -(Sps[0].C * Lmd[0] + Sps[1].C * Lmd[1])
        case 3:
            # シンプレクス (三角形) 上
            Lmd = SignedVolume2(Sps[0].C, Sps[1].C, Sps[2].C)
            Dir = -(Sps[0].C * Lmd[0] + Sps[1].C * Lmd[1] + Sps[2].C * Lmd[2])
        case 4:
            # シンプレクス (四面体) 上
            Lmd = SignedVolume3(Sps[0].C, Sps[1].C, Sps[2].C, Sps[3].C)
            Dir = -(Sps[0].C * Lmd[0] + Sps[1].C * Lmd[1] + Sps[2].C * Lmd[2] + Sps[3].C * Lmd[3])
    return Lmd, Dir

# Gilbert Johnson Keerthi
#   最初のサポートポイント A を (例えば) (1, 1, 1) 方向に見つける
#   原点方向 (逆方向) のサポートポイント B を見つける
#   線分 AB が原点を含めば衝突、そうでなければ線分 AB から原点方向のサポートポイントを C を見つける
#   三角形 ABC が原点を含めば衝突、そうでなければ三角形 ABC から原点方向のサポートポイント D を見つける
#   四面体 ABCD が原点を含めば衝突、そうでなければ (例えば一番近い三角形をABDとすると)三角形 ABD から原点方向のサポートポイント E を見つける (Cは破棄)
#   四面体 ABDE が原点を含むば衝突、... (この繰り返し)
#   最終的に四面体が原点を含む(衝突)か、サポートポイントが無くなる(非衝突)まで続ける
def GJK(ShA, PosA, RotA,
        ShB, PosB, RotB,
        OnIntersection, Bias, WithClosetPt = True):
    # サポートポイント保持先
    Sps = []

    # (1, 1, 1) 方向のサポートポイントを求める
    Dir = np.ones(3)
    Dir /= np.linalg.norm(Dir)
    Sps.append(GetSupportPoint(ShA, PosA, RotA,
                               ShB, PosB, RotB,
                               Dir, 0.0))
    
    # 原点に向かう方向 (逆向き) に次のサポートポイントを求める
    Dir = -Sps[0].C
    ClosestSq = sys.float_info.max
    HasIntersection = False
    Lambda = []
    while HasIntersection == False:
        Dir /= np.linalg.norm(Dir)
        Pt = GetSupportPoint(ShA, PosA, RotA, 
                             ShB, PosB, RotB, 
                             Dir, 0.0)

        # Pt は既存の点、もうこれ以上拡張できない -> 衝突無し
        if list(filter(lambda rhs: np.isclose(rhs.C, Pt.C).all(), Sps)):
            break

        # 最近接点を求めない場合は早期終了可能
        if WithClosetPt == False:
            # 新しいサポートポイント Pt が原点を超えていなければ、原点を含まない (衝突無し)
            if Dir @ Pt.C < 0.0:
                break

        # 新しいサポートポイントを追加
        Sps.append(Pt)

        # シンプレックスが原点を含む (ベクトルの長さが 0.0) なら衝突
        Lmd, Dir = SignedVolume(Sps, Dir)
        LenSq = Dir @ Dir
        if np.isclose(LenSq, 0.0):
            HasIntersection = True
            break

        # 最短距離を更新、更新できなれば終了
		# (2 つの三角形からなる四角形の対角線上に原点が位置するような場合、2 つの三角形間で切り替わり続けるループになるのを回避する為にこうしている)
        if LenSq >= ClosestSq:
            break
        ClosestSq = LenSq

        # 有効なサポートポイント (Lambda が非 0.0 ) のみを残す 
        #   enumerate で Lmd をインデックス付きにする (インデックス[0]、値[1])
        #   filter で値[1] が 0.0 でないもののみに絞り込み
        #   map で Sps[インデックス[0]] を抽出した新しいリストを作成
        Lambda = list(map(lambda rhs: [Sps[rhs[0]], rhs[1]], filter(lambda rhs: rhs[1] != 0.0, enumerate(Lmd))))
        # Sps だけ抜き出し
        Sps = list(map(lambda rhs: rhs[0], Lambda))

        # 四面体でここまで来たら原点を含む (衝突)
        HasIntersection = (4 == len(Sps))

    # 衝突確定
    if HasIntersection:
        # 成否と衝突点を返す
        return True, OnIntersection(ShA, PosA, RotA, 
                                    ShB, PosB, RotB,
                                    Sps, Bias)
    
    # 最近接点を求める場合
    if WithClosetPt:
        # 成否と最近接点を返す
        return False, [sum(map(lambda rhs: rhs[0].A * rhs[1], Lambda)), sum(map(lambda rhs: rhs[0].B * rhs[1], Lambda))]
    else:
        # 成否のみ返す
        return False, None

# 四面体にする (サポートポイント数が 4 になるまで増やす)
def ToTetrahedron(ShA, PosA, RotA,
                  ShB, PosB, RotB,
                  Sps):
    if len(Sps) == 1:
        Dir = -Sps[0].C
        Dir /= np.linalg.norm(Dir)
        Sps.append(GetSupportPoint(ShA, PosA, RotA, 
                                   ShB, PosB, RotB, 
                                   Dir, 0.0))
    if len(Sps) == 2:        
        Dir = Sps[1].C - Sps[0].C
        Dir /= np.linalg.norm(Dir)
        
        # 直交ベクトル(Orthogonal) U, V を求める
        U, V = GetOrtho(Dir)
                
        Sps.append(GetSupportPoint(ShA, PosA, RotA, 
                                   ShB, PosB, RotB, 
                                   U, 0.0))
    if len(Sps) == 3:
        AB = Sps[1].C - Sps[0].C
        AC = Sps[2].C - Sps[0].C
        Dir = np.cross(AB, AC)
        Dir /= np.linalg.norm(Dir)
        Sps.append(GetSupportPoint(ShA, PosA, RotA,
                                   ShB, PosB, RotB, 
                                   Dir, 0.0))

# Expanding Polytope Algorithm
def EPA(ShA, PosA, RotA,
        ShB, PosB, RotB,
        Sps, Bias):
    
    # EPA ではサポートポイントが 3-シンプレクス (四面体) である必要がある
    ToTetrahedron(ShA, PosA, RotA,
                  ShB, PosB, RotB,
                  Sps)

    # バイアス分拡張する (A, B の衝突点がほぼ同一の場合に法線が上手く求められない場合があるため)
    Center = sum(map(lambda rhs: rhs.C, Sps)) * 0.25
    Sps = list(map(lambda rhs: 
                   (lambda Dir = rhs.C - Center:
                    (lambda Dir = Dir / np.linalg.norm(Dir) * Bias:
                     SupportPoint(rhs.A + Dir, rhs.B - Dir)
                     )()
                    )(), Sps))
    
    # 外側に法線が向くように三角形のインデックスを生成
    Tris = []
    for i in range(4):
        j = (i + 1) % 4
        k = (i + 2) % 4
        l = (i + 3) % 4
        Tris.append([j, i, k] if IsFrontTriangle(Sps[l].C, 
                                                 Sps[i].C, Sps[j].C, Sps[k].C) else [i, j, k])
    
    while True:
        # 原点に最も近い三角形を見つける
        Tri = Tris[np.argmin(map(lambda rhs:
                                np.abs(PointTriangle(np.zeros(3), 
                                                     Sps[rhs[0]].C, Sps[rhs[1]].C, Sps[rhs[2]].C)), Tris))]
        A = Sps[Tri[0]].C
        B = Sps[Tri[1]].C
        C = Sps[Tri[2]].C

        # 三角形の法線
        N = np.cross(B - A, C - A)
        N /= np.linalg.norm(N)

        # 法線方向のサポートポイント    
        Pt = GetSupportPoint(ShA, PosA, RotA, 
                             ShB, PosB, RotB, 
                             N, Bias)

        # Pt が ABC 法線の反対側ならこれ以上拡張できない
        if (Pt.C - A) @ N <= 0.0:
            break;

        # 既存の場合これ以上拡張できない
        if list(filter(lambda rhs: np.isclose(rhs.C, Pt.C).all(), Sps)):
            break

        Sps.append(Pt)
        PtIdx = len(Sps)

        # Pt を向く三角形を削除、削除できない場合 (Lenが変わらない) は終了
        PrevLen = len(Tris)
        Tris = list(filter(lambda rhs: False == IsFrontTriangle(Pt.C, 
                                                                Sps[rhs[0]].C, Sps[rhs[1]].C, Sps[rhs[2]].C), Tris))
        TriLen = len(Tris)
        if PrevLen == TriLen:
            break

        # ぶら下がった辺 (削除した三角形群と、残った三角形群の境界となるような辺) を収集、見つからなければ終了
        DanglingEdges = []
        for i in range(TriLen):
            # 三角形 A
            TriA = Tris[i]
            # 三角形 A の 3 辺 (インデックスのペア)
            EdgesA = [
                [ TriA[0], TriA[1] ], 
                [ TriA[1], TriA[2] ], 
                [ TriA[2], TriA[0] ], 
            ]
            EdgeLenA = len(EdgesA)
            # 三角形 A の 3 辺の出現回数
            Count = [ 0, 0, 0 ]

            for j in range(TriLen):
                # 自身 (A) 以外と比較
                if i == j:
                    continue

                # 三角形 B
                TriB = Tris[j]
                # 三角形 B の 3 辺
                EdgesB = [
                    [ TriB[0], TriB[1] ], 
                    [ TriB[1], TriB[2] ], 
                    [ TriB[2], TriB[0] ], 
                ]
                EdgeLenB = len(EdgesB)

                # 出現回数をカウント
                for k in range(EdgeLenA):
                    for l in range(EdgeLenB):
                        # 逆向きも同一とみなす
                        if (EdgesA[k][0] == EdgesB[l][0] and EdgesA[k][1] == EdgesB[l][1]) or (EdgesA[k][1] == EdgesB[l][0] and EdgesA[k][0] == EdgesB[l][1]):
                            Count[k] += 1
                
                # ユニークな辺 (ぶら下がり) のみを収集
                for k in range(EdgeLenA):
                    if Count[k] == 0:
                        DanglingEdges.append(EdgesA[k])
        # そのような辺が無ければ終了
        if DanglingEdges == []:
            break

        # Pt とぶら下がり辺との新しい三角形群を (法線が外側を向くように) 追加していく
        for i in DanglingEdges:
            Tri = [PtIdx, i[1], i[0]]
            Tris.append(Tri if IsFrontTriangle(Center, Sps[Tri[0]].C, Sps[Tri[1]].C, Sps[Tri[2]].C) else [PtIdx, i[0], i[1]])
    
    # 原点に最も近い三角形(ABC)を見つける
    CTri = Tris[np.argmin(map(lambda rhs: np.abs(PointTriangle(np.zeros(3), Sps[rhs[0]].C, Sps[rhs[1]].C,Sps[rhs[2]].C)), Tris))]
    IA = CTri[0]
    IB = CTri[1]
    IC = CTri[2]
    # ABC 上での原点の重心座標を取得
    b, Lmd = BaryCentric(Sps[IA].C, Sps[IB].C, Sps[IC].C)
    return [Sps[IA].A * Lmd[0] + Sps[IB].A * Lmd[1] + Sps[IC].A * Lmd[2], Sps[IA].B * Lmd[0] + Sps[IB].B * Lmd[1] + Sps[IC].B * Lmd[2]]

# 形状 A, B のミンコフスキー差を、形状 C とした場合
# C のサポートポイントは A, B のサポートポイントの差に等しい
class SupportPoint:
    """SupportPoint"""

    def __init__(self, SpA, SpB):
        self.A = SpA
        self.B = SpB
        self.C = self.A - self.B
    
    def __del__(self):
        pass

def GetSupportPoint(ShA, PosA, RotA,
                    ShB, PosB, RotB,
                    UDir, Bias):
    # A は UDir 方向に一番遠い点、B はその反対方向に一番遠い点を求める
    A = ShA.GetSupportPoint(PosA, RotA,  UDir, Bias)
    B = ShB.GetSupportPoint(PosB, RotB, -UDir, Bias)
    return SupportPoint(A, B)

# シンプレクス計算のテスト
def TestSignedVolume():
    print("TestSignedVolume")
    
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
