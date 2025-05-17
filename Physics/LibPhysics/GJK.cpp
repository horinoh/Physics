#include "GJK.h"
#include "RigidBody.h"
#include "Shape.h"
#include "Collision.h"
#include "Convex.h"
#include "Util.h"

#include "Log.h"

//!< 1-シンプレクス (線分) ... 原点のシンプレックス上での重心座標 (シンプレックス上にあるか) を返す
Math::Vec2 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B)
{
	const auto AB = B - A;
	//!< 原点を AB 軸に射影し P とする
	const auto P = A + AB * AB.Dot(-A) / AB.LengthSq(); //!< 平方根を省けるので正規化しない方が良い

	//!< 線分 AB を X, Y, Z 軸 へ射影したそれぞれの線分
	std::vector<float> Segs;
	std::ranges::transform(static_cast<const Math::Component3&>(AB), std::back_inserter(Segs),
		[](const auto& rhs) { 
			return std::abs(rhs); 
		});
	//!< 線分が最長となる軸のインデックス
	const auto Index = static_cast<int>(std::ranges::max_element(Segs) - std::cbegin(Segs));
	
	//!< P、線分 AB を選択した軸へ射影
	const auto PrjA = A[Index], PrjB = B[Index], PrjP = P[Index];

	//!< P が 線分 AB の内部にあれば重心座標を返す
	if ((PrjP > PrjA && PrjP < PrjB) || (PrjP > PrjB && PrjP < PrjA)) {
		return Math::Vec2(PrjB - PrjP, PrjP - PrjA) / AB[Index];
	}

	//!< 外側確定、P が A 側か B 側か
	return (PrjP <= PrjA) ? Math::Vec2::AxisX() : Math::Vec2::AxisY();
}

//!< 2-シンプレクス (三角形)
Math::Vec3 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	//!< ABC 上での 原点 の重心座標
	const auto BC = Barycentric(A, B, C);
	if (BC != std::nullopt) {
		return BC.value();
	}
	
	//!< YZ, ZX, XY 平面
	constexpr std::array XYZ = { 0, 1, 2 };
	constexpr auto XYZSize = static_cast<int>(std::size(XYZ));

	//!< P を 3 辺に射影して一番近いものを見つける (1-シンプレクス に帰着)
	const std::array Edges = { A, B, C };
	//!< 3 辺に対して、重心座標と距離(二乗)のペアを収集
	std::vector<std::pair<Math::Vec2, float>> LmdLen;
	std::ranges::transform(XYZ, std::back_inserter(LmdLen),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;
			
			const auto Lmd = SignedVolume(Edges[j], Edges[k]);
			return std::pair<Math::Vec2, float>({ Lmd, (Edges[j] * Lmd[0] + Edges[k] * Lmd[1]).LengthSq() });
		});
	//!< 距離が最小となる辺とそのインデックス
	const auto MinIt = std::ranges::min_element(LmdLen,
		[](const auto& lhs, const auto& rhs) {
			return lhs.second < rhs.second;
		});
	const auto Index = static_cast<int>(MinIt - std::cbegin(LmdLen));

	//!< 重心座標を Vec3 として返す
	Math::Vec3 Lambda;
	Lambda[(Index + 0) % XYZSize] = 0.0f;
	Lambda[(Index + 1) % XYZSize] = MinIt->first[0];
	Lambda[(Index + 2) % XYZSize] = MinIt->first[1];
	return Lambda;
}

//!< 3-シンプレクス (四面体)
Math::Vec4 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D)
{
	//!< 四面体を座標系と考える、その逆(転置)行列の四因子
	const auto M = Math::Mat4(Math::Vec4(A, 1.0f), Math::Vec4(B, 1.0f), Math::Vec4(C, 1.0f), Math::Vec4(D, 1.0f)).Transpose();
	const auto Cofactors = Math::Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
	const auto& CofCmps = static_cast<const Math::Component4&>(Cofactors);
	const auto Det = std::accumulate(std::cbegin(CofCmps), std::cend(CofCmps), 0.0f);
	//!< 原点が四面体内部にあれば、重心座標を返す
	if (std::ranges::all_of(CofCmps,
		[&](const auto rhs) {
			return std::signbit(Det) == std::signbit(rhs);
		})) {
		return Cofactors / Det;
	}

	constexpr std::array XYZW = { 0, 1, 2, 3 };
	constexpr auto XYZWSize = static_cast<int>(std::size(XYZW));

	//!< 4 面に射影して一番近いものを見つける (2-シンプレクス に帰着)
	const std::array Faces = { A, B, C, D };
	//!< 4 面に対して、重心座標と距離(二乗)のペアを収集
	std::vector<std::pair<Math::Vec3, float>> LmdLen;
	std::ranges::transform(XYZW, std::back_inserter(LmdLen),
		[&](const auto& rhs) {
			const auto i = rhs;
			const auto j = (rhs + 1) % XYZWSize;
			const auto k = (rhs + 2) % XYZWSize;

			const auto Lmd = SignedVolume(Faces[i], Faces[j], Faces[k]);
			return std::pair<Math::Vec3, float>({ Lmd, (Faces[i] * Lmd[0] + Faces[j] * Lmd[1] + Faces[k] * Lmd[2]).LengthSq() });
		});
	//!< 距離が最小となる面とそのインデックス
	const auto MinIt = std::ranges::min_element(LmdLen,
		[](const auto& lhs, const auto& rhs) {
			return lhs.second < rhs.second;
		});
	const auto Index = static_cast<int>(MinIt - std::cbegin(LmdLen));
	
	Math::Vec4 Lambda;
	Lambda[(Index + 0) % XYZWSize] = MinIt->first[0];
	Lambda[(Index + 1) % XYZWSize] = MinIt->first[1];
	Lambda[(Index + 2) % XYZWSize] = MinIt->first[2];
	Lambda[(Index + 3) % XYZWSize] = 0.0f;
	return Lambda;
}

std::optional<Math::Vec3> Collision::Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	const auto N = Math::Vec3::Normal(A, B, C);
	if (N.NearlyEqual(Math::Vec3::Zero())) {
		return std::nullopt;
	}

	//!< YZ, ZX, XY 平面
	constexpr std::array XYZ = { 0, 1, 2 }; 
	constexpr auto XYZSize = static_cast<int>(std::size(XYZ));

	//!< 原点を 三角形 ABC に射影し P とする
	const auto P = N * A.Dot(N) / N.LengthSq();

	//!< 三角形 ABC を YZ, ZX, XY 平面へ射影したそれぞれの面積
	std::vector<float> Areas;
	std::ranges::transform(XYZ, std::back_inserter(Areas),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;

			const auto a = Math::Vec2(A[j], A[k]);
			const auto b = Math::Vec2(B[j], B[k]);
			const auto c = Math::Vec2(C[j], C[k]);
			const auto AB = b - a;
			const auto AC = c - a;
			return Math::Mat2(AB, AC).Determinant();
		});
	//!< 面積の絶対値が最大となる平面とそのインデックス
	const auto AbsMaxIt = std::ranges::max_element(Areas,
		[](const auto& lhs, const auto& rhs) {
			return std::abs(lhs) < std::abs(rhs);
		});
	const auto Index = static_cast<int>(AbsMaxIt - std::cbegin(Areas));

	//!< P、三角形 ABC を選択した平面に射影 (Z が選択された場合は XY 平面といった具合になる)
	const auto X = (Index + 1) % XYZSize;
	const auto Y = (Index + 2) % XYZSize;
	const std::array PrjABC = { Math::Vec2(A[X], A[Y]), Math::Vec2(B[X], B[Y]), Math::Vec2(C[X], C[Y]) };
	const auto PrjP = Math::Vec2(P[X], P[Y]);

	//!< 射影点と辺からなるサブ三角形それぞれの面積
	std::vector<float> SubAreas;
	std::ranges::transform(XYZ, std::back_inserter(SubAreas),
		[&](const auto& rhs) {
			const auto j = (rhs + 1) % XYZSize;
			const auto k = (rhs + 2) % XYZSize;
			return Math::Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
		});

	//!< P が 三角形 ABC の内部にあれば (サブ三角形の面積の符号から判断)、重心座標を返す
	if (std::ranges::all_of(SubAreas,
		[&](const auto rhs) {
			return std::signbit(Areas[Index]) == std::signbit(rhs);
		})) {
		return Math::Vec3(SubAreas[0], SubAreas[1], SubAreas[2]) / Areas[Index];
	}

	return std::nullopt;
}
std::optional<Math::Vec3> Collision::Barycentric(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	//!< ABC から Pt を引くことで、原点の重心座標へ帰着
	return Barycentric(A - Pt, B - Pt, C - Pt);
}

//!< A, B のサポートポイント、及びその差 C を求める
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	const Math::Vec3& UDir, const float Bias)
{
	return { ShA->GetSupportPoint(PosA, RotA, UDir, Bias), ShB->GetSupportPoint(PosB, RotB, -UDir, Bias) };
}
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	const Math::Vec3& UDir, const float Bias)
{
	return Get(RbA->Shape, RbA->Position, RbA->Rotation, RbB->Shape, RbB->Position, RbB->Rotation, UDir, Bias);
}

void Collision::SupportPoint::ToTetrahedron(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, 
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	std::vector<SupportPoint::Points>& Sps)
{
	if (1 == std::size(Sps)) {
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, -Sps[0].GetC().Normalize(), 0.0f));
	}
	if (2 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		Math::Vec3 U, V;
		AB.GetOrtho(U, V);
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, U, 0.0f));
	}
	if (3 == std::size(Sps)) {
		const auto AB = Sps[1].GetC() - Sps[0].GetC();
		const auto AC = Sps[2].GetC() - Sps[0].GetC();
		Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, AB.Cross(AC).Normalize(), 0.0f));
	}
}
void Collision::SupportPoint::ToTetrahedron(const Physics::RigidBody* RbA,
	const Physics::RigidBody* RbB, 
	std::vector<Collision::SupportPoint::Points>& Sps)
{
	ToTetrahedron(RbA->Shape, RbA->Position, RbA->Rotation, RbB->Shape, RbB->Position, RbB->Rotation, Sps);
}

void Collision::SupportPoint::Expand(const float Bias, std::vector<Collision::SupportPoint::Points>& Sps)
{
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(), 
		[](const auto& Acc, const auto& i) { 
			return Acc + i.GetC(); 
		}) / static_cast<float>(std::size(Sps));
	std::ranges::transform(Sps, std::begin(Sps),
		[&](const auto& i) {
			const auto Dir = (i.GetC() - Center).Normalize() * Bias;
			return Collision::SupportPoint::Points(i.GetA() + Dir, i.GetB() - Dir);
		});
}

bool Collision::Intersection::GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB,
	OnIntersectGJK OnIntersect, const float Bias,
	const bool WithClosestPoint,
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	std::vector<Collision::SupportPoint::Points> Sps;
	Sps.reserve(4); //!< 4 枠

	//!< (1, 1, 1) 方向のサポートポイントを求める
	Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, Math::Vec3::One().Normalize(), 0.0f));

	//!< 原点を元に反対側
	auto Dir = -Sps.back().GetC();
	auto ClosestDistSq = (std::numeric_limits<float>::max)();
	auto HasIntersection = false;
	Math::Vec4 Lambda;
	do {
		//!< Dir 方向のサポートポイントを求める
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, Dir.ToNormalized(), 0.0f);

		//!< Pt は既存の点、もうこれ以上拡張できない -> 衝突無し
		if (std::ranges::any_of(Sps,
			[&](const auto& i) {
				return Pt.GetC().NearlyEqual(i.GetC());
			})) {
			break;
		}

		//!< 最近接点を求めない場合は早期終了できる可能性がある
		if (!WithClosestPoint) {
			//!< 新しいサポートポイント Pt が原点を超えていない場合は原点を含まない、早期終了
			if (Dir.Dot(Pt.GetC()) < 0.0f) {
				break;
			}
		}

		//!< 新しいサポートポイントを追加
		Sps.emplace_back(Pt);

		//!< シンプレックスが原点を含むなら衝突
		if ((HasIntersection = Collision::SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda))) {
			break;
		}

		//!< 最短距離を更新、更新できなれば終了
		//!< (2 つの三角形からなる四角形の対角線上に原点が位置するような場合、2 つの三角形間で切り替えループになるのを回避)
		const auto DistSq = Dir.LengthSq();
		if (DistSq >= ClosestDistSq) {
			break;
		}
		ClosestDistSq = DistSq;

		//!< 有効なサポートポイント (対応する Lambda が非 0.0 のコンポーネント) だけを残す
		auto Index = 0;
		const auto SpsRange = std::ranges::remove_if(Sps,
			[&](const auto& i) {
				return 0.0f == Lambda[Index++];
			});
		Sps.erase(std::ranges::cbegin(SpsRange), std::ranges::cend(SpsRange));

		//!< Lambda の有効な (非 0.0) 要素を前へ集める (ケツにはゴミが残るが、Sps 個数分しかアクセスしない)
		[[maybe_unused]] const auto Dmy = std::ranges::remove(static_cast<Math::Component4&>(Lambda), 0.0f);

		//!< 3-シンプレックス (四面体) でここまで来たら原点を含む
		HasIntersection = (4 == std::size(Sps));
	} while (!HasIntersection); //!< 原点を含まずここまで来たらループ

	//!< (EPA 等で) 衝突点を求める
	if (HasIntersection && nullptr != OnIntersect) {
		OnIntersect(ShA, PosA, RotA,
			ShB, PosB, RotB,
			Sps,
			Bias, OnA, OnB);
		return true;
	}

	//!< 最近接点を求める
	if (WithClosestPoint) {
		auto Index = 0;
		OnA = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetA() * Lambda[Index++];
			});
		Index = 0;
		OnB = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetB() * Lambda[Index++];
			});
	}

	return false;
}
bool Collision::Intersection::GJK(const Physics::RigidBody* RbA,
	const Physics::RigidBody* RbB,
	OnIntersectGJK OnIntersect, const float Bias,
	Math::Vec3& OnA, Math::Vec3& OnB) {
	return GJK(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation,
		OnIntersect, Bias,
		true,
		OnA, OnB);
}

void Collision::Intersection::EPA(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	const std::vector<SupportPoint::Points>& SupportPoints, const float Bias, 
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	//!< 作業用サポートポイント
	std::vector<SupportPoint::Points> Sps;
	Sps.assign(std::cbegin(SupportPoints), std::cend(SupportPoints));

	//!< EPA の前準備
	{
		//!< EPA では四面体を必要とするので、(四面体でない場合は) 四面体へ拡張する
		SupportPoint::ToTetrahedron(ShA, PosA, RotA, ShB, PosB, RotB, Sps);

		//!< シンプレックスを拡張する (A, B の衝突点がほぼ同一の場合に法線が上手く求められない場合があるため、バイアス分拡張する)
		SupportPoint::Expand(Bias, Sps);
	}

	//!< 外側に法線が向くようにインデックスを生成する
	constexpr std::array XYZW = { 0, 1, 2, 3 };
	constexpr auto XYZWSize = static_cast<uint32_t>(std::size(XYZW));
	std::vector<Collision::TriInds> Tris;
	std::ranges::transform(XYZW, std::back_inserter(Tris),
		[&](const auto& rhs) {
			const auto i = (rhs + 0) % XYZWSize;
			const auto j = (rhs + 1) % XYZWSize;
			const auto k = (rhs + 2) % XYZWSize;
			const auto l = (rhs + 3) % XYZWSize; //!< 三角形 i, j, k を底辺とする四面体の頂点
			return Collision::Distance::IsFront(Sps[l].GetC(), Sps[i].GetC(), Sps[j].GetC(), Sps[k].GetC()) ? 
				Collision::TriInds({ j, i, k }) : Collision::TriInds({ i, j, k });
		});

	//!< 四面体の中心を求める
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
		[](const auto& Acc, const auto& i) {
			return Acc + i.GetC(); 
		}) / static_cast<float>(std::size(Sps));

	//!< 原点に最も近い面を見つける為にシンプレックスを拡張する (原点の向きに限定して凸包を形成していく感じ)
	while (true) {
		//!< 原点に最も近い三角形 ABC を取得
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]].GetC(), B = Sps[CTri[1]].GetC(), C = Sps[CTri[2]].GetC();
		//!< 三角形の法線
		const auto N = Math::Vec3::UnitNormal(A, B, C);

		//!< 法線方向のサポートポイントを取得
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, N, Bias);

		//!< これ以上拡張できない (Pt が ABC 法線の反対側) なら終了 
		//if (Distance::PointTriangle(Pt.GetC(), A, B, C) <= 0.0f) {
		//	break;
		//}
		if ((Pt.GetC() - A).Dot(N) <= 0.0f) {
			break;
		}

		//!< サポートポイントが既出の場合は、これ以上拡張できないことを意味するので終了
		if (std::ranges::any_of(Tris, 
			[&](const auto& i) {
				//!< 三角形群
				return std::ranges::any_of(i,
					[&](const auto& j) { 
						//!< 3 頂点のいずれかに等しい場合、既出
						constexpr auto Eps = 0.01f; 
						return Sps[j].GetC().NearlyEqual(Pt.GetC(), Eps);
					});
			})) {
			break;
		}

		Sps.emplace_back(Pt);

		//!< サポートポイント側を向いている三角形を削除
		{
			const auto Range = std::ranges::remove_if(Tris,
				[&](const auto& i) {
					return Distance::IsFront(Pt.GetC(), Sps[i[0]].GetC(), Sps[i[1]].GetC(), Sps[i[2]].GetC());
				});
			Tris.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
			//!< 削除できない場合は終了
			if (std::ranges::empty(Range)) {
				break; 
			}
		}

		//!< ぶら下がった辺 (削除した三角形と、残った三角形の境界となるような辺) を収集する
		{
			//!< 複数の三角形から共有されていないようなユニークな辺を収集すればよい
			std::vector<Collision::EdgeIndsWithCount> DanglingEdges;
			Convex::CollectUniqueEdges(Tris, DanglingEdges);
			//!< そのような辺が無ければ終了
			if (std::empty(DanglingEdges)) {
				break;
			}

			//!< サポートポイント A と境界辺 BC からなる三角形群を追加
			const auto A = static_cast<uint32_t>(std::size(Sps)) - 1;
			std::ranges::transform(DanglingEdges, std::back_inserter(Tris),
				[&](const auto& i) {
					const auto B = i.first[0], C = i.first[1];
					//!< 法線が外側を向くようにインデックスを生成する
					if (Distance::IsFront(Center, Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC())) {
						return Collision::TriInds({ A, C, B });
					}
					else {
						return Collision::TriInds({ A, B, C });
					}
				});
		}
	}

	{
		//!< 原点に最も近い三角形 ABC を取得
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]], B = Sps[CTri[1]], C = Sps[CTri[2]];

		//!< ABC 上での原点の重心座標を取得
		const auto Lambda = Barycentric(A.GetC(), B.GetC(), C.GetC());
		if (Lambda != std::nullopt) {
			OnA = A.GetA() * Lambda.value()[0] + B.GetA() * Lambda.value()[1] + C.GetA() * Lambda.value()[2];
			OnB = A.GetB() * Lambda.value()[0] + B.GetB() * Lambda.value()[1] + C.GetB() * Lambda.value()[2];
#ifdef _DEBUG
			//LOG(std::data(std::format("PenetrationDistSq = {}\n", (OnB - OnA).LengthSq())));
#endif
		}
		else {
			//!< #TODO 原点が三角形の外側にある
			LOG(std::data(std::format("EPA: Barycentric failed\n")));
		}
	}
}

//!< #TODO 要検証			
//!< ミンコフスキー差の凸包を生成する代わりに原点を含むようなシンプレックス (単体) を生成する事で代用する
//!< A, B のミンコフスキー差 C が原点を含めば衝突となる
//!< A, B のサポートポイントの差が C のサポートポイントとなる
//!<	最初のサポートポイント 1 を見つける
//!<	原点方向の次のサポートポイント 2 を見つける
//!<	1, 2 の線分から原点方向の次のサポートポイント 3 を見つける
//!<	1, 2, 3 がなす三角形が原点を含めば衝突、終了
//!<	原点を向く法線方向の次のサポートポイント 4 を見つける
//!<	1, 2, 3, 4 がなす四面体が原点を含めば衝突、終了
//!<	一番近い三角形 (例えば 1, 2, 4) から、原点を向く法線方向の次のサポートポイント 5 を見つける
//!<	四面体 (1, 2, 4, 5) が原点を含むか、サポートポイントが無くなるまで続ける
bool Collision::Intersection::GJK_EPA(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	const float Bias, 
	bool WithClosestPoint,
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	return GJK_EPA(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation,
		Bias,
		WithClosestPoint,
		OnA, OnB);
}
bool Collision::Intersection::GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB) 
{
	return GJK(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation);
}

void Collision::Closest::GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, 
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	Intersection::GJK(ShA, PosA, RotA,
		ShB, PosB, RotB,
		Intersection::OnIntersectDummy, 0.0f,
		true,
		OnA, OnB);
}
void Collision::Closest::GJK(const Physics::RigidBody* RbA, 
	const Physics::RigidBody* RbB, 
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	GJK(RbA->Shape, RbA->Position, RbA->Rotation, 
		RbB->Shape, RbB->Position, RbB->Rotation, 
		OnA, OnB);
}

#ifdef _DEBUG
void Collision::SignedVolumeTest()
{
	constexpr auto Eps = 0.001f;
	{
		const std::vector OrgPts = {
			Math::Vec3(0.0f, 0.0f, 0.0f),
			Math::Vec3(1.0f, 0.0f, 0.0f),
			Math::Vec3(0.0f, 1.0f, 0.0f),
			Math::Vec3(0.0f, 0.0f, 1.0f),
		};
		{
			//!< 検証済
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(1.0f, 1.0f, 1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
			const auto CorrectV = Math::Vec3::One();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< 検証済
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(-1.0f, -1.0f, -1.0f) * 0.25f));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.25f, 0.25f, 0.25f, 0.25f);
			const auto CorrectV = Math::Vec3::Zero();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< 検証済
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(-1.0f, -1.0f, -1.0f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.0f, 0.333f, 0.333f, 0.333f);
			const auto CorrectV = Math::Vec3(-0.667f, -0.667f, -0.667f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
		{
			//!< 検証済
			std::vector<Math::Vec3> Pts;
			Pts.resize(std::size(OrgPts));
			std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Math::Vec3(1.0f, 1.0f, -0.5f)));
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			const auto CorrectLambda = Math::Vec4(0.5f, 0.0f, 0.0f, 0.5f);
			const auto CorrectV = Math::Vec3(1.0f, 1.0f, 0.0f);
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
	}
	{
		//!< 検証済
		const std::array Pts = {
			Math::Vec3(51.1996613f, 26.1989613f, 1.91339576f),
			Math::Vec3(-51.0567360f, -26.0565681f, -0.436143428f),
			Math::Vec3(50.8978920f, -24.1035538f, -1.04042661f),
			Math::Vec3(-49.1021080f, 25.8964462f, -1.04042661f)
		};
		const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
		const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

		//!< 答え合わせ
		const auto CorrectLambda = Math::Vec4(0.290f, 0.302f, 0.206f, 0.202f);
		const auto CorrectV = Math::Vec3::Zero();
		if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
			__debugbreak();
		}
		if (!V.NearlyEqual(CorrectV, Eps)) {
			__debugbreak();
		}
	}
}
#endif
