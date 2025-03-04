#include "GJK.h"
#include "RigidBody.h"
#include "Shape.h"
#include "Collision.h"
#include "Convex.h"

#include "Log.h"

//!< 1-シンプレックス (線分) ... 原点のシンプレックス上での重心座標 (シンプレックス上にあるか) を返す
Math::Vec2 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B)
{
	const auto AB = B - A;
	//!< 原点を AB 軸に射影し P とする
	const auto P = A + AB * AB.Dot(-A) / AB.LengthSq(); //!< 平方根を省けるので正規化しない方が良い

	//!< X, Y, Z 軸の内、絶対値が最大のものを見つける
	auto Index = 0;
	auto MaxVal = 0.0f;
	for (auto i = 0; i < 3; ++i) {
		if (std::abs(MaxVal) < std::abs(AB[i])) {
			MaxVal = AB[i];
			Index = i;
		}
	}

	//!< 「P と線分」を選択した軸へ射影
	const auto PrjA = A[Index];
	const auto PrjB = B[Index];
	const auto PrjP = P[Index];

	//!< P が [A, B] の内部にある場合
	if ((PrjP > PrjA && PrjP < PrjB) || (PrjP > PrjB && PrjP < PrjA)) {
		return Math::Vec2(PrjB - PrjP, PrjP - PrjA) / MaxVal;
	}
	//!< P が A 側の外側
	if ((PrjA <= PrjB && PrjP <= PrjA) || (PrjA >= PrjB && PrjP >= PrjA)) {
		return Math::Vec2::AxisX();
	}
	//!< P が B 側の外側
	else {
		return Math::Vec2::AxisY();
	}
}

//!< 2-シンプレックス (三角形)
Math::Vec3 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C)
{
	Math::Vec3 BC;
	if (Barycentric(A, B, C, BC)) {
		return BC;
	}

	//!< 3 辺に射影して一番近いものを見つける (1-SignedVolume に帰着)
	const std::array EdgePts = { A, B, C };
	Math::Vec3 Lambda;
	auto MinVal = (std::numeric_limits<float>::max)();
	for (auto i = 0; i < 3; ++i) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		const auto LambdaEdge = SignedVolume(EdgePts[j], EdgePts[k]);
		const auto LenSq = (EdgePts[j] * LambdaEdge[0] + EdgePts[k] * LambdaEdge[1]).LengthSq();
		if (LenSq < MinVal) {
			Lambda[i] = 0.0f;
			Lambda[j] = LambdaEdge[0];
			Lambda[k] = LambdaEdge[1];
			MinVal = LenSq;
		}
	}
	return Lambda;
}

//!< 3-シンプレックス (四面体)
Math::Vec4 Collision::SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D)
{
	//const auto Cofactor = Vec4(-Mat3(B, C, D).Determinant(), Mat3(A, C, D).Determinant(), -Mat3(A, B, D).Determinant(), Mat3(A, B, C).Determinant());
	const auto M = Math::Mat4(Math::Vec4(A.X(), B.X(), C.X(), D.X()), 
		Math::Vec4(A.Y(), B.Y(), C.Y(), D.Y()), 
		Math::Vec4(A.Z(), B.Z(), C.Z(), D.Z()), 
		Math::Vec4::One());
	const auto Cofactor = Math::Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
	const auto Det = Cofactor.X() + Cofactor.Y() + Cofactor.Z() + Cofactor.W();

	//!< 四面体内部にあれば、重心座標を返す
	if (Sign(Det) == Sign(Cofactor.X()) && Sign(Det) == Sign(Cofactor.Y()) && Sign(Det) == Sign(Cofactor.Z()) && Sign(Det) == Sign(Cofactor.W())) {
		return Cofactor / Det;
	}

	//!< 3 面に射影して一番近いものを見つける (2-SignedVolume に帰着)
	const std::array FacePts = { A, B, C, D };
	Math::Vec4 Lambda;
	auto MinVal = (std::numeric_limits<float>::max)();
	for (auto i = 0; i < 4; ++i) {
		const auto j = (i + 1) % 4;
		const auto k = (i + 2) % 4;

		const auto LambdaFace = SignedVolume(FacePts[i], FacePts[j], FacePts[k]);
		const auto LenSq = (FacePts[i] * LambdaFace[0] + FacePts[j] * LambdaFace[1] + FacePts[k] * LambdaFace[2]).LengthSq();
		if (LenSq < MinVal) {
			Lambda.ToZero();
			Lambda[i] = LambdaFace[0];
			Lambda[j] = LambdaFace[1];
			Lambda[k] = LambdaFace[2];
			MinVal = LenSq;
		}
	}
	return Lambda;
}

bool Collision::Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, Math::Vec3& BC)
{
	const auto N = Math::Vec3::Normal(A, B, C);
	if (N.NearlyEqual(Math::Vec3::Zero())) { 
		return false; 
	}
	const auto P = N * A.Dot(N) / N.LengthSq();

	//!< XY, YZ, ZX 平面の内、射影面積が最大のものを見つける
	auto Index = 0;
	auto MaxVal = 0.0f;
	for (auto i = 0; i < 3; ++i) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		const auto a = Math::Vec2(A[j], A[k]);
		const auto b = Math::Vec2(B[j], B[k]);
		const auto c = Math::Vec2(C[j], C[k]);
		const auto AB = b - a;
		const auto AC = c - a;
		const auto Area = Math::Mat2(AB, AC).Determinant();

		if (std::abs(Area) > std::abs(MaxVal)) {
			MaxVal = Area;
			Index = i;
		}
	}

	//!< 「P と三角形」を選択した平面に射影 (X が選択された場合 Index1, Index2 はそれぞれ Y, Z といった具合になる)
	const auto X = (Index + 1) % 3;
	const auto Y = (Index + 2) % 3;
	const std::array PrjABC = { Math::Vec2(A[X], A[Y]), Math::Vec2(B[X], B[Y]), Math::Vec2(C[X], C[Y]) };
	const auto PrjP = Math::Vec2(P[X], P[Y]);

	//!< 射影点と辺からなるサブ三角形の面積
	Math::Vec3 Areas;
	for (auto i = 0; i < 3; i++) {
		const auto j = (i + 1) % 3;
		const auto k = (i + 2) % 3;

		Areas[i] = Math::Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
	}

	//!< P が [A, B, C] の内部にある場合 (サブ三角形の面積の符号から分かる)
	if (Sign(MaxVal) == Sign(Areas.X()) && Sign(MaxVal) == Sign(Areas.Y()) && Sign(MaxVal) == Sign(Areas.Z())) {
		BC = Areas / MaxVal;
		return true;
	}

	return false;
}
bool Collision::Barycentric(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, Math::Vec3& BC) 
{
	return Barycentric(A - Pt, B - Pt, C - Pt, BC);
}

//!< A, B のサポートポイント、及びその差 C を求める
Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, const Math::Vec3& UDir, const float Bias)
{
	return { ShA->GetSupportPoint(PosA, RotA, UDir, Bias), ShB->GetSupportPoint(PosB, RotB, -UDir, Bias) };
}
//Collision::SupportPoint::Points Collision::SupportPoint::Get(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& UDir, const float Bias)
//{
//	return Get(RbA->Shape, RbA->Position, RbA->Rotation, RbB->Shape, RbB->Position, RbB->Rotation, UDir, Bias);
//}
void Collision::SupportPoint::ToTetrahedron(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, std::vector<SupportPoint::Points>& Sps)
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
void Collision::SupportPoint::ToTetrahedron(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, std::vector<Collision::SupportPoint::Points>& Sps)
{
	ToTetrahedron(RbA->Shape, RbA->Position, RbA->Rotation, RbB->Shape, RbB->Position, RbB->Rotation, Sps);
}
void Collision::SupportPoint::Expand(const float Bias, std::vector<Collision::SupportPoint::Points>& Sps)
{
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(), [](const auto& Acc, const auto& i) { return Acc + i.GetC(); }) / static_cast<float>(std::size(Sps));
	std::ranges::transform(Sps, std::begin(Sps), [&](const auto& i) {
		const auto Dir = (i.GetC() - Center).Normalize() * Bias;
		return Collision::SupportPoint::Points(i.GetA() + Dir, i.GetB() - Dir);
		});
}

void Collision::Intersection::EPA(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, const std::vector<SupportPoint::Points>& SupportPoints, const float Bias, Math::Vec3& OnA, Math::Vec3& OnB)
{
	//!< 作業用サポートポイント
	std::vector<SupportPoint::Points> Sps;
	Sps.assign(std::cbegin(SupportPoints), std::cend(SupportPoints));

	//!< EPA の前準備
	{
		//!< EPA は四面体を必要とするので、四面体へ拡張する
		SupportPoint::ToTetrahedron(ShA, PosA, RotA, ShB, PosB, RotB, Sps);
		//!< シンプレックスをバイアスの分だけ拡張する
		SupportPoint::Expand(Bias, Sps);
	}

	//!< 外側に法線が向くように4面分の三角形を形成
	std::vector<Collision::TriInds> Tris;
	for (uint32_t i = 0; i < 4; ++i) {
		const auto j = (i + 1) % 4;
		const auto k = (i + 2) % 4;

		const auto l = (i + 3) % 4;

		if (Collision::Distance::IsFront(Sps[l].GetC(), Sps[i].GetC(), Sps[j].GetC(), Sps[k].GetC())) {
			Tris.emplace_back(Collision::TriInds({ j, i, k }));
		}
		else {
			Tris.emplace_back(Collision::TriInds({ i, j, k }));
		}
	}

	//!< 四面体の中心
	const auto Center = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(), [](const auto& Acc, const auto& i) { return Acc + i.GetC(); }) / static_cast<float>(std::size(Sps));

	//!< 原点に最も近い面を見つける為にシンプレックスを拡張する
	//!< 原点の向きに限定して凸包を形成していく感じ
	while (true) {
		//!< 原点に最も近い三角形を取得
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto& A = Sps[CTri[0]].GetC(), B = Sps[CTri[1]].GetC(), C = Sps[CTri[2]].GetC();

		//!< 三角形の法線
		const auto N = Math::Vec3::UnitNormal(A, B, C);

		//!< ミンコフスキー差の法線方向のサポートポイントを取得
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, N, Bias);

		//!< サポートポイントが既出の場合は、これ以上拡張できない
		if (std::ranges::any_of(Tris, [&](const auto& i) {
			return std::ranges::any_of(i, [&](const auto& j) { constexpr auto Eps = 0.01f; return Sps[j].GetC().NearlyEqual(Pt.GetC(), Eps); });
			})) {
			break;
		}

		//!< これ以上拡張できない
		if (Distance::PointTriangle(Pt.GetC(), A, B, C) <= 0.0f) {
			break;
		}

		Sps.emplace_back(Pt);

		//!< サポートポイント側を向いている三角形を削除
		{
			const auto Range = std::ranges::remove_if(Tris, [&](const auto& i) {
				return Distance::IsFront(Pt.GetC(), Sps[i[0]].GetC(), Sps[i[1]].GetC(), Sps[i[2]].GetC());
				});
			Tris.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
			//!< 削除できない場合は終了
			if (std::ranges::empty(Range)) { break; }
		}

		{
			//!< (削除した三角形と残った三角形の) 境界となるような (ユニークな) 辺を収集する、
			std::vector<Collision::EdgeIndsCount> DanglingEdges;
			Convex::CollectUniqueEdges(Tris, DanglingEdges);
			//!< そのような辺が無ければ終了
			if (std::empty(DanglingEdges)) { break; }

			//!< サポートポイントと境界辺からなる三角形群を追加
			const auto A = static_cast<uint32_t>(std::size(Sps)) - 1;
			std::ranges::transform(DanglingEdges, std::back_inserter(Tris), [&](const auto& i) {
				const auto B = i.first[0], C = i.first[1];
				//!< 法線が外側を向くように
				if (!Distance::IsFront(Center, Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC())) {
					return Collision::TriInds({ A, B, C });
				}
				else {
					return Collision::TriInds({ A, C, B });
				}
				});
		}
	}

	{
		//!< 原点に最も近い三角形を取得
		const auto& CTri = *SupportPoint::Distance::Closest(Math::Vec3::Zero(), Sps, Tris);
		const auto A = CTri[0], B = CTri[1], C = CTri[2];

		//!< それ上での、原点の重心座標を取得
		Math::Vec3 Lambda;
		Barycentric(Math::Vec3::Zero(), Sps[A].GetC(), Sps[B].GetC(), Sps[C].GetC(), Lambda);
		OnA = Sps[A].GetA() * Lambda[0] + Sps[B].GetA() * Lambda[1] + Sps[C].GetA() * Lambda[2];
		OnB = Sps[A].GetB() * Lambda[0] + Sps[B].GetB() * Lambda[1] + Sps[C].GetB() * Lambda[2];
#ifdef _DEBUG
		//LOG(std::data(std::format("PenetrationDistSq = {}\n", (OnB - OnA).LengthSq())));
#endif
	}
}

bool Collision::Intersection::GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
	const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB,
	OnIntersectGJK OnIntersect, const float Bias,
	const bool WidthClosestPoint,
	Math::Vec3& OnA, Math::Vec3& OnB) 
{
	std::vector<Collision::SupportPoint::Points> Sps;
	Sps.reserve(4); //!< 4 枠

	//!< (1, 1, 1) 方向のサポートポイントを求める
	Sps.emplace_back(Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, Math::Vec3::One().Normalize(), 0.0f));

	auto Dir = -Sps.back().GetC();
	auto ClosestDistSq = (std::numeric_limits<float>::max)();
	auto HasIntersection = false;
	Math::Vec4 Lambda;
	do {
		const auto Pt = Collision::SupportPoint::Get(ShA, PosA, RotA, ShB, PosB, RotB, Dir.ToNormalized(), 0.0f);

		//!< Pt は既存の点、もうこれ以上拡張できない -> 衝突無し
		if (std::ranges::any_of(Sps, [&](const auto& i) { return Pt.GetC().NearlyEqual(i.GetC()); })) { 
			break; 
		}

		//!< 最近接点を求める場合は早期終了させない
		if (WidthClosestPoint) {
			//!< 新しいサポートポイント Pt が原点を超えていない場合は原点を含まない
			if (Dir.Dot(Pt.GetC() - Math::Vec3::Zero()) < 0.0f) {
				break;
			}
		}

		//!< 新しいサポートポイントを追加した上で
		Sps.emplace_back(Pt);

		//!< シンプレックスが原点を含むなら衝突
		if ((HasIntersection = Collision::SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda))) {
			break;
		}

		//!< 最短距離を更新できなれば終了
		const auto DistSq = Dir.LengthSq();
		if (DistSq >= ClosestDistSq) {
			break;
		}
		ClosestDistSq = DistSq;

		//!< 有効な Sps だけを残す
		const auto SpsRange = std::ranges::remove_if(Sps, 
			[&](const auto& i) { 
				return 0.0f == Lambda[static_cast<int>(IndexOf(Sps, i))];
			});
		Sps.erase(std::ranges::cbegin(SpsRange), std::ranges::cend(SpsRange));

		[[maybe_unused]] const auto Dmy = std::ranges::remove(static_cast<Math::Component4&>(Lambda), 0.0f);

		//!< 3-シンプレックス (四面体) でここまで来たら原点を含む
		HasIntersection = (4 == size(Sps));
	} while (!HasIntersection); //!< 原点を含まずここまで来たらループ

	//!< 衝突点を求める
	if (HasIntersection) {
		OnIntersect(ShA, PosA, RotA, 
			ShB, PosB, RotB, 
			Sps, 
			Bias, OnA, OnB);
		return true;
	}

	//!< 最近接点を求める
	if (WidthClosestPoint) {
		OnA = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(), 
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetA() * Lambda[static_cast<int>(IndexOf(Sps, i))];
			});
		OnB = std::accumulate(std::cbegin(Sps), std::cend(Sps), Math::Vec3::Zero(),
			[&](const auto& Acc, const auto& i) {
				return Acc + i.GetB() * Lambda[static_cast<int>(IndexOf(Sps, i))];
			});
	}

	return false;
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
	bool WidthClosestPoint,
	Math::Vec3& OnA, Math::Vec3& OnB)
{
	return GJK_EPA(RbA->Shape, RbA->Position, RbA->Rotation,
		RbB->Shape, RbB->Position, RbB->Rotation,
		Bias,
		WidthClosestPoint,
		OnA, OnB);
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
