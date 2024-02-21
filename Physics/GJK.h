#pragma once

#include "Math.h"
using namespace Math;

#include <algorithm>
#include <functional>

#include "Shape.h"
#include "RigidBody.h"
using namespace Physics;

namespace Collision
{
	//!< SignedVolue : 射影が最大となる軸や平面を見つけ、それに対し原点を射影して内部にあれば重心を返す

	//!< 検証済
	[[nodiscard]] static Vec2 SignedVolume(const Vec3& A, const Vec3& B)
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
			return Vec2(PrjB - PrjP, PrjP - PrjA) / MaxVal;
		}
		//!< P が A 側の外側
		if ((PrjA <= PrjB && PrjP <= PrjA) || (PrjA >= PrjB && PrjP >= PrjA)) {
			return Vec2::AxisX();
		}
		//!< P が B 側の外側
		else {
			return Vec2::AxisY();
		}
	}
	//!< 検証済
	[[nodiscard]] static Vec3 SignedVolume(const Vec3& A, const Vec3& B, const Vec3& C)
	{
		const auto N = (B - A).Cross(C - A);
		const auto P = N * A.Dot(N) / N.LengthSq(); //!< 平方根を省けるので正規化しない方が良い

		//!<  XY, YZ, ZX 平面の内、射影面積が最大のものを見つける
		auto Index = 0;
		auto MaxVal = 0.0f;
		for (auto i = 0; i < 3; ++i) {
			const auto j = (i + 1) % 3;
			const auto k = (i + 2) % 3;

			const auto a = Vec2(A[j], A[k]);
			const auto b = Vec2(B[j], B[k]);
			const auto c = Vec2(C[j], C[k]);
			const auto AB = b - a;
			const auto AC = c - a;
			const auto Area = Mat2(AB, AC).Determinant();
			//const auto Area = AB.X() * AC.Y() - AB.Y() * AC.X();

			if (std::abs(Area) > std::abs(MaxVal)) {
				MaxVal = Area;
				Index = i;
			}
		}

		//!< 「P と三角形」を選択した平面に射影 (X が選択された場合 Index1, Index2 はそれぞれ Y, Z といった具合になる)
		const auto X = (Index + 1) % 3;
		const auto Y = (Index + 2) % 3;
		const std::array PrjABC = { Vec2(A[X], A[Y]), Vec2(B[X], B[Y]), Vec2(C[X], C[Y]) };
		const auto PrjP = Vec2(P[X], P[Y]);

		//!< 射影点と辺からなるサブ三角形の面積
		Vec3 Areas;
		for (auto i = 0; i < 3; ++i) {
			const auto j = (i + 1) % 3;
			const auto k = (i + 2) % 3;

			Areas[i] = Mat2(PrjABC[j] - PrjP, PrjABC[k] - PrjP).Determinant();
		}
		//!< P が [A, B, C] の内部にある場合 (サブ三角形の面積の符号から分かる)
		if (Sign(MaxVal) == Sign(Areas.X()) && Sign(MaxVal) == Sign(Areas.Y()) && Sign(MaxVal) == Sign(Areas.Z())) {
			return Areas / MaxVal;
		}

		//!< 3 辺に射影して一番近いものを見つける (1-SignedVolume に帰着)
		const std::array EdgePts = { A, B, C };
		Vec3 Lambda;
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
	//!< 検証済
	[[nodiscard]] static Vec4 SignedVolume(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
	{
		//const auto Cofactor = Vec4(-Mat3(B, C, D).Determinant(), Mat3(A, C, D).Determinant(), -Mat3(A, B, D).Determinant(), Mat3(A, B, C).Determinant());
		const auto M = Mat4(Vec4(A.X(), B.X(), C.X(), D.X()), Vec4(A.Y(), B.Y(), C.Y(), D.Y()), Vec4(A.Z(), B.Z(), C.Z(), D.Z()), Vec4::One());
		const auto Cofactor = Vec4(M.Cofactor(3, 0), M.Cofactor(3, 1), M.Cofactor(3, 2), M.Cofactor(3, 3));
		const auto Det = Cofactor.X() + Cofactor.Y() + Cofactor.Z() + Cofactor.W();

		//!< 四面体内部にあれば、重心座標を返す
		if (Sign(Det) == Sign(Cofactor.X()) && Sign(Det) == Sign(Cofactor.Y()) && Sign(Det) == Sign(Cofactor.Z()) && Sign(Det) == Sign(Cofactor.W())) {
			return Cofactor / Det;
		}

		//!< 3 面に射影して一番近いものを見つける (2-SignedVolume に帰着)
		const std::array FacePts = { A, B, C, D };
		Vec4 Lambda;
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

	namespace SupportPoint {
		//!< サポートポイント : 特定の方向に最も遠い点

		//!< A, B のサポートポイントの差が、(A, B のミンコフスキー差) C のサポートポイントとなる
		class Points
		{
		public:
			Points(const Vec3& A, const Vec3& B) : SPs({ A - B, A, B }) { }

			const Vec3 GetA() const { return SPs[1]; }
			const Vec3 GetB() const { return SPs[2]; }
			const Vec3 GetC() const { return SPs[0]; }

			bool operator == (const Points& i) const { return std::ranges::equal(SPs, i.SPs); }
		private:
			std::array<Vec3, 3> SPs;
		};
		static [[nodiscard]] Points GetPoints(const RigidBody* RbA, const RigidBody* RbB, const Vec3& NDir, const float Bias) {
			return Points(RbA->Shape->GetSupportPoint(RbA->Position, RbA->Rotation, NDir, Bias), RbB->Shape->GetSupportPoint(RbB->Position, RbB->Rotation, -NDir, Bias));
		}
		static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<Points>& Sps, Vec3& Dir, Vec4& OutLambda)
		{
			//constexpr auto Eps2 = (std::numeric_limits<float>::epsilon)() * (std::numeric_limits<float>::epsilon)();
			constexpr auto Eps2 = 0.0001f * 0.00001f;

			switch (size(Sps)) {
			case 2:
				OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC());
				Dir = -(Sps[0].GetC() * OutLambda[0] + Sps[1].GetC() * OutLambda[1]);
				break;
			case 3:
				OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC(), Sps[2].GetC());
				Dir = -(Sps[0].GetC() * OutLambda[0] + Sps[1].GetC() * OutLambda[1] + Sps[2].GetC() * OutLambda[2]);
				break;
			case 4:
				OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC(), Sps[2].GetC(), Sps[3].GetC());
				Dir = -(Sps[0].GetC() * OutLambda[0] + Sps[1].GetC() * OutLambda[1] + Sps[2].GetC() * OutLambda[2] + Sps[3].GetC() * OutLambda[3]);
				break;
			default:
				__debugbreak();
				break;
			}

			//!< 原点を含む -> 衝突
			return Dir.LengthSq() < Eps2;
		}

		//!< サポートポイントが四面体をなしていない場合、四面体を形成する
		void ToTetrahedron(const RigidBody* RbA, const RigidBody* RbB, std::vector<SupportPoint::Points>& Sps) {
			if (1 == std::size(Sps)) {
				Sps.emplace_back(GetPoints(RbA, RbB, -Sps[0].GetC().Normalize(), 0.0f));
			}
			if (2 == std::size(Sps)) {
				const auto AB = Sps[1].GetC() - Sps[0].GetC();
				Vec3 U, V;
				AB.GetOrtho(U, V);
				Sps.emplace_back(GetPoints(RbA, RbB, U, 0.0f));
			}
			if (3 == std::size(Sps)) {
				const auto AB = Sps[1].GetC() - Sps[0].GetC();
				const auto AC = Sps[2].GetC() - Sps[0].GetC();
				Sps.emplace_back(GetPoints(RbA, RbB, AB.Cross(AC).Normalize(), 0.0f));
			}
		}
		//!< バイアスの分だけ拡張する
		void Expand(const float Bias, std::vector<SupportPoint::Points>& Sps) {
			const auto Center = (Sps[0].GetC() + Sps[1].GetC() + Sps[2].GetC() + Sps[3].GetC()) * 0.25f;
			std::ranges::transform(Sps, std::begin(Sps), [&](const auto& rhs) {
				const auto Dir = (rhs.GetC() - Center).Normalize() * Bias;
				return SupportPoint::Points(rhs.GetA() + Dir, rhs.GetB() - Dir);
			});
		}
	}

	namespace Intersection {
		//!< #TODO 要検証			
		//!< ミンコフスキー差の凸包を生成する代わりに原点を含むような単体を生成する事で代用する
		//!< A, B のミンコフスキー差 C が原点を含めば衝突となる
		//!< A, B のサポートポイントの差が C のサポートポイントとなる
		//!<	最初のサポートポイント 1 を見つける
		//!<	原点方向の次のサポートポイント 2 を見つける
		//!<	1, 2 の線分から原点方向の次のサポートポイント 3 を見つける
		//!<	1, 2, 3 がなす三角形が原点を含めば衝突、終了
		//!<	原点を向く法線方向の次のサポートポイント 4 を見つける
		//!<	1, 2, 3, 4 がなす四面体が原点を含めば衝突、終了
		//!<	一番近い三角形 (例えば 1, 2, 4) から、原点を向く法線方向の次のサポートポイント 5 を見つける
		//!<	四面体が原点を含むか、サポートポイントが無くなるまで続ける
		[[nodiscard]] static bool GJK(const RigidBody* RbA, const RigidBody* RbB, std::function<void(const RigidBody*, const RigidBody*, const std::vector<SupportPoint::Points>&, const float, Vec3&, Vec3&)> OnIntersect, const float Bias, Vec3& OnA, Vec3& OnB)
		{
			std::vector<SupportPoint::Points> Sps;
			Sps.reserve(4); //!< 4 枠

			//!< (1, 1, 1) 方向のサポートポイントを求める
			Sps.emplace_back(SupportPoint::GetPoints(RbA, RbB, Vec3::One().Normalize(), 0.0f));

			auto ClosestDist = (std::numeric_limits<float>::max)();
			auto Dir = -Sps.back().GetC();
			auto ContainOrigin = false;
			do {
				const auto Pt = SupportPoint::GetPoints(RbA, RbB, Dir.ToNormalized(), 0.0f);
				Dir *= -1.0f;

				//!< 既存の点ということはもうこれ以上拡張できない -> 衝突無し
				if (std::end(Sps) != std::ranges::find_if(Sps, [&](const auto& rhs) { return Pt.GetC().NearlyEqual(rhs.GetC()); })) {
					break;
				}

				Sps.emplace_back(Pt);

				//!< これはいらない気がする…
				//!< 新しい点が原点を超えていない場合、原点が内部に含まれない -> 衝突無し
				//if (Dir.Dot(Pt.GetC()) < 0.0f) {
				//	break;
				//}

				//!< シンプレックスが原点を含む -> 衝突
				//!< (Dir を更新、Lambda を返す)
				Vec4 Lambda;
				if ((ContainOrigin = SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda))) {
					break;
				}

				//!< 最短距離を更新、更新できなれば終了
				const auto Dist = Dir.LengthSq();
				if (Dist < ClosestDist) {
					ClosestDist = Dist;
				}
				else {
					break;
				}

				//!< 有効 (Lambda[n] が 非 0) な Sps だけを残す
				const auto [Beg, End] = std::ranges::remove_if(Sps, [&](const auto& rhs) {
					return 0.0f == Lambda[static_cast<int>(IndexOf(Sps, rhs))];
				});
				Sps.erase(Beg, End);

				//!< シンプレックスが四面体でここまで来たら原点を含む
				ContainOrigin = (4 == size(Sps));
			} while (!ContainOrigin); //!< 原点を含まずここまで来たらループ

			//!< 原点を含まずループを抜けた場合 -> 衝突無し
			if (!ContainOrigin) {
				return false;
			}

			//!< ---- 衝突確定 ----
		
			//!< EPA は四面体を必要とするので、四面体を形成する
			SupportPoint::ToTetrahedron(RbA, RbB, Sps);
	
			//!< シンプレックスをバイアスの分だけ拡張する
			SupportPoint::Expand(Bias, Sps);

			//!< 衝突確定時に呼び出す関数 (EPA 等)
			OnIntersect(RbA, RbB, Sps, Bias, OnA, OnB);

			return true;
		}

		//!< EPA (Expanding Polytope Algorithm)
		[[nodiscard]] float EPA(const RigidBody* RbA, const RigidBody* RbB, const std::vector<SupportPoint::Points>& _Sps, const float Bias, Vec3& OnA, Vec3& OnB) {
			//!< 作業用サポートポイント
			std::vector<SupportPoint::Points> Sps;
			Sps.assign(std::begin(_Sps), std::end(_Sps));

			//!< 頂点から三角形のインデックスリストを生成
			std::vector<TriInds> Tris;
			for (uint32_t i = 0; i < 4; ++i) {
				const auto j = (i + 1) % 4;
				const auto k = (i + 2) % 4;

				const auto l = (i + 3) % 4;

				if (Distance::PointTriangle(Sps[l].GetC(), Sps[i].GetC(), Sps[j].GetC(), Sps[k].GetC()) > 0.0f) {
					Tris.emplace_back(TriInds({ j, i, k }));
				}
				else {
					Tris.emplace_back(TriInds({ i, j, k }));
				}
			}

			const Vec3 Center = (Sps[0].GetC() + Sps[1].GetC() + Sps[2].GetC() + Sps[3].GetC()) * 0.25f;

			while (true) {
				//!< 原点に最も近い三角形を取得
				const auto& CTri = *std::ranges::min_element(Tris, [&](const auto& lhs, const auto& rhs) {
					return Distance::PointTriangle(Vec3::Zero(), Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC()) < Distance::PointTriangle(Vec3::Zero(), Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC());
				});
				//!< 三角形の法線
				const auto Nrm = Vec3::Normal(Sps[CTri[0]].GetC(), Sps[CTri[1]].GetC(), Sps[CTri[2]].GetC());
			
				//!< 法線方向のサポートポイントを取得
				const auto Pt = SupportPoint::GetPoints(RbA, RbB, Nrm, Bias);

				//!< サポートポイントが既出の場合は、これ以上拡張できない
				if (std::ranges::any_of(Tris, [&](const auto rhs) {
					return Sps[rhs[0]].GetC().NearlyEqual(Pt.GetC()) || Sps[rhs[1]].GetC().NearlyEqual(Pt.GetC()) || Sps[rhs[2]].GetC().NearlyEqual(Pt.GetC());
				})) {
					break;
				}

				//!< サポートポイントと三角形の距離が 0 以下の場合は、これ以上拡張できない
				if (Distance::PointTriangle(Pt.GetC(), Sps[CTri[0]].GetC(), Sps[CTri[1]].GetC(), Sps[CTri[2]].GetC()) <= 0.0f) {
					break;
				}
				
				Sps.emplace_back(Pt);

				//!< サポートポイント側を向いている三角形を削除、削除できない場合は終了
				const auto Range = std::ranges::remove_if(Tris, [&](const auto& i) {
					return Distance::PointTriangle(Pt.GetC(), Sps[i[0]].GetC(), Sps[i[1]].GetC(), Sps[i[2]].GetC()) > 0.0f;
				});
				Tris.erase(std::begin(Range), std::end(Range));
				if (0 == std::ranges::distance(Range)) {
					break;
				}

				//!< 宙ぶらりんの辺を探す、無ければ終了
				
				//!< #TODO
			}

			//!< #TODO
			return 0.0f;
		}

		[[nodiscard]] static bool GJK(const RigidBody* RbA, const RigidBody* RbB) {
			Vec3 OnA, OnB;
			return GJK(RbA, RbB, [](const RigidBody*, const RigidBody*, const std::vector<SupportPoint::Points>&, const float, Vec3&, Vec3&) {}, 0.001f, OnA, OnB);
		}
		[[nodiscard]] static bool GJK_EPA(const RigidBody* RbA, const RigidBody* RbB, const float Bias, Vec3& OnA, Vec3& OnB) {
			return GJK(RbA, RbB, EPA, Bias, OnA, OnB);
		}
	}
	namespace Closest {
		static void GJK(const RigidBody* RbA, const RigidBody* RbB, Vec3& OnA, Vec3& OnB) {
			std::vector<SupportPoint::Points> Sps;
			Sps.reserve(4);

			Sps.emplace_back(SupportPoint::GetPoints(RbA, RbB, Vec3::One().Normalize(), 0.0f));

			auto ClosestDist = (std::numeric_limits<float>::max)();
			auto Dir = -Sps.back().GetC();
			Vec4 Lambda;
			do {
				const auto Pt = SupportPoint::GetPoints(RbA, RbB, Dir.ToNormalized(), 0.0f);
				Dir *= -1.0f;

				if (std::end(Sps) != std::ranges::find_if(Sps, [&](const auto& rhs) { return Pt.GetC().NearlyEqual(rhs.GetC()); })) {
					break;
				}

				Sps.emplace_back(Pt);

				SupportPoint::SimplexSignedVolumes(Sps, Dir, Lambda);

				const auto [Beg, End] = std::ranges::remove_if(Sps, [&](const auto& rhs) {
					return 0.0f == Lambda[static_cast<int>(IndexOf(Sps, rhs))];
				});
				Sps.erase(Beg, End);

				const auto Dist = Dir.LengthSq();
				if (Dist < ClosestDist) {
					ClosestDist = Dist;
				}
				else {
					break;
				}
			} while (std::size(Sps) < 4);

			OnA.ToZero();
			OnB.ToZero();
			for (auto i = 0; i < std::size(Sps); ++i) {
				OnA += Sps[i].GetA() * Lambda[i];
				OnB += Sps[i].GetB() * Lambda[i];
			}
		}
	}

#ifdef _DEBUG
	static void SignedVolumeTest()
	{
		constexpr auto Eps = 0.001f;	
		{
			const std::vector OrgPts = {
				Vec3(0.0f, 0.0f, 0.0f),
				Vec3(1.0f, 0.0f, 0.0f),
				Vec3(0.0f, 1.0f, 0.0f),
				Vec3(0.0f, 0.0f, 1.0f),
			};
			{
				//!< 検証済
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(1.0f, 1.0f, 1.0f)));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(1.0f, 0.0f, 0.0f, 0.0f);
				const auto CorrectV = Vec3::One();
				if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
					__debugbreak();
				}
				if (!V.NearlyEqual(CorrectV, Eps)) {
					__debugbreak();
				}
			}
			{
				//!< 検証済
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(-1.0f, -1.0f, -1.0f) * 0.25f));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(0.25f, 0.25f, 0.25f, 0.25f);
				const auto CorrectV = Vec3::Zero();
				if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
					__debugbreak();
				}
				if (!V.NearlyEqual(CorrectV, Eps)) {
					__debugbreak();
				}
			}
			{
				//!< 検証済
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(-1.0f, -1.0f, -1.0f)));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(0.0f, 0.333f, 0.333f, 0.333f);
				const auto CorrectV = Vec3(-0.667f, -0.667f, -0.667f);
				if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
					__debugbreak();
				}
				if (!V.NearlyEqual(CorrectV, Eps)) {
					__debugbreak();
				}
			}
			{
				//!< 検証済
				std::vector<Vec3> Pts;
				Pts.resize(std::size(OrgPts));
				std::ranges::transform(OrgPts, std::begin(Pts), std::bind(std::plus(), std::placeholders::_1, Vec3(1.0f, 1.0f, -0.5f)));
				const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
				const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

				const auto CorrectLambda = Vec4(0.5f, 0.0f, 0.0f, 0.5f);
				const auto CorrectV = Vec3(1.0f, 1.0f, 0.0f);
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
				Vec3(51.1996613f, 26.1989613f, 1.91339576f),
				Vec3(-51.0567360f, -26.0565681f, -0.436143428f),
				Vec3(50.8978920f, -24.1035538f, -1.04042661f),
				Vec3(-49.1021080f, 25.8964462f, -1.04042661f)
			};
			const auto Lambda = SignedVolume(Pts[0], Pts[1], Pts[2], Pts[3]);
			const auto V = Pts[0] * Lambda[0] + Pts[1] * Lambda[1] + Pts[2] * Lambda[2] + Pts[3] * Lambda[3];

			//!< 答え合わせ
			const auto CorrectLambda = Vec4(0.290f, 0.302f, 0.206f, 0.202f);
			const auto CorrectV = Vec3::Zero();
			if (!Lambda.NearlyEqual(CorrectLambda, Eps)) {
				__debugbreak();
			}
			if (!V.NearlyEqual(CorrectV, Eps)) {
				__debugbreak();
			}
		}
	}
#endif
}