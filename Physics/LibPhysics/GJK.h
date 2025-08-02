#pragma once

#include <algorithm>
#include <functional>
#include <optional>

#include "LinAlg.h"
#include "Collision.h"

namespace Physics 
{
	class Shape;
	class RigidBody;
}

namespace Collision
{
	//!< SignedVolue : 射影が最大となる軸や平面を見つけ、それに対し原点を射影して内部にあれば重心を返す
	[[nodiscard]] LinAlg::Vec2 SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B);
	[[nodiscard]] LinAlg::Vec3 SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C);
	[[nodiscard]] LinAlg::Vec4 SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D);
	
	//!< ABC 上での 原点 の重心座標
	[[nodiscard]] std::optional<LinAlg::Vec3> Barycentric(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C);
	//!< ABC 上での Pt の重心座標
	[[nodiscard]] std::optional<LinAlg::Vec3> Barycentric(const LinAlg::Vec3& Pt, const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C);

	namespace SupportPoint {
		//!< サポートポイント : 特定の方向に最も遠い点
		//!< A, B のサポートポイントの差が、(A, B のミンコフスキー差) C のサポートポイントとなる
		//!< ミンコフスキー和の凸包 : Configuration Space Object (CSO)
		class Points
		{
		public:
			Points(const LinAlg::Vec3& A, const LinAlg::Vec3& B) : SPs({ A - B, A, B }) { }

			const LinAlg::Vec3 GetA() const { return SPs[1]; }
			const LinAlg::Vec3 GetB() const { return SPs[2]; }
			const LinAlg::Vec3 GetC() const { return SPs[0]; }

		private:
			std::array<LinAlg::Vec3, 3> SPs;
		};
		[[nodiscard]] static Points Get(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
			const LinAlg::Vec3& UDir, const float Bias);
		[[nodiscard]] static Points Get(const Physics::RigidBody* RbA, 
			const Physics::RigidBody* RbB, 
			const LinAlg::Vec3& UDir, const float Bias);
		//!< n-シンプレックスが原点を含むかどうかを返す
		//!< 過程で原点のシンプレックス上での重心座標、原点へのベクトルを求めている
		static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<Points>& Sps,
			LinAlg::Vec3& Dir, LinAlg::Vec4& Lambda)
		{
			switch (std::size(Sps)) {
			case 2:
				//!< 原点のシンプレックス上での重心座標 (シンプレックス上にあるか) を返す
				Lambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC());
				//!< 原点へのベクトル
				Dir = -(Sps[0].GetC() * Lambda[0] + Sps[1].GetC() * Lambda[1]);
				break;
			case 3:
				Lambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC(), Sps[2].GetC());
				Dir = -(Sps[0].GetC() * Lambda[0] + Sps[1].GetC() * Lambda[1] + Sps[2].GetC() * Lambda[2]);
				break;
			case 4:
				Lambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC(), Sps[2].GetC(), Sps[3].GetC());
				Dir = -(Sps[0].GetC() * Lambda[0] + Sps[1].GetC() * Lambda[1] + Sps[2].GetC() * Lambda[2] + Sps[3].GetC() * Lambda[3]);
				break;
			default:
				__debugbreak();
				break;
			}

			//!< 原点へのベクトルがほぼゼロ -> 原点を含む -> 衝突
			constexpr auto Eps2 = 0.0001f * 0.00001f;
			return Dir.LengthSq() < Eps2;
		}

		//!< サポートポイントが四面体をなしていない場合、四面体を形成する
		static void ToTetrahedron(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB,
			std::vector<SupportPoint::Points>& Sps);
		//!< バイアスの分だけ拡張する
		static LinAlg::Vec3 Expand(const float Bias, std::vector<SupportPoint::Points>& Sps);

		namespace Distance
		{
			//!< 点から一番遠い三角形インデックスのイテレータを返す
			[[nodiscard]] static auto Farthest(const LinAlg::Vec3& Pt,
				const std::vector<SupportPoint::Points>& Sps, const std::vector<Collision::TriInds>& Indices) {
				return std::ranges::max_element(Indices, 
					[&](const auto& lhs, const auto& rhs) {
						using namespace Collision::Distance;
						return std::abs(PointTriangle(Pt, Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC())) < std::abs(PointTriangle(Pt, Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC()));
					});
			}
			//!< 点から一番近い三角形インデックスのイテレータを返す
			[[nodiscard]] static auto Closest(const LinAlg::Vec3& Pt, 
				const std::vector<SupportPoint::Points>& Sps, const std::vector<Collision::TriInds>& Indices) {
				return std::ranges::min_element(Indices, 
					[&](const auto& lhs, const auto& rhs) {
						using namespace Collision::Distance;
						return std::abs(PointTriangle(Pt, Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC())) < std::abs(PointTriangle(Pt, Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC()));
					});
			}
		}
	}

	namespace Intersection 
	{
		//!< 衝突点を求める関数 (EPA 等)
		using OnIntersectGJK = std::function<void(const Physics::Shape*, const LinAlg::Vec3&, const LinAlg::Quat&, 
			const Physics::Shape*, const LinAlg::Vec3&, const LinAlg::Quat&, 
			std::vector<SupportPoint::Points>&, const float,
			LinAlg::Vec3&, LinAlg::Vec3&)>;

		//!< GJK 本体
		bool GJK(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB,
			OnIntersectGJK OnIntersect, const float Bias, const bool zWithClosestPoint,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
		//!< 引数簡易版
		[[nodiscard]] bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
			OnIntersectGJK OnIntersect, const float Bias,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
		//!< 衝突検出のみ (衝突点を求めない)
		[[nodiscard]] static bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
			const float Bias,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
			return GJK(RbA, RbB, nullptr, Bias, OnA, OnB);
		}

		//!< EPA (Expanding Polytope Algorithm) 衝突点検出
		void EPA(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
			std::vector<SupportPoint::Points>& Sps, const float Bias, 
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
		
		//!< 衝突点を EPA を用いて求める
		[[nodiscard]] static bool GJK_EPA(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
			const float Bias, const bool WidthClosestPoint,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
			return GJK(ShA, PosA, RotA, 
				ShB, PosB, RotB, 
				EPA, Bias, WidthClosestPoint,
				OnA, OnB);
		}
		//!< 引数簡易版、EPA で衝突点を求める、無い場合は最近接点を求める
		[[nodiscard]] bool GJK_EPA(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
			const float Bias,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);

		//!< 判定のみ (衝突点、最近接点が不要な場合)
		[[nodiscard]] static bool GJK(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB) {
			LinAlg::Vec3 OnA, OnB;
			return GJK(ShA, PosA, RotA, 
				ShB, PosB, RotB, 
				nullptr, 0.0f,
				false,
				OnA, OnB);
		}
		[[nodiscard]] bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
	}

	//!< 最近接点 (衝突していないことが前提)
	namespace Closest
	{
		static void GJK(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
			Intersection::GJK(ShA, PosA, RotA,
				ShB, PosB, RotB,
				nullptr, 0.0f,
				true,
				OnA, OnB);
		}
		void GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, 
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
	}
	
#ifdef _DEBUG
	void SignedVolumeTest();
#endif
}