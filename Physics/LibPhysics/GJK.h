#pragma once

#include <algorithm>
#include <functional>

#include "PhysicsMath.h"
#include "Collision.h"

namespace Physics 
{
	class Shape;
	class RigidBody;
}

namespace Collision
{
	//!< SignedVolue : 射影が最大となる軸や平面を見つけ、それに対し原点を射影して内部にあれば重心を返す
	[[nodiscard]] Math::Vec2 SignedVolume(const Math::Vec3& A, const Math::Vec3& B);
	[[nodiscard]] Math::Vec3 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C);
	[[nodiscard]] Math::Vec4 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D);
	//!< ABC 上での 原点 の重心座標
	bool Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, Math::Vec3& BC);
	//!< ABC 上での Pt の重心座標
	bool Barycentric(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, Math::Vec3& BC);

	namespace SupportPoint {
		//!< サポートポイント : 特定の方向に最も遠い点
		//!< A, B のサポートポイントの差が、(A, B のミンコフスキー差) C のサポートポイントとなる
		//!< ミンコフスキー和の凸包 : Configuration Space Object (CSO)
		class Points
		{
		public:
			Points(const Math::Vec3& A, const Math::Vec3& B) : SPs({ A - B, A, B }) { }

			const Math::Vec3 GetA() const { return SPs[1]; }
			const Math::Vec3 GetB() const { return SPs[2]; }
			const Math::Vec3 GetC() const { return SPs[0]; }

		private:
			std::array<Math::Vec3, 3> SPs;
		};
		[[nodiscard]] static Points Get(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, const Math::Vec3& UDir, const float Bias);
		//[[nodiscard]] static Points Get(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& UDir, const float Bias);
		//!< n-シンプレックスが原点を含むかどうかを返す
		//!< 過程で原点のシンプレックス上での重心座標、原点へのベクトルを求めている
		static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<Points>& Sps, Math::Vec3& Dir, Math::Vec4& OutLambda)
		{
			switch (size(Sps)) {
			case 2:
				//!< 原点のシンプレックス上での重心座標 (シンプレックス上にあるか) を返す
				OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC());
				//!< 原点へのベクトル
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

			//!< 原点へのベクトルがほぼゼロ -> 原点を含む -> 衝突
			constexpr auto Eps2 = 0.0001f * 0.00001f;
			return Dir.LengthSq() < Eps2;
		}

		//!< サポートポイントが四面体をなしていない場合、四面体を形成する
		void ToTetrahedron(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, std::vector<SupportPoint::Points>& Sps);
		void ToTetrahedron(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, std::vector<SupportPoint::Points>& Sps);
		//!< バイアスの分だけ拡張する
		void Expand(const float Bias, std::vector<SupportPoint::Points>& Sps);

		namespace Distance
		{
			//!< 三角形から一番遠い点のイテレータを返す
			[[nodiscard]] static auto Farthest(const Math::Vec3& Pt, const std::vector<SupportPoint::Points>& Sps, const std::vector<Collision::TriInds>& Indices)
			{
				return std::ranges::max_element(Indices, [&](const auto& lhs, const auto& rhs) {
					return std::abs(Collision::Distance::PointTriangle(Pt, Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC())) < std::abs(Collision::Distance::PointTriangle(Pt, Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC()));
				});
			}
			//!< 三角形から一番近い点のイテレータを返す
			[[nodiscard]] static auto Closest(const Math::Vec3& Pt, const std::vector<SupportPoint::Points>& Sps, const std::vector<Collision::TriInds>& Indices)
			{
				return std::ranges::min_element(Indices, [&](const auto& lhs, const auto& rhs) {
					return std::abs(Collision::Distance::PointTriangle(Pt, Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC())) < std::abs(Collision::Distance::PointTriangle(Pt, Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC()));
				});
			}
		}
	}

	namespace Intersection 
	{
		//!< 衝突点算出用 (EPA 等)
		using OnIntersectGJK = std::function<void(const Physics::Shape*, const Math::Vec3&, const Math::Quat&, 
			const Physics::Shape*, const Math::Vec3&, const Math::Quat&, 
			const std::vector<SupportPoint::Points>&, 
			const float, Math::Vec3&, Math::Vec3&)>;
		//!< EPA (Expanding Polytope Algorithm)
		void EPA(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, 
			const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
			const std::vector<SupportPoint::Points>& SupportPoints, const float Bias, 
			Math::Vec3& OnA, Math::Vec3& OnB);
		static void OnIntersectDummy([[maybe_unused]] const Physics::Shape* ShA, [[maybe_unused]] const Math::Vec3& PosA, [[maybe_unused]] const Math::Quat& RotA,
			[[maybe_unused]] const Physics::Shape* ShB, [[maybe_unused]] const Math::Vec3& PosB, [[maybe_unused]] const Math::Quat& RotB,
			[[maybe_unused]] const std::vector<SupportPoint::Points>& SupportPoints, [[maybe_unused]] const float Bias,
			[[maybe_unused]] Math::Vec3& OnA, [[maybe_unused]] Math::Vec3& OnB) {
		}

		//!< GJK 本体
		bool GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
			const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB,
			OnIntersectGJK OnIntersect, const float Bias, 
			const bool WidthClosestPoint,
			Math::Vec3& OnA, Math::Vec3& OnB);
		
		//!< 衝突点を EPA を用いて求める
		//!< 衝突が無い場合は最近接点を求める
		[[nodiscard]] static bool GJK_EPA(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, 
			const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, 
			const float Bias, 
			const bool WidthClosestPoint,
			Math::Vec3& OnA, Math::Vec3& OnB) 
		{
			return GJK(ShA, PosA, RotA, 
				ShB, PosB, RotB, 
				EPA, Bias, 
				WidthClosestPoint,
				OnA, OnB);
		}
		[[nodiscard]] bool GJK_EPA(const Physics::RigidBody* RbA,
			const Physics::RigidBody* RbB,
			const float Bias, 
			bool WidthClosestPoint,
			Math::Vec3& OnA, Math::Vec3& OnB);

		//!< 判定のみ (衝突点、最近接点が不要な場合)
		[[nodiscard]] static bool GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
			const Physics::Shape* ShB,
			const Math::Vec3& PosB, const Math::Quat& RotB)
		{
			Math::Vec3 OnA, OnB;
			return GJK(ShA, PosA, RotA, 
				ShB, PosB, RotB, 
				OnIntersectDummy, 0.0f,
				false,
				OnA, OnB);
		}
		[[nodiscard]] bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
	}
	namespace Closest
	{		
		//!< 最近接点 (非衝突が確定であること)
		void GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, Math::Vec3& OnA, Math::Vec3& OnB);
		void GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, Math::Vec3& OnA, Math::Vec3& OnB);
	}
	
#ifdef _DEBUG
	void SignedVolumeTest();
#endif
}