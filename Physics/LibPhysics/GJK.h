#pragma once

#include <algorithm>
#include <functional>

#include "PhysicsMath.h"

namespace Physics 
{
	class RigidBody;
}

template<typename T> static [[nodiscard]] size_t IndexOf(const std::vector<T>& Vector, const T& rhs) {
	return static_cast<size_t>(&rhs - &*std::begin(Vector));
}

namespace Collision
{
	//!< SignedVolue : 射影が最大となる軸や平面を見つけ、それに対し原点を射影して内部にあれば重心を返す

	//!< 検証済
	[[nodiscard]] Math::Vec2 SignedVolume(const Math::Vec3& A, const Math::Vec3& B);
	//!< 検証済
	[[nodiscard]] Math::Vec3 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C);
	//!< 検証済
	[[nodiscard]] Math::Vec4 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D);
	[[nodiscard]] Math::Vec3 Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& Pt);

	namespace SupportPoint {
		//!< サポートポイント : 特定の方向に最も遠い点

		//!< A, B のサポートポイントの差が、(A, B のミンコフスキー差) C のサポートポイントとなる
		class Points
		{
		public:
			Points(const Math::Vec3& A, const Math::Vec3& B) : SPs({ A - B, A, B }) { }

			const Math::Vec3 GetA() const { return SPs[1]; }
			const Math::Vec3 GetB() const { return SPs[2]; }
			const Math::Vec3 GetC() const { return SPs[0]; }

			bool operator == (const Points& i) const { return std::ranges::equal(SPs, i.SPs); }
		private:
			std::array<Math::Vec3, 3> SPs;
		};
		[[nodiscard]] Points GetPoints(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& UDir, const float Bias);
		static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<Points>& Sps, Math::Vec3& Dir, Math::Vec4& OutLambda)
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
		void ToTetrahedron(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, std::vector<SupportPoint::Points>& Sps);
		//!< バイアスの分だけ拡張する
		void Expand(const float Bias, std::vector<SupportPoint::Points>& Sps);
	}

	namespace Intersection 
	{
		using OnIntersectGJK = std::function<void(const Physics::RigidBody*, const Physics::RigidBody*, const std::vector<SupportPoint::Points>&, const float, Math::Vec3&, Math::Vec3&)>;
		
		[[nodiscard]] bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, OnIntersectGJK OnIntersect, const float Bias, Math::Vec3& OnA, Math::Vec3& OnB);
		
		//!< EPA (Expanding Polytope Algorithm)
		void EPA(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const std::vector<SupportPoint::Points>& SupportPoints, const float Bias, Math::Vec3& OnA, Math::Vec3& OnB);
		
		[[nodiscard]] static bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB) 
		{
			Math::Vec3 OnA, OnB;
			return GJK(RbA, RbB, [](const Physics::RigidBody*, const Physics::RigidBody*, const std::vector<SupportPoint::Points>&, const float, Math::Vec3&, Math::Vec3&) {}, 0.001f, OnA, OnB);
		}
		[[nodiscard]] static bool GJK_EPA(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const float Bias, Math::Vec3& OnA, Math::Vec3& OnB) 
		{
			return GJK(RbA, RbB, EPA, Bias, OnA, OnB);
		}
	}
	//namespace Closest 
	//{
	//	//!< 非衝突が確定であること
	//	void GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, Math::Vec3& OnA, Math::Vec3& OnB);
	//}

#ifdef _DEBUG
	void SignedVolumeTest();
#endif
}