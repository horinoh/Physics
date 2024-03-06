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
	//!< SignedVolue : �ˉe���ő�ƂȂ鎲�╽�ʂ������A����ɑ΂����_���ˉe���ē����ɂ���Ώd�S��Ԃ�

	//!< ���؍�
	[[nodiscard]] Math::Vec2 SignedVolume(const Math::Vec3& A, const Math::Vec3& B);
	//!< ���؍�
	[[nodiscard]] Math::Vec3 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C);
	//!< ���؍�
	[[nodiscard]] Math::Vec4 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D);
	[[nodiscard]] Math::Vec3 Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& Pt);

	namespace SupportPoint {
		//!< �T�|�[�g�|�C���g : ����̕����ɍł������_

		//!< A, B �̃T�|�[�g�|�C���g�̍����A(A, B �̃~���R�t�X�L�[��) C �̃T�|�[�g�|�C���g�ƂȂ�
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

			//!< ���_���܂� -> �Փ�
			return Dir.LengthSq() < Eps2;
		}

		//!< �T�|�[�g�|�C���g���l�ʑ̂��Ȃ��Ă��Ȃ��ꍇ�A�l�ʑ̂��`������
		void ToTetrahedron(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, std::vector<SupportPoint::Points>& Sps);
		//!< �o�C�A�X�̕������g������
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
	//	//!< ��Փ˂��m��ł��邱��
	//	void GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, Math::Vec3& OnA, Math::Vec3& OnB);
	//}

#ifdef _DEBUG
	void SignedVolumeTest();
#endif
}