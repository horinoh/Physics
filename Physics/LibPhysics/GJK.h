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
	//!< SignedVolue : �ˉe���ő�ƂȂ鎲�╽�ʂ������A����ɑ΂����_���ˉe���ē����ɂ���Ώd�S��Ԃ�
	[[nodiscard]] Math::Vec2 SignedVolume(const Math::Vec3& A, const Math::Vec3& B);
	[[nodiscard]] Math::Vec3 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C);
	[[nodiscard]] Math::Vec4 SignedVolume(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D);
	//!< ABC ��ł� ���_ �̏d�S���W
	bool Barycentric(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, Math::Vec3& BC);
	//!< ABC ��ł� Pt �̏d�S���W
	bool Barycentric(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, Math::Vec3& BC);

	namespace SupportPoint {
		//!< �T�|�[�g�|�C���g : ����̕����ɍł������_
		//!< A, B �̃T�|�[�g�|�C���g�̍����A(A, B �̃~���R�t�X�L�[��) C �̃T�|�[�g�|�C���g�ƂȂ�
		//!< �~���R�t�X�L�[�a�̓ʕ� : Configuration Space Object (CSO)
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
		//!< n-�V���v���b�N�X�����_���܂ނ��ǂ�����Ԃ�
		//!< �ߒ��Ō��_�̃V���v���b�N�X��ł̏d�S���W�A���_�ւ̃x�N�g�������߂Ă���
		static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<Points>& Sps, Math::Vec3& Dir, Math::Vec4& OutLambda)
		{
			switch (size(Sps)) {
			case 2:
				//!< ���_�̃V���v���b�N�X��ł̏d�S���W (�V���v���b�N�X��ɂ��邩) ��Ԃ�
				OutLambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC());
				//!< ���_�ւ̃x�N�g��
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

			//!< ���_�ւ̃x�N�g�����قڃ[�� -> ���_���܂� -> �Փ�
			constexpr auto Eps2 = 0.0001f * 0.00001f;
			return Dir.LengthSq() < Eps2;
		}

		//!< �T�|�[�g�|�C���g���l�ʑ̂��Ȃ��Ă��Ȃ��ꍇ�A�l�ʑ̂��`������
		void ToTetrahedron(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, std::vector<SupportPoint::Points>& Sps);
		void ToTetrahedron(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, std::vector<SupportPoint::Points>& Sps);
		//!< �o�C�A�X�̕������g������
		void Expand(const float Bias, std::vector<SupportPoint::Points>& Sps);

		namespace Distance
		{
			//!< �O�p�`�����ԉ����_�̃C�e���[�^��Ԃ�
			[[nodiscard]] static auto Farthest(const Math::Vec3& Pt, const std::vector<SupportPoint::Points>& Sps, const std::vector<Collision::TriInds>& Indices)
			{
				return std::ranges::max_element(Indices, [&](const auto& lhs, const auto& rhs) {
					return std::abs(Collision::Distance::PointTriangle(Pt, Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC())) < std::abs(Collision::Distance::PointTriangle(Pt, Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC()));
				});
			}
			//!< �O�p�`�����ԋ߂��_�̃C�e���[�^��Ԃ�
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
		//!< �Փ˓_�Z�o�p (EPA ��)
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

		//!< GJK �{��
		bool GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA,
			const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB,
			OnIntersectGJK OnIntersect, const float Bias, 
			const bool WidthClosestPoint,
			Math::Vec3& OnA, Math::Vec3& OnB);
		
		//!< �Փ˓_�� EPA ��p���ċ��߂�
		//!< �Փ˂������ꍇ�͍ŋߐړ_�����߂�
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

		//!< ����̂� (�Փ˓_�A�ŋߐړ_���s�v�ȏꍇ)
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
		//!< �ŋߐړ_ (��Փ˂��m��ł��邱��)
		void GJK(const Physics::Shape* ShA, const Math::Vec3& PosA, const Math::Quat& RotA, const Physics::Shape* ShB, const Math::Vec3& PosB, const Math::Quat& RotB, Math::Vec3& OnA, Math::Vec3& OnB);
		void GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, Math::Vec3& OnA, Math::Vec3& OnB);
	}
	
#ifdef _DEBUG
	void SignedVolumeTest();
#endif
}