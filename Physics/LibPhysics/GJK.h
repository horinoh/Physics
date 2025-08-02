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
	//!< SignedVolue : �ˉe���ő�ƂȂ鎲�╽�ʂ������A����ɑ΂����_���ˉe���ē����ɂ���Ώd�S��Ԃ�
	[[nodiscard]] LinAlg::Vec2 SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B);
	[[nodiscard]] LinAlg::Vec3 SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C);
	[[nodiscard]] LinAlg::Vec4 SignedVolume(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D);
	
	//!< ABC ��ł� ���_ �̏d�S���W
	[[nodiscard]] std::optional<LinAlg::Vec3> Barycentric(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C);
	//!< ABC ��ł� Pt �̏d�S���W
	[[nodiscard]] std::optional<LinAlg::Vec3> Barycentric(const LinAlg::Vec3& Pt, const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C);

	namespace SupportPoint {
		//!< �T�|�[�g�|�C���g : ����̕����ɍł������_
		//!< A, B �̃T�|�[�g�|�C���g�̍����A(A, B �̃~���R�t�X�L�[��) C �̃T�|�[�g�|�C���g�ƂȂ�
		//!< �~���R�t�X�L�[�a�̓ʕ� : Configuration Space Object (CSO)
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
		//!< n-�V���v���b�N�X�����_���܂ނ��ǂ�����Ԃ�
		//!< �ߒ��Ō��_�̃V���v���b�N�X��ł̏d�S���W�A���_�ւ̃x�N�g�������߂Ă���
		static [[nodiscard]] bool SimplexSignedVolumes(const std::vector<Points>& Sps,
			LinAlg::Vec3& Dir, LinAlg::Vec4& Lambda)
		{
			switch (std::size(Sps)) {
			case 2:
				//!< ���_�̃V���v���b�N�X��ł̏d�S���W (�V���v���b�N�X��ɂ��邩) ��Ԃ�
				Lambda = SignedVolume(Sps[0].GetC(), Sps[1].GetC());
				//!< ���_�ւ̃x�N�g��
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

			//!< ���_�ւ̃x�N�g�����قڃ[�� -> ���_���܂� -> �Փ�
			constexpr auto Eps2 = 0.0001f * 0.00001f;
			return Dir.LengthSq() < Eps2;
		}

		//!< �T�|�[�g�|�C���g���l�ʑ̂��Ȃ��Ă��Ȃ��ꍇ�A�l�ʑ̂��`������
		static void ToTetrahedron(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB,
			std::vector<SupportPoint::Points>& Sps);
		//!< �o�C�A�X�̕������g������
		static LinAlg::Vec3 Expand(const float Bias, std::vector<SupportPoint::Points>& Sps);

		namespace Distance
		{
			//!< �_�����ԉ����O�p�`�C���f�b�N�X�̃C�e���[�^��Ԃ�
			[[nodiscard]] static auto Farthest(const LinAlg::Vec3& Pt,
				const std::vector<SupportPoint::Points>& Sps, const std::vector<Collision::TriInds>& Indices) {
				return std::ranges::max_element(Indices, 
					[&](const auto& lhs, const auto& rhs) {
						using namespace Collision::Distance;
						return std::abs(PointTriangle(Pt, Sps[lhs[0]].GetC(), Sps[lhs[1]].GetC(), Sps[lhs[2]].GetC())) < std::abs(PointTriangle(Pt, Sps[rhs[0]].GetC(), Sps[rhs[1]].GetC(), Sps[rhs[2]].GetC()));
					});
			}
			//!< �_�����ԋ߂��O�p�`�C���f�b�N�X�̃C�e���[�^��Ԃ�
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
		//!< �Փ˓_�����߂�֐� (EPA ��)
		using OnIntersectGJK = std::function<void(const Physics::Shape*, const LinAlg::Vec3&, const LinAlg::Quat&, 
			const Physics::Shape*, const LinAlg::Vec3&, const LinAlg::Quat&, 
			std::vector<SupportPoint::Points>&, const float,
			LinAlg::Vec3&, LinAlg::Vec3&)>;

		//!< GJK �{��
		bool GJK(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA,
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB,
			OnIntersectGJK OnIntersect, const float Bias, const bool zWithClosestPoint,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
		//!< �����ȈՔ�
		[[nodiscard]] bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
			OnIntersectGJK OnIntersect, const float Bias,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
		//!< �Փˌ��o�̂� (�Փ˓_�����߂Ȃ�)
		[[nodiscard]] static bool GJK(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
			const float Bias,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
			return GJK(RbA, RbB, nullptr, Bias, OnA, OnB);
		}

		//!< EPA (Expanding Polytope Algorithm) �Փ˓_���o
		void EPA(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
			std::vector<SupportPoint::Points>& Sps, const float Bias, 
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);
		
		//!< �Փ˓_�� EPA ��p���ċ��߂�
		[[nodiscard]] static bool GJK_EPA(const Physics::Shape* ShA, const LinAlg::Vec3& PosA, const LinAlg::Quat& RotA, 
			const Physics::Shape* ShB, const LinAlg::Vec3& PosB, const LinAlg::Quat& RotB, 
			const float Bias, const bool WidthClosestPoint,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
			return GJK(ShA, PosA, RotA, 
				ShB, PosB, RotB, 
				EPA, Bias, WidthClosestPoint,
				OnA, OnB);
		}
		//!< �����ȈՔŁAEPA �ŏՓ˓_�����߂�A�����ꍇ�͍ŋߐړ_�����߂�
		[[nodiscard]] bool GJK_EPA(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB,
			const float Bias,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB);

		//!< ����̂� (�Փ˓_�A�ŋߐړ_���s�v�ȏꍇ)
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

	//!< �ŋߐړ_ (�Փ˂��Ă��Ȃ����Ƃ��O��)
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