#pragma once

#include <algorithm>

#include "PhysicsMath.h"

namespace Physics 
{
	class RigidBody;
}
namespace Collision
{
	using TriInds = std::array<uint32_t, 3>;
	using EdgeInds = std::array<uint32_t, 2>;
	using EdgeIndsCount = std::pair<EdgeInds, uint32_t>;

	class AABB
	{
	public:
		AABB() {}
		AABB(const Math::Vec3& MinVal, const Math::Vec3& MaxVal) : Min(MinVal), Max(MaxVal) {}
		AABB(const AABB& rhs) : Min(rhs.Min), Max(rhs.Max) {}
		AABB(const std::vector<Math::Vec3>& Vertices) { Expand(Vertices); }
		AABB& operator=(const AABB& rhs) { Min = rhs.Min; Max = rhs.Max; return *this; }

		AABB& Clear() {
			Min = Math::Vec3::Max();
			Max = Math::Vec3::Min();
			return *this;
		}
		AABB& Expand(const Math::Vec3& rhs) {
			Min = { (std::min)(Min.X(), rhs.X()), (std::min)(Min.Y(), rhs.Y()), (std::min)(Min.Z(), rhs.Z()) };
			Max = { (std::max)(Max.X(), rhs.X()), (std::max)(Max.Y(), rhs.Y()), (std::max)(Max.Z(), rhs.Z()) };
			return *this;
		}
		AABB& Expand(const std::vector<Math::Vec3>& Vertices) { for (auto& i : Vertices) { Expand(i); } return *this; }
		AABB& Expand(const AABB& rhs) {
			Expand(rhs.Min);
			Expand(rhs.Max);
			return *this;
		}

		[[nodiscard]] Math::Vec3 GetCenter() const { return (Min + Max) * 0.5f; }
		[[nodiscard]] Math::Vec3 GetExtent() const { return Max - Min; }

		Math::Vec3 Min = Math::Vec3::Max();
		Math::Vec3 Max = Math::Vec3::Min();
	};

	struct BoundEdge
	{
		int Index;
		float Value;
		bool IsMin;
	};

//!< �s�v�ȋC������
//#define WORLD_CONTACT_POINT
	struct Contact
	{
		float TimeOfImpact = 0.0f;

		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		//!< ���[�J���X�y�[�X
		Math::Vec3 PointA;
		Math::Vec3 PointB;

#ifdef WORLD_CONTACT_POINT
		//!< ���[���h�X�y�[�X
		Math::Vec3 WPointA;
		Math::Vec3 WPointB;
#endif

		//!< ���[���h�X�y�[�X A -> B
		Math::Vec3 Normal;

		Contact& Swap() {
			std::swap(RigidBodyA, RigidBodyB);
			std::swap(PointA, PointB);
#ifdef WORLD_CONTACT_POINT
			std::swap(WPointA, WPointB);
#endif
			Normal = -Normal;
			return *this;
		}
	};

	namespace Distance 
	{
		[[nodiscard]] static float AABBPointSq(const AABB& Ab,
			const Math::Vec3& Pt) {
			return Math::Vec3(std::max(std::max(Ab.Min.X() - Pt.X(), 0.0f), std::max(Pt.X() - Ab.Max.X(), 0.0f)),
				std::max(std::max(Ab.Min.Y() - Pt.Y(), 0.0f), std::max(Pt.Y() - Ab.Max.Y(), 0.0f)),
				std::max(std::max(Ab.Min.Z() - Pt.Z(), 0.0f), std::max(Pt.Z() - Ab.Max.Z(), 0.0f))).LengthSq();
		}

		[[nodiscard]] static float PointRaySq(const Math::Vec3& Pt, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			const auto ToPt = Pt - RayPos;
			return ((RayDir * ToPt.Dot(RayDir)) - ToPt).LengthSq();
		}
		[[nodiscard]] static float PointRay(const Math::Vec3& Pt, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return std::sqrtf(PointRaySq(Pt, RayPos, RayDir));
		}
		[[nodiscard]] static float PointTriangle(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			return (Pt - A).Dot(Math::Vec3::UnitNormal(A, B, C));
		}
		[[nodiscard]] static bool IsFront(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			//!< ������������΂悢�̂Ő��K������K�v�͂Ȃ�
			return (Pt - A).Dot(Math::Vec3::Normal(A, B, C)) >= 0.0f;
		}

		//!< �w��̕����Ɉ�ԉ����_�̃C�e���[�^��Ԃ� 
		[[nodiscard]] static auto Farthest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& Dir) {
			return std::ranges::max_element(Pts, [&](const auto& lhs, const auto& rhs) { return Dir.Dot(lhs) < Dir.Dot(rhs); });
		}
		//!< �w��̕����Ɉ�ԋ߂��_�̃C�e���[�^��Ԃ� 
		[[nodiscard]] static auto Closest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& Dir) {
			return std::ranges::min_element(Pts, [&](const auto& lhs, const auto& rhs) { return Dir.Dot(lhs) < Dir.Dot(rhs); });
		}
		//!< ���C�����ԉ����_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Farthest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return std::ranges::max_element(Pts, [&](const auto& lhs, const auto& rhs) { return PointRaySq(lhs, RayPos, RayDir) < PointRaySq(rhs, RayPos, RayDir); });
		}
		//!< ���C�����ԋ߂��_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Closest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return std::ranges::min_element(Pts, [&](const auto& lhs, const auto& rhs) { return PointRaySq(lhs, RayPos, RayDir) < PointRaySq(rhs, RayPos, RayDir); });
		}
		//!< �O�p�`�����ԉ����_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Farthest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			return std::ranges::max_element(Pts, [&](const auto& lhs, const auto& rhs) { return PointTriangle(lhs, A, B, C) < PointTriangle(rhs, A, B, C); });
		}
		//!< �O�p�`�����ԋ߂��_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Closest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			return std::ranges::min_element(Pts, [&](const auto& lhs, const auto& rhs) { return PointTriangle(lhs, A, B, C) < PointTriangle(rhs, A, B, C); });
		}
	}

	namespace Intersection 
	{
		//!< AABB vs AABB
		[[nodiscard]] static bool AABBAABB(const AABB& AbA, const AABB& AbB) {
			if (AbA.Max.X() < AbB.Min.X() || AbB.Max.X() < AbA.Min.X() ||
				AbA.Max.Y() < AbB.Min.Y() || AbB.Max.Y() < AbA.Min.Y() ||
				AbA.Max.Z() < AbB.Min.Z() || AbB.Max.Z() < AbA.Min.Z()) {
				return false;
			}
			return true;
		}
		[[nodiscard]] bool AABBAABB(const AABB& AbA, const AABB& AbB,
			const Math::Vec3& VelA, const Math::Vec3& VelB,
			float& T);
		[[nodiscard]] bool AABBRay(const AABB& Ab,
			const Math::Vec3& RayPos, const Math::Vec3& RayDir,
			float& T);
		[[nodiscard]] static bool AABBSegment(const AABB& Ab,
			const Math::Vec3& SegA, const Math::Vec3& SegB,
			float& T) {
			return AABBRay(Ab, SegA, (SegB - SegA), T) && T <= 1.0f; 
		}
		[[nodiscard]] static bool AABBSphere(const AABB& Ab,
			const Math::Vec3& SpPos, const float SpRad) {
			return Distance::AABBPointSq(Ab, SpPos) <= std::powf(SpRad, 2.0f);
		}


		//!< ���C vs ��
		[[nodiscard]] bool RaySphere(const Math::Vec3& RayPos, const Math::Vec3& RayDir, const Math::Vec3& SpPos, const float SpRad, float& T0, float& T1);
	
		//!< �� vs ��
		[[nodiscard]] static bool SphereShpere(const float RadA, const float RadB, const Math::Vec3& PosA, const Math::Vec3& PosB) {
			return (PosB - PosA).LengthSq() <= std::powf(RadA + RadB, 2.0f); 
		}
		[[nodiscard]] bool SphereShpere(const float RadA, const float RadB,
			const Math::Vec3& PosA, const Math::Vec3& PosB,
			const Math::Vec3& VelA, const Math::Vec3& VelB,
			float& T);

		[[nodiscard]] bool RigidBodyRigidBody(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const float DeltaSec, Contact& Ct);
	}

	namespace Volume
	{
		[[nodiscard]] static float Tetrahedron(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D)
		{
			return std::fabsf((D - A).Dot((D - B).Cross(D - C)) / 6.0f);
		}
	}

	//!< �Փˎ��̗͐ς̓K�p
	void ResolveContact(const Contact& Ct);
}