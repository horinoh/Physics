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
	struct Contact
	{
		float TimeOfImpact = 0.0f;

		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		Math::Vec3 PointA;
		Math::Vec3 PointB;

		Math::Vec3 Normal;

		Contact& Swap() {
			std::swap(RigidBodyA, RigidBodyB);
			std::swap(PointA, PointB);
			Normal = -Normal;
			return *this;
		}
	};

	namespace Distance 
	{
		[[nodiscard]] static float PointRaySq(const Math::Vec3& Pt, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			const auto ToPt = Pt - RayPos;
			return ((RayDir * ToPt.Dot(RayDir)) - ToPt).LengthSq();
		}
		[[nodiscard]] static float PointRay(const Math::Vec3& Pt, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return sqrtf(PointRaySq(Pt, RayPos, RayDir));
		}
		[[nodiscard]] static float PointTriangle(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			return (Pt - A).Dot(Math::Vec3::UnitNormal(A, B, C));
		}
		[[nodiscard]] static bool IsFront(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) { return PointTriangle(Pt, A, B, C) >= 0.0f; }

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
		[[nodiscard]] static bool AABBAABB(const AABB& lhs, const AABB& rhs) {
			if (lhs.Max.X() < rhs.Min.X() || rhs.Max.X() < lhs.Min.X() ||
				lhs.Max.Y() < rhs.Min.Y() || rhs.Max.Y() < lhs.Min.Y() ||
				lhs.Max.Z() < rhs.Min.Z() || rhs.Max.Z() < lhs.Min.Z()) {
				return false;
			}
			return true;
		}
		//!< ���C vs ��
		[[nodiscard]] bool RaySphere(const Math::Vec3& RayPos, const Math::Vec3& RayDir, const Math::Vec3& SpPos, const float SpRad, float& T0, float& T1);
		//!< �� vs ��
		[[nodiscard]] bool SphereShpere(const float RadA, const float RadB,
			const Math::Vec3& PosA, const Math::Vec3& PosB,
			const Math::Vec3& VelA, const Math::Vec3& VelB,
			const float DeltaSec, float& T);
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
	void Resolve(const Contact& Ct);
}