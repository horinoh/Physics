#pragma once

#include <algorithm>
#include <optional>

#include "LinAlg.h"

namespace Physics 
{
	class Shape;
	class RigidBody;
}
namespace Collision
{
	using TriInds = std::array<uint32_t, 3>;
	using EdgeInds = std::array<uint32_t, 2>;
	using EdgeIndsWithCount = std::pair<EdgeInds, uint32_t>;

	class AABB
	{
	public:
		AABB() {}
		AABB(const LinAlg::Vec3& MinVal, const LinAlg::Vec3& MaxVal) : Min(MinVal), Max(MaxVal) {}
		AABB(const AABB& rhs) : Min(rhs.Min), Max(rhs.Max) {}
		AABB(const std::vector<LinAlg::Vec3>& Vertices) { Expand(Vertices); }
		AABB& operator=(const AABB& rhs) { 
			Min = rhs.Min; 
			Max = rhs.Max; 
			return *this; 
		}

		AABB& Clear() {
			Min = LinAlg::Vec3::Max();
			Max = LinAlg::Vec3::Min();
			return *this;
		}
		AABB& Expand(const LinAlg::Vec3& rhs) {
			Min = { (std::min)(Min.X(), rhs.X()), (std::min)(Min.Y(), rhs.Y()), (std::min)(Min.Z(), rhs.Z()) };
			Max = { (std::max)(Max.X(), rhs.X()), (std::max)(Max.Y(), rhs.Y()), (std::max)(Max.Z(), rhs.Z()) };
			return *this;
		}
		AABB& Expand(const std::vector<LinAlg::Vec3>& Vertices) { 
			for (auto& i : Vertices) { 
				Expand(i);
			}
			return *this;
		}
		AABB& Expand(const AABB& rhs) {
			Expand(rhs.Min);
			Expand(rhs.Max);
			return *this;
		}

		[[nodiscard]] LinAlg::Vec3 GetCenter() const { 
			//return (Min + Max) * 0.5f; 
			return LinAlg::Vec3(std::midpoint(Min.X(), Max.X()), std::midpoint(Min.Y(), Max.Y()), std::midpoint(Min.Z(), Max.Z()));
		}
		[[nodiscard]] LinAlg::Vec3 GetExtent() const { return Max - Min; }

		LinAlg::Vec3 Min = LinAlg::Vec3::Max();
		LinAlg::Vec3 Max = LinAlg::Vec3::Min();
	};

	struct BoundEdge
	{
		int Index;
		float Value;
		bool IsMin;
	};

	struct ContactBase
	{
		float TimeOfImpact = 0.0f;

		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		//!< ���[���h�X�y�[�X�Փ˓_
		LinAlg::Vec3 WPointA;
		LinAlg::Vec3 WPointB;

		//!< ���[���h�X�y�[�X�@��
		LinAlg::Vec3 WNormal;

		ContactBase& Swap() {
			std::swap(RigidBodyA, RigidBodyB);
			WNormal = -WNormal;
			return *this;
		}
	};
	struct Contact : ContactBase
	{
		//!< ���[�J���X�y�[�X�Փ˓_
		//!< (ConstraintPenetration �Ŏg�p�A���[�J���X�y�[�X�Փ˓_���o�������ĐV�����g�����X�t�H�[���ŕϊ����Ďg��)
		LinAlg::Vec3 LPointA;
		LinAlg::Vec3 LPointB;

		void CalcLocal();

		Contact& Swap() {
			ContactBase::Swap();
			std::swap(LPointA, LPointB);
			return *this;
		}
	};

	namespace Distance 
	{
		[[nodiscard]] static float AABBPointSq(const AABB& Ab, const LinAlg::Vec3& Pt) {
			return LinAlg::Vec3(std::max(std::max(Ab.Min.X() - Pt.X(), 0.0f), std::max(Pt.X() - Ab.Max.X(), 0.0f)),
				std::max(std::max(Ab.Min.Y() - Pt.Y(), 0.0f), std::max(Pt.Y() - Ab.Max.Y(), 0.0f)),
				std::max(std::max(Ab.Min.Z() - Pt.Z(), 0.0f), std::max(Pt.Z() - Ab.Max.Z(), 0.0f))).LengthSq();
		}
		[[nodiscard]] static float AABBPoint(const AABB& Ab, const LinAlg::Vec3& Pt) {
			return  std::sqrtf(AABBPoint(Ab, Pt));
		}

		[[nodiscard]] float CapsulePointSq(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
			const LinAlg::Vec3& Pt);
		[[nodiscard]] static float CapsulePoint(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
			const LinAlg::Vec3& Pt) {
			return std::sqrtf(CapsulePointSq(CapA, CapB, CapR, Pt));
		}

		[[nodiscard]] float CylinderPointSq(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
			const LinAlg::Vec3& Pt);
		[[nodiscard]] static float CylinderPoint(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
			const LinAlg::Vec3& Pt) {
			return std::sqrtf(CylinderPointSq(CyA, CyB, CyR, Pt));
		}

		[[nodiscard]] static float PointRaySq(const LinAlg::Vec3& Pt, const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir) {
			const auto ToPt = Pt - RayPos;
			return ((RayDir * ToPt.Dot(RayDir)) - ToPt).LengthSq();
		}
		[[nodiscard]] static float PointRay(const LinAlg::Vec3& Pt, const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir) {
			return std::sqrtf(PointRaySq(Pt, RayPos, RayDir));
		}

		[[nodiscard]] static float PointSegmentSq(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB) {
			const auto AB = SegB - SegA;
			const auto AP = Pt - SegA;
			const auto T = AP.Dot(AB);
			if (T <= 0.0f) { return AP.LengthSq(); }
			const auto T1 = AB.LengthSq();
			if (T1 <= T) { return (Pt - SegB).LengthSq(); }
			return AP.LengthSq() - std::pow(T, 2.0f) / T1;
		}
		[[nodiscard]] static float PointSegment(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB) {
			return std::sqrtf(PointSegmentSq(Pt, SegA, SegB));
		}

		[[nodiscard]] static float PointSphereSq(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SpPos, const float SpRad) {
			return std::max((Pt - SpPos).LengthSq() - std::pow(SpRad, 2.0f), 0.0f);
		}
		[[nodiscard]] static float PointSphere(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SpPos, const float SpRad) {
			return std::sqrtf(PointSphereSq(Pt, SpPos, SpRad));
		}

		[[nodiscard]] static float PointTriangle(const LinAlg::Vec3& Pt, const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C) {
			return (Pt - A).Dot(LinAlg::Vec3::UnitNormal(A, B, C));
		}
		[[nodiscard]] static bool IsFront(const LinAlg::Vec3& Pt, const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C) {
			//!< ������������΂悢�̂Ő��K������K�v�͂Ȃ�
			return (Pt - A).Dot(LinAlg::Vec3::Normal(A, B, C)) >= 0.0f;
		}

		[[nodiscard]] static float PlanePoint(const LinAlg::Vec3& PlN, const float PlD,
			const LinAlg::Vec3 Pt) {
			return PlN.Dot(Pt) + PlD;
		}

		[[nodiscard]] float SegmentSegmentSq(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
			float& T0, float& T1);
		[[nodiscard]] float SegmentSegmentSq(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD);
		[[nodiscard]] static float SegmentSegment(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
			float& T0, float& T1) {
			return std::sqrtf(SegmentSegmentSq(SegA, SegB, SegC, SegD, T0, T1));
		}
		[[nodiscard]] static float SegmentSegment(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD) {
			return std::sqrtf(SegmentSegmentSq(SegA, SegB, SegC, SegD));
		}

		//!< �w��̕����Ɉ�ԉ����_�̃C�e���[�^��Ԃ� 
		[[nodiscard]] static auto Farthest(const std::vector<LinAlg::Vec3>& Pts, const LinAlg::Vec3& Dir) {
			return std::ranges::max_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return Dir.Dot(lhs) < Dir.Dot(rhs); 
				});
		}
		//!< �w��̕����Ɉ�ԋ߂��_�̃C�e���[�^��Ԃ� 
		[[nodiscard]] static auto Closest(const std::vector<LinAlg::Vec3>& Pts, const LinAlg::Vec3& Dir) {
			return std::ranges::min_element(Pts, 
				[&](const auto& lhs, const auto& rhs) {
					return Dir.Dot(lhs) < Dir.Dot(rhs);
				});
		}
		//!< ���C�����ԉ����_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Farthest(const std::vector<LinAlg::Vec3>& Pts, const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir) {
			return std::ranges::max_element(Pts,
				[&](const auto& lhs, const auto& rhs) { 
					return PointRaySq(lhs, RayPos, RayDir) < PointRaySq(rhs, RayPos, RayDir);
				});
		}
		//!< ���C�����ԋ߂��_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Closest(const std::vector<LinAlg::Vec3>& Pts, const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir) {
			return std::ranges::min_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return PointRaySq(lhs, RayPos, RayDir) < PointRaySq(rhs, RayPos, RayDir); 
				});
		}
		//!< �O�p�`�����ԉ����_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Farthest(const std::vector<LinAlg::Vec3>& Pts, const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C) {
			return std::ranges::max_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return PointTriangle(lhs, A, B, C) < PointTriangle(rhs, A, B, C); 
				});
		}
		//!< �O�p�`�����ԋ߂��_�̃C�e���[�^��Ԃ�
		[[nodiscard]] static auto Closest(const std::vector<LinAlg::Vec3>& Pts, const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C) {
			return std::ranges::min_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return PointTriangle(lhs, A, B, C) < PointTriangle(rhs, A, B, C);
				});
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
			const LinAlg::Vec3& VelA, const LinAlg::Vec3& VelB,
			float& T);

		//!< AABB vs ���C
		[[nodiscard]] bool AABBRay(const AABB& Ab,
			const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir,
			float& T);

		//!< AABB vs ����
		[[nodiscard]] static bool AABBSegment(const AABB& Ab,
			const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			float& T) {
			return AABBRay(Ab, SegA, (SegB - SegA), T) && T <= 1.0f; 
		}

		//!< AABB vs ��
		[[nodiscard]] static bool AABBSphere(const AABB& Ab,
			const LinAlg::Vec3& SpPos, const float SpRad) {
			return Distance::AABBPointSq(Ab, SpPos) <= std::powf(SpRad, 2.0f);
		}

		[[nodiscard]] bool CapsulePoint(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
			const LinAlg::Vec3& Pt);

		[[nodiscard]] bool CylinderPoint(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
			const LinAlg::Vec3& Pt);

		//!< ���C vs ��
		[[nodiscard]] std::optional<std::pair<float, float>> RaySphere(const LinAlg::Vec3& RayPos, const LinAlg::Vec3& RayDir, const LinAlg::Vec3& SpPos, const float SpRad);

		//!< �� vs ��
		[[nodiscard]] static bool SphereShpere(const float RadA, const float RadB, const LinAlg::Vec3& PosA, const LinAlg::Vec3& PosB) {
			return (PosB - PosA).LengthSq() <= std::powf(RadA + RadB, 2.0f); 
		}
		//!< �f���^���ԂŃR�[������ꍇ�́A���x���f���^���Ԃ̂��̂�n������
		[[nodiscard]] std::optional<float> SphereShpere(const float RadA, const float RadB,
			const LinAlg::Vec3& PosA, const LinAlg::Vec3& PosB,
			const LinAlg::Vec3& VelA, const LinAlg::Vec3& VelB);

		[[nodiscard]] bool SphereSphere(const Physics::RigidBody* RbA,
			const Physics::RigidBody* RbB,
			const float DeltaSec, ContactBase& Ct);

		[[nodiscard]] bool RigidBodyRigidBody(const Physics::RigidBody* RbA, 
			const Physics::RigidBody* RbB, 
			const float DeltaSec, Contact& Ct);
	}

	namespace Closest 
	{
		[[nodiscard]] static LinAlg::Vec3 AABBPoint(const AABB& Ab,
			const LinAlg::Vec3& Pt) {
			return LinAlg::Vec3(std::clamp(Pt.X(), Ab.Min.X(), Ab.Max.X()), std::clamp(Pt.Y(), Ab.Min.Y(), Ab.Max.Y()), std::clamp(Pt.Z(), Ab.Min.Z(), Ab.Max.Z()));
		}
		
		[[nodiscard]] LinAlg::Vec3 CapsulePoint(const LinAlg::Vec3& CapA, const LinAlg::Vec3& CapB, const float CapR,
			const LinAlg::Vec3& Pt);

		[[nodiscard]] LinAlg::Vec3 CylinderPoint(const LinAlg::Vec3& CyA, const LinAlg::Vec3& CyB, const float CyR,
			const LinAlg::Vec3& Pt);

		[[nodiscard]] static LinAlg::Vec3 OBBPoint(const LinAlg::Vec3& ObPos, const LinAlg::Vec3& ObX, const LinAlg::Vec3& ObY, const LinAlg::Vec3& ObZ, const LinAlg::Vec3& ObExt,
			const LinAlg::Vec3& Pt) {
			const auto D(Pt - ObPos);
			return ObPos + 
				std::clamp(D.Dot(ObX), -ObExt.X(), ObExt.X()) * ObX + 
				std::clamp(D.Dot(ObY), -ObExt.Y(), ObExt.Y()) * ObY + 
				std::clamp(D.Dot(ObZ), -ObExt.Z(), ObExt.Z()) * ObZ;
		}

		[[nodiscard]] static LinAlg::Vec3 PointSegment(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			float& T) {
			const auto AB = SegB - SegA;
			T = (Pt - SegA).Dot(AB);
			if (T <= 0.0f) {
				T = 0.0f;
				return SegA;
			}
			else {
				const auto Denom = AB.LengthSq();
				if (T >= Denom) {
					T = 1.0f;
					return SegB;
				}
				return SegA + AB * T / Denom;
			}
		}
		[[nodiscard]] static LinAlg::Vec3 PointSegment(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB) { float T; return PointSegment(Pt, SegA, SegB, T); }

		[[nodiscard]] static LinAlg::Vec3 PointSphere(const LinAlg::Vec3& Pt,
			const LinAlg::Vec3& SpPos, const float SpRad) {
			return SpPos + (Pt - SpPos).Normalize() * SpRad;
		}

		[[nodiscard]] static LinAlg::Vec3 PlanePoint(const LinAlg::Vec3& PlN, const float PlD,
			const LinAlg::Vec3& Pt);

		void SegmentSegment(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
			float& T0, float& T1,
			LinAlg::Vec3* OnAB = nullptr, LinAlg::Vec3* OnCD = nullptr);
		void SegmentSegment(const LinAlg::Vec3& SegA, const LinAlg::Vec3& SegB,
			const LinAlg::Vec3& SegC, const LinAlg::Vec3& SegD,
			LinAlg::Vec3& OnAB, LinAlg::Vec3& OnCD);
		
		static void SphereSphere(const LinAlg::Vec3& SpPosA, const float SpRadA,
			const LinAlg::Vec3& SpPosB, const float SpRadB,
			LinAlg::Vec3& OnA, LinAlg::Vec3& OnB) {
			const auto AB = (SpPosB - SpPosA).Normalize();
			OnA = AB * SpRadA + SpPosA;
			OnB = -AB * SpRadB + SpPosB;
		}
	}

	namespace Volume
	{
		[[nodiscard]] static float Tetrahedron(const LinAlg::Vec3& A, const LinAlg::Vec3& B, const LinAlg::Vec3& C, const LinAlg::Vec3& D) {
			return std::fabsf((D - A).Dot((D - B).Cross(D - C)) / 6.0f);
		}
	}

	//!< �Փˎ��̗͐ς̓K�p
	void ResolveContact(const Contact& Ct);
}