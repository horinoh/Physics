#pragma once

#include <algorithm>
#include <optional>

#include "PhysicsMath.h"

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
		AABB(const Math::Vec3& MinVal, const Math::Vec3& MaxVal) : Min(MinVal), Max(MaxVal) {}
		AABB(const AABB& rhs) : Min(rhs.Min), Max(rhs.Max) {}
		AABB(const std::vector<Math::Vec3>& Vertices) { Expand(Vertices); }
		AABB& operator=(const AABB& rhs) { 
			Min = rhs.Min; 
			Max = rhs.Max; 
			return *this; 
		}

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
		AABB& Expand(const std::vector<Math::Vec3>& Vertices) { 
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

#define NRMB
	struct ContactBase
	{
		float TimeOfImpact = 0.0f;

		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		//!< ワールドスペース
		Math::Vec3 WPointA;
		Math::Vec3 WPointB;

#ifdef NRMB
		//!< ワールドスペース B -> A
#else
		//!< ワールドスペース A -> B
#endif
		Math::Vec3 WNormal;

		ContactBase& Swap() {
			std::swap(RigidBodyA, RigidBodyB);
			WNormal = -WNormal;
			return *this;
		}
	};
	struct Contact : ContactBase
	{
		//!< ローカルスペース (ConstraintPenetration で使用、衝突時のローカル位置を覚えおいて新しいトランスフォームで変換する)
		Math::Vec3 LPointA;
		Math::Vec3 LPointB;

		void CalcLocal();

		Contact& Swap() {
			ContactBase::Swap();
			std::swap(LPointA, LPointB);
			return *this;
		}
	};

	namespace Distance 
	{
		[[nodiscard]] static float AABBPointSq(const AABB& Ab, const Math::Vec3& Pt) {
			return Math::Vec3(std::max(std::max(Ab.Min.X() - Pt.X(), 0.0f), std::max(Pt.X() - Ab.Max.X(), 0.0f)),
				std::max(std::max(Ab.Min.Y() - Pt.Y(), 0.0f), std::max(Pt.Y() - Ab.Max.Y(), 0.0f)),
				std::max(std::max(Ab.Min.Z() - Pt.Z(), 0.0f), std::max(Pt.Z() - Ab.Max.Z(), 0.0f))).LengthSq();
		}
		[[nodiscard]] static float AABBPoint(const AABB& Ab, const Math::Vec3& Pt) {
			return  std::sqrtf(AABBPoint(Ab, Pt));
		}

		[[nodiscard]] float CapsulePointSq(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
			const Math::Vec3& Pt);
		[[nodiscard]] static float CapsulePoint(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
			const Math::Vec3& Pt) {
			return std::sqrtf(CapsulePointSq(CapA, CapB, CapR, Pt));
		}

		[[nodiscard]] float CylinderPointSq(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
			const Math::Vec3& Pt);
		[[nodiscard]] static float CylinderPoint(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
			const Math::Vec3& Pt) {
			return std::sqrtf(CylinderPointSq(CyA, CyB, CyR, Pt));
		}

		[[nodiscard]] static float PointRaySq(const Math::Vec3& Pt, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			const auto ToPt = Pt - RayPos;
			return ((RayDir * ToPt.Dot(RayDir)) - ToPt).LengthSq();
		}
		[[nodiscard]] static float PointRay(const Math::Vec3& Pt, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return std::sqrtf(PointRaySq(Pt, RayPos, RayDir));
		}

		[[nodiscard]] static float PointSegmentSq(const Math::Vec3& Pt,
			const Math::Vec3& SegA, const Math::Vec3& SegB) {
			const auto AB = SegB - SegA;
			const auto AP = Pt - SegA;
			const auto T = AP.Dot(AB);
			if (T <= 0.0f) { return AP.LengthSq(); }
			const auto T1 = AB.LengthSq();
			if (T1 <= T) { return (Pt - SegB).LengthSq(); }
			return AP.LengthSq() - std::pow(T, 2.0f) / T1;
		}
		[[nodiscard]] static float PointSegment(const Math::Vec3& Pt,
			const Math::Vec3& SegA, const Math::Vec3& SegB) {
			return std::sqrtf(PointSegmentSq(Pt, SegA, SegB));
		}

		[[nodiscard]] static float PointSphereSq(const Math::Vec3& Pt,
			const Math::Vec3& SpPos, const float SpRad) {
			return std::max((Pt - SpPos).LengthSq() - std::pow(SpRad, 2.0f), 0.0f);
		}
		[[nodiscard]] static float PointSphere(const Math::Vec3& Pt,
			const Math::Vec3& SpPos, const float SpRad) {
			return std::sqrtf(PointSphereSq(Pt, SpPos, SpRad));
		}

		[[nodiscard]] static float PointTriangle(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			return (Pt - A).Dot(Math::Vec3::UnitNormal(A, B, C));
		}
		[[nodiscard]] static bool IsFront(const Math::Vec3& Pt, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			//!< 側だけ分かればよいので正規化する必要はない
			return (Pt - A).Dot(Math::Vec3::Normal(A, B, C)) >= 0.0f;
		}

		[[nodiscard]] static float PlanePoint(const Math::Vec3& PlN, const float PlD,
			const Math::Vec3 Pt) {
			return PlN.Dot(Pt) + PlD;
		}

		[[nodiscard]] float SegmentSegmentSq(const Math::Vec3& SegA, const Math::Vec3& SegB,
			const Math::Vec3& SegC, const Math::Vec3& SegD,
			float& T0, float& T1);
		[[nodiscard]] float SegmentSegmentSq(const Math::Vec3& SegA, const Math::Vec3& SegB,
			const Math::Vec3& SegC, const Math::Vec3& SegD);
		[[nodiscard]] static float SegmentSegment(const Math::Vec3& SegA, const Math::Vec3& SegB,
			const Math::Vec3& SegC, const Math::Vec3& SegD,
			float& T0, float& T1) {
			return std::sqrtf(SegmentSegmentSq(SegA, SegB, SegC, SegD, T0, T1));
		}
		[[nodiscard]] static float SegmentSegment(const Math::Vec3& SegA, const Math::Vec3& SegB,
			const Math::Vec3& SegC, const Math::Vec3& SegD) {
			return std::sqrtf(SegmentSegmentSq(SegA, SegB, SegC, SegD));
		}

		//!< 指定の方向に一番遠い点のイテレータを返す 
		[[nodiscard]] static auto Farthest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& Dir) {
			return std::ranges::max_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return Dir.Dot(lhs) < Dir.Dot(rhs); 
				});
		}
		//!< 指定の方向に一番近い点のイテレータを返す 
		[[nodiscard]] static auto Closest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& Dir) {
			return std::ranges::min_element(Pts, 
				[&](const auto& lhs, const auto& rhs) {
					return Dir.Dot(lhs) < Dir.Dot(rhs);
				});
		}
		//!< レイから一番遠い点のイテレータを返す
		[[nodiscard]] static auto Farthest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return std::ranges::max_element(Pts,
				[&](const auto& lhs, const auto& rhs) { 
					return PointRaySq(lhs, RayPos, RayDir) < PointRaySq(rhs, RayPos, RayDir);
				});
		}
		//!< レイから一番近い点のイテレータを返す
		[[nodiscard]] static auto Closest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& RayPos, const Math::Vec3& RayDir) {
			return std::ranges::min_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return PointRaySq(lhs, RayPos, RayDir) < PointRaySq(rhs, RayPos, RayDir); 
				});
		}
		//!< 三角形から一番遠い点のイテレータを返す
		[[nodiscard]] static auto Farthest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
			return std::ranges::max_element(Pts, 
				[&](const auto& lhs, const auto& rhs) { 
					return PointTriangle(lhs, A, B, C) < PointTriangle(rhs, A, B, C); 
				});
		}
		//!< 三角形から一番近い点のイテレータを返す
		[[nodiscard]] static auto Closest(const std::vector<Math::Vec3>& Pts, const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C) {
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
			const Math::Vec3& VelA, const Math::Vec3& VelB,
			float& T);

		//!< AABB vs レイ
		[[nodiscard]] bool AABBRay(const AABB& Ab,
			const Math::Vec3& RayPos, const Math::Vec3& RayDir,
			float& T);

		//!< AABB vs 線分
		[[nodiscard]] static bool AABBSegment(const AABB& Ab,
			const Math::Vec3& SegA, const Math::Vec3& SegB,
			float& T) {
			return AABBRay(Ab, SegA, (SegB - SegA), T) && T <= 1.0f; 
		}

		//!< AABB vs 球
		[[nodiscard]] static bool AABBSphere(const AABB& Ab,
			const Math::Vec3& SpPos, const float SpRad) {
			return Distance::AABBPointSq(Ab, SpPos) <= std::powf(SpRad, 2.0f);
		}

		[[nodiscard]] bool CapsulePoint(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
			const Math::Vec3& Pt);

		[[nodiscard]] bool CylinderPoint(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
			const Math::Vec3& Pt);

		//!< レイ vs 球
		[[nodiscard]] std::optional<std::pair<float, float>> RaySphere(const Math::Vec3& RayPos, const Math::Vec3& RayDir, const Math::Vec3& SpPos, const float SpRad);

		//!< 球 vs 球
		[[nodiscard]] static bool SphereShpere(const float RadA, const float RadB, const Math::Vec3& PosA, const Math::Vec3& PosB) {
			return (PosB - PosA).LengthSq() <= std::powf(RadA + RadB, 2.0f); 
		}
		//!< デルタ時間でコールする場合は、速度もデルタ時間のものを渡すこと
		[[nodiscard]] std::optional<float> SphereShpere(const float RadA, const float RadB,
			const Math::Vec3& PosA, const Math::Vec3& PosB,
			const Math::Vec3& VelA, const Math::Vec3& VelB);

		[[nodiscard]] bool SphereSphere(const Physics::RigidBody* RbA,
			const Physics::RigidBody* RbB,
			const float DeltaSec, ContactBase& Ct);

		[[nodiscard]] bool RigidBodyRigidBody(const Physics::RigidBody* RbA, 
			const Physics::RigidBody* RbB, 
			const float DeltaSec, Contact& Ct);
	}

	namespace Closest 
	{
		[[nodiscard]] static Math::Vec3 AABBPoint(const AABB& Ab,
			const Math::Vec3& Pt) {
			return Math::Vec3(std::clamp(Pt.X(), Ab.Min.X(), Ab.Max.X()), std::clamp(Pt.Y(), Ab.Min.Y(), Ab.Max.Y()), std::clamp(Pt.Z(), Ab.Min.Z(), Ab.Max.Z()));
		}
		
		[[nodiscard]] Math::Vec3 CapsulePoint(const Math::Vec3& CapA, const Math::Vec3& CapB, const float CapR,
			const Math::Vec3& Pt);

		[[nodiscard]] Math::Vec3 CylinderPoint(const Math::Vec3& CyA, const Math::Vec3& CyB, const float CyR,
			const Math::Vec3& Pt);

		[[nodiscard]] static Math::Vec3 OBBPoint(const Math::Vec3& ObPos, const Math::Vec3& ObX, const Math::Vec3& ObY, const Math::Vec3& ObZ, const Math::Vec3& ObExt,
			const Math::Vec3& Pt) {
			const auto D(Pt - ObPos);
			return ObPos + 
				std::clamp(D.Dot(ObX), -ObExt.X(), ObExt.X()) * ObX + 
				std::clamp(D.Dot(ObY), -ObExt.Y(), ObExt.Y()) * ObY + 
				std::clamp(D.Dot(ObZ), -ObExt.Z(), ObExt.Z()) * ObZ;
		}

		[[nodiscard]] static Math::Vec3 PointSegment(const Math::Vec3& Pt,
			const Math::Vec3& SegA, const Math::Vec3& SegB,
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
		[[nodiscard]] static Math::Vec3 PointSegment(const Math::Vec3& Pt,
			const Math::Vec3& SegA, const Math::Vec3& SegB) { float T; return PointSegment(Pt, SegA, SegB, T); }

		[[nodiscard]] static Math::Vec3 PointSphere(const Math::Vec3& Pt,
			const Math::Vec3& SpPos, const float SpRad) {
			return SpPos + (Pt - SpPos).Normalize() * SpRad;
		}

		[[nodiscard]] static Math::Vec3 PlanePoint(const Math::Vec3& PlN, const float PlD,
			const Math::Vec3& Pt);

		void SegmentSegment(const Math::Vec3& SegA, const Math::Vec3& SegB,
			const Math::Vec3& SegC, const Math::Vec3& SegD,
			float& T0, float& T1,
			Math::Vec3* OnAB = nullptr, Math::Vec3* OnCD = nullptr);
		void SegmentSegment(const Math::Vec3& SegA, const Math::Vec3& SegB,
			const Math::Vec3& SegC, const Math::Vec3& SegD,
			Math::Vec3& OnAB, Math::Vec3& OnCD);
		
		static void SphereSphere(const Math::Vec3& SpPosA, const float SpRadA,
			const Math::Vec3& SpPosB, const float SpRadB,
			Math::Vec3& OnA, Math::Vec3& OnB) {
			const auto AB = (SpPosB - SpPosA).Normalize();
			OnA = AB * SpRadA + SpPosA;
			OnB = -AB * SpRadB + SpPosB;
		}
	}

	namespace Volume
	{
		[[nodiscard]] static float Tetrahedron(const Math::Vec3& A, const Math::Vec3& B, const Math::Vec3& C, const Math::Vec3& D) {
			return std::fabsf((D - A).Dot((D - B).Cross(D - C)) / 6.0f);
		}
	}

	//!< 衝突時の力積の適用
	void ResolveContact(const Contact& Ct);
}