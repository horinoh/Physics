#pragma once

#include <numbers>

#include "PhysicsMath.h"

namespace Collision {
	class AABB
	{
	public:
		AABB() { Clear(); }
		AABB(const Math::Vec3& minVal, const Math::Vec3& maxVal) : Min(minVal), Max(maxVal) {}
		AABB(const AABB& rhs) : Min(rhs.Min), Max(rhs.Max) {}
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
		AABB& Expand(const AABB& rhs) {
			Expand(rhs.Min);
			Expand(rhs.Max);
			return *this;
		}

		[[nodiscard]] bool Intersect(const AABB& rhs) const {
			if (Max.X() < rhs.Min.X() || rhs.Max.X() < Min.X() ||
				Max.Y() < rhs.Min.Y() || rhs.Max.Y() < Min.Y() ||
				Max.Z() < rhs.Min.Z() || rhs.Max.Z() < Min.Z()) {
				return false;
			}
			return true;
		}

		[[nodiscard]] Math::Vec3 GetCenter() const { return (Min + Max) * 0.5f; }
		[[nodiscard]] Math::Vec3 GetExtent() const { return Max - Min; }

		Math::Vec3 Min = Math::Vec3::Max();
		Math::Vec3 Max = Math::Vec3::Min();
	};
}

namespace Physics
{
	class Shape
	{
	public:
		virtual ~Shape() {}
		virtual void Init() {
			CenterOfMass = CalcCenterOfMass();
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
		}

		enum class SHAPE
		{
			SPHERE,
			BOX,
			CONVEX,
		};
		[[nodiscard]] virtual SHAPE GetShapeTyoe() const = 0;

		[[nodiscard]] virtual Math::Vec3 CalcCenterOfMass() const = 0;
		[[nodiscard]] virtual Math::Mat3 CalcInertiaTensor() const = 0;
		[[nodiscard]] virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const = 0;

#pragma region GJK
		//!< 指定の方向 (NDir : 正規化されていること) に一番遠い点を返す
		[[nodiscard]] virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& NDir, const float Bias) const = 0;
		//!< 指定の方向に最も速く動いている頂点の速度を返す (長いものは回転により衝突の可能性がある (速度が無くても))
		[[nodiscard]] virtual float GetFastestPointSpeed(const Math::Vec3& AngVel, const Math::Vec3& Dir) const { return 0.0f; }
#pragma endregion

		[[nodiscard]] const Math::Vec3& GetCenterOfMass() const { return CenterOfMass; }
		[[nodiscard]] const Math::Mat3& GetInertiaTensor() const { return InertiaTensor; }
		[[nodiscard]] const Math::Mat3& GetInvInertiaTensor() const { return InvInertiaTensor; }

	public:
		Math::Vec3 CenterOfMass = Math::Vec3::Zero();
		Math::Mat3 InertiaTensor = Math::Mat3::Identity();
		Math::Mat3 InvInertiaTensor = Math::Mat3::Identity();
		//AABB Aabb;
	};

	//!< 検証済み
	class ShapeSphere : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeSphere() {}
		ShapeSphere(const float R) : Radius(R) {}
		virtual ~ShapeSphere() {}

		virtual SHAPE GetShapeTyoe() const override { return SHAPE::SPHERE; }

		virtual Math::Vec3 CalcCenterOfMass() const override { return Math::Vec3::Zero(); }
		virtual Math::Mat3 CalcInertiaTensor() const override {
			//!< 球の慣性テンソル R^2 * 2 / 5  * Identity
			return Radius * Radius * 2.0f / 5.0f * Math::Mat3::Identity();
		}

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, [[maybe_unused]] const Math::Quat& Rot) const override {
			return { Pos - Math::Vec3(Radius), Pos + Math::Vec3(Radius) };
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
			return Pos + UDir * (Radius + Bias);
		}

	public:
		float Radius = 1.0f;
	};

	//!< #TODO 未検証
	class ShapeBox : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeBox() {}
		ShapeBox(const float R) : Extent(Math::Vec3(R)) {}
		virtual ~ShapeBox() {}

		virtual SHAPE GetShapeTyoe() const override { return SHAPE::BOX; }
		
		virtual Math::Vec3 CalcCenterOfMass() const override { return Math::Vec3::Zero(); }
		virtual Math::Mat3 CalcInertiaTensor() const override {
			//!< ボックスの慣性テンソル 1 / 12 * (H^2+D^2,       0,       0)
			//!<							  (      0, w^2+D^2,       0)
			//!<							  (      0,       0, W^2+H^2)
			const auto W2 = Extent.X() * Extent.X(), H2 = Extent.Y() * Extent.Y(), D2 = Extent.Z() * Extent.Z();
			return {
				{ (H2 + D2) / 12.0f, 0.0f, 0.0f },
				{ 0.0f, (W2 + D2) / 12.0f, 0.0f },
				{ 0.0f, 0.0f, (W2 + H2) / 12.0f }
			};
		}

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const override {
			const std::array Points = {
				Extent,
				Math::Vec3(Extent.X(), Extent.Y(), -Extent.Z()),
				Math::Vec3(Extent.X(), -Extent.Y(), Extent.Z()),
				Math::Vec3(Extent.X(), -Extent.Y(), -Extent.Z()),
				Math::Vec3(-Extent.X(), Extent.Y(), Extent.Z()),
				Math::Vec3(-Extent.X(), Extent.Y(), -Extent.Z()),
				Math::Vec3(-Extent.X(), -Extent.Y(), Extent.Z()),
				-Extent
			};

			Collision::AABB Ab;
			for (auto& i : Points) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}

			return Ab;
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
			const std::array Points = {
				Extent,
				Math::Vec3(Extent.X(), Extent.Y(), -Extent.Z()),
				Math::Vec3(Extent.X(), -Extent.Y(), Extent.Z()),
				Math::Vec3(Extent.X(), -Extent.Y(), -Extent.Z()),
				Math::Vec3(-Extent.X(), Extent.Y(), Extent.Z()),
				Math::Vec3(-Extent.X(), Extent.Y(), -Extent.Z()),
				Math::Vec3(-Extent.X(), -Extent.Y(), Extent.Z()),
				-Extent
			};
			Math::Vec3 MaxPt = Rot.Rotate(Points[0]) + Pos;
			auto MaxDist = UDir.Dot(MaxPt);
			for (auto i = 1; i < std::size(Points); ++i) {
				const auto Pt = Rot.Rotate(Points[i]) + Pos;
				const auto Dist = UDir.Dot(Pt);
				if (Dist > MaxDist) {
					MaxDist = Dist;
					MaxPt = Pt;
				}
			}
			return MaxPt + UDir * Bias;
		}
		virtual float GetFastestPointSpeed(const Math::Vec3& AngVel, const Math::Vec3& Dir) const override {
			const std::array Points = {
				Extent,
				Math::Vec3(Extent.X(), Extent.Y(), -Extent.Z()),
				Math::Vec3(Extent.X(), -Extent.Y(), Extent.Z()),
				Math::Vec3(Extent.X(), -Extent.Y(), -Extent.Z()),
				Math::Vec3(-Extent.X(), Extent.Y(), Extent.Z()),
				Math::Vec3(-Extent.X(), Extent.Y(), -Extent.Z()),
				Math::Vec3(-Extent.X(), -Extent.Y(), Extent.Z()),
				-Extent
			};
			auto MaxSpeed = 0.0f;
			for (auto& i : Points) {
				const auto Speed = Dir.Dot(AngVel.Cross(i - GetCenterOfMass()));
				if (Speed > MaxSpeed) {
					MaxSpeed = Speed;
				}
			}
			return MaxSpeed;
		}

	public:
		Math::Vec3 Extent = Math::Vec3::One();
	};

	//!< #TODO
	class ShapeConvex : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeConvex() {}
		virtual ~ShapeConvex() {}
		virtual void Init(const std::vector<Math::Vec3>& Vert) {
			Super::Init();
			//Convex::BuildConvexHull(Vert, Vertices, Indices);

			//CenterOfMass = ;
		}

		virtual SHAPE GetShapeTyoe() const override { return SHAPE::CONVEX; }

		virtual Math::Vec3 CalcCenterOfMass() const {
			//!< #TODO
			return Math::Vec3::Zero();
		}
		virtual Math::Mat3 CalcInertiaTensor() const override {
			//!< #TODO
			return Math::Mat3::Identity();
		}

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const override {
			Collision::AABB Ab;
			for (auto& i : Vertices) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}
			return Ab;
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& NDir, const float Bias) const override {
#if 0
			return *std::ranges::max_element(Points, [&](const auto lhs, const auto rhs) { 
				return NDir.Dot(Rot.Rotate(lhs) + Pos) < NDir.Dot(Rot.Rotate(rhs) + Pos); 
			}) + NDir * Bias;
#else
			Math::Vec3 MaxPt = Rot.Rotate(Vertices[0]) + Pos;
			auto MaxDist = NDir.Dot(MaxPt);
			for (auto i = 1; i < std::size(Vertices); ++i) {
				const auto Pt = Rot.Rotate(Vertices[i]) + Pos;
				const auto Dist = NDir.Dot(Pt);
				if (Dist > MaxDist) {
					MaxDist = Dist;
					MaxPt = Pt;
				}
			}
			return MaxPt + NDir * Bias;
#endif
		}
		virtual float GetFastestPointSpeed(const Math::Vec3& AngVel, const Math::Vec3& Dir) const override {
			auto MaxSpeed = 0.0f;
			for (auto& i : Vertices) {
				const auto Speed = Dir.Dot(AngVel.Cross(i - GetCenterOfMass()));
				if (Speed > MaxSpeed) {
					MaxSpeed = Speed;
				}
			}
			return MaxSpeed;
		}

	public:
		std::vector<Math::Vec3> Vertices;

		//!< インデックスは描画しない場合不要だが、一応持っておく
		//std::vector<Collition::TriInds> Indices;
	};

	static void CreateVertices_Diamond(std::vector<Math::Vec3>& Dst);
}

