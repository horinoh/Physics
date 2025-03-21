#pragma once

#include <numbers>

#include "PhysicsMath.h"
#include "Collision.h"

namespace Physics
{
	class Shape
	{
	public:
		virtual ~Shape() {}

		enum class SHAPE
		{
			SPHERE,
			BOX,
			CONVEX,
		};
		[[nodiscard]] virtual SHAPE GetShapeType() const = 0;

		[[nodiscard]] virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const = 0;

#pragma region GJK
		//!< 指定の方向 (UDir : 正規化されていること) に一番遠い点を返す
		[[nodiscard]] virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const = 0;
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

		ShapeSphere& Init() {
			//CenterOfMass = Math::Vec3::Zero();
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return *this;
		}

		virtual SHAPE GetShapeType() const override { return SHAPE::SPHERE; }

		//!< 球の慣性テンソル R^2 * 2 / 5  * Identity
		Math::Mat3 CalcInertiaTensor() const { return Radius * Radius * 2.0f / 5.0f * Math::Mat3::Identity(); }

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, [[maybe_unused]] const Math::Quat& Rot) const override {
			return { Pos - Math::Vec3(Radius), Pos + Math::Vec3(Radius) };
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
			return Pos + UDir * (Radius + Bias);
		}

	public:
		float Radius = 1.0f;
	};

	class ShapeBox : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeBox() {}
		ShapeBox(const float R) {
			Vertices = {
				Math::Vec3(R),
				Math::Vec3(R, R, -R),
				Math::Vec3(R, -R, R),
				Math::Vec3(R, -R, -R),
				Math::Vec3(-R, R, R),
				Math::Vec3(-R, R, -R),
				Math::Vec3(-R, -R, R),
				Math::Vec3(-R)
			};
		}
		virtual ~ShapeBox() {}

		ShapeBox& Init() {
			//CenterOfMass = Math::Vec3::Zero();
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return *this;
		}

		virtual SHAPE GetShapeType() const override { return SHAPE::BOX; }

		//!< ボックスの慣性テンソル 1 / 12 * (H^2+D^2,       0,       0)
		//!<							  (      0, w^2+D^2,       0)
		//!<							  (      0,       0, W^2+H^2)
		Math::Mat3 CalcInertiaTensor() const {
			const auto& V = Vertices[0];
			const auto W2 = V.X() * V.X(), H2 = V.Y() * V.Y(), D2 = V.Z() * V.Z();
			return {
				{ (H2 + D2) / 12.0f, 0.0f, 0.0f },
				{ 0.0f, (W2 + D2) / 12.0f, 0.0f },
				{ 0.0f, 0.0f, (W2 + H2) / 12.0f }
			};
		}

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const override {
			Collision::AABB Ab;
			for (auto& i : Vertices) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}
			return Ab;
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
			Math::Vec3 MaxPt = Rot.Rotate(Vertices[0]) + Pos;
			auto MaxDist = UDir.Dot(MaxPt);
			for (auto i = 1; i < std::size(Vertices); ++i) {
				const auto Pt = Rot.Rotate(Vertices[i]) + Pos;
				const auto Dist = UDir.Dot(Pt);
				if (Dist > MaxDist) {
					MaxDist = Dist;
					MaxPt = Pt;
				}
			}
			return MaxPt + UDir * Bias;
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
		std::array<Math::Vec3, 8> Vertices = {
			Math::Vec3(0.5f),
			Math::Vec3(0.5f, 0.5f, -0.5f),
			Math::Vec3(0.5f, -0.5f, 0.5f),
			Math::Vec3(0.5f, -0.5f, -0.5f),
			Math::Vec3(-0.5f, 0.5f, 0.5f),
			Math::Vec3(-0.5f, 0.5f, -0.5f),
			Math::Vec3(-0.5f, -0.5f, 0.5f),
			Math::Vec3(-0.5f),
		};
	};

	class ShapeConvex : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeConvex() {}
		virtual ~ShapeConvex() {}

		virtual ShapeConvex& Init(const std::vector<Math::Vec3>& Vert);

		virtual SHAPE GetShapeType() const override { return SHAPE::CONVEX; }

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const override {
			Collision::AABB Ab;
			for (auto& i : Vertices) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}
			return Ab;
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
#if 0
			return *std::ranges::max_element(Points, [&](const auto lhs, const auto rhs) {
				return UDir.Dot(Rot.Rotate(lhs) + Pos) < UDir.Dot(Rot.Rotate(rhs) + Pos);
				}) + UDir * Bias;
#else
			Math::Vec3 MaxPt = Rot.Rotate(Vertices[0]) + Pos;
			auto MaxDist = UDir.Dot(MaxPt);
			for (auto i = 1; i < std::size(Vertices); ++i) {
				const auto Pt = Rot.Rotate(Vertices[i]) + Pos;
				const auto Dist = UDir.Dot(Pt);
				if (Dist > MaxDist) {
					MaxDist = Dist;
					MaxPt = Pt;
				}
			}
			return MaxPt + UDir * Bias;
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
		std::vector<Collision::TriInds> Indices;
	};

	void CreateVertices_Box(std::vector<Math::Vec3>& Dst, const float W = 0.5f, const float H = 0.5f, const float D = 0.5f);
	void CreateVertices_Diamond(std::vector<Math::Vec3>& Dst);
}

