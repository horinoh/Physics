#pragma once

#include <numbers>
#include <span>

#include "PhysicsMath.h"
#include "Collision.h"

namespace Physics
{
	class Shape
	{
	public:
		virtual ~Shape() {}

		enum class SHAPE_TYPE
		{
			SPHERE,
			BOX,
			CONVEX,
		};
		[[nodiscard]] virtual SHAPE_TYPE GetShapeType() const = 0;

		[[nodiscard]] virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const = 0;
		[[nodiscard]] inline Collision::AABB GetAABB(std::span<const Math::Vec3> Vertices, const Math::Vec3& Pos, const Math::Quat& Rot) const {
			Collision::AABB Ab;
			for (auto& i : Vertices) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}
			return Ab;
		}

		//!< 指定の方向 (UDir : 正規化されていること) に一番遠い点を返す
		[[nodiscard]] virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const = 0;
		[[nodiscard]] inline Math::Vec3 GetSupportPoint(std::span<const Math::Vec3> Vertices, const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const {
			std::vector<Math::Vec3> Pts;
			Pts.reserve(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(Pts),
				[&](const auto& rhs) {
					return Rot.Rotate(rhs) + Pos;
				});
			return *std::ranges::max_element(Pts,
				[&](const auto& lhs, const auto& rhs) {
					return UDir.Dot(lhs) < UDir.Dot(rhs);
				}) + UDir * Bias;
		}

		//!< (回転により) 指定の方向に最も速く動いている頂点の速度を返す
		[[nodiscard]] virtual float GetFastestPointSpeed(const Math::Vec3& AngVel, const Math::Vec3& Dir) const { return 0.0f; }
		[[nodiscard]] inline float GetFastestPointSpeed(std::span<const Math::Vec3> Vertices, const Math::Vec3& AngVel, const Math::Vec3& Dir) const {
			std::vector<float> Speeds;
			Speeds.reserve(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(Speeds),
				[&](const auto& rhs) {
					return Dir.Dot(AngVel.Cross(rhs - GetCenterOfMass()));
				});
			return std::ranges::max(Speeds);
		}

		[[nodiscard]] const Math::Vec3& GetCenterOfMass() const { return CenterOfMass; }
		[[nodiscard]] const Math::Mat3& GetInertiaTensor() const { return InertiaTensor; }
		[[nodiscard]] const Math::Mat3& GetInvInertiaTensor() const { return InvInertiaTensor; }

	public:
		Math::Vec3 CenterOfMass = Math::Vec3::Zero();
		Math::Mat3 InertiaTensor = Math::Mat3::Identity();
		Math::Mat3 InvInertiaTensor = Math::Mat3::Identity();
	};

	class ShapeSphere : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeSphere() {}
		ShapeSphere(const float R) : Radius(R) { Init(); }
		virtual ~ShapeSphere() {}

		ShapeSphere& Init() {
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return *this;
		}

		virtual SHAPE_TYPE GetShapeType() const override { return SHAPE_TYPE::SPHERE; }

		//!< 球の慣性テンソル 2 / 5  * (R^2,   0,   0)
		//!<						 (  0, R^2,   0)
		//!<						 (  0,   0, R^2)
		Math::Mat3 CalcInertiaTensor() const {
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

	class ShapeBox : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeBox() {}
		ShapeBox(const float Ext) {
			const auto R = Ext * 0.5f;
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
			Init();
		}
		ShapeBox(const Math::Vec3& Ext) {
			const auto WR = Ext.X() * 0.5f;
			const auto HR = Ext.Y() * 0.5f;
			const auto DR = Ext.Z() * 0.5f;
			Vertices = {
				Math::Vec3(WR, HR, DR),
				Math::Vec3(WR, HR, -DR),
				Math::Vec3(WR, -HR, DR),
				Math::Vec3(WR, -HR, -DR),
				Math::Vec3(-WR, HR, DR),
				Math::Vec3(-WR, HR, -DR),
				Math::Vec3(-WR, -HR, DR),
				Math::Vec3(-WR, -HR, -DR)
			};
			Init();
		}
		virtual ~ShapeBox() {}

		ShapeBox& Init() {
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return *this;
		}

		virtual SHAPE_TYPE GetShapeType() const override { return SHAPE_TYPE::BOX; }

		Math::Vec3 CalcExtent() const {
			std::vector<float> Xs, Ys, Zs;
			Xs.reserve(std::size(Vertices));
			Ys.reserve(std::size(Vertices));
			Zs.reserve(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(Xs), [](const auto& rhs) { return rhs.X(); });
			std::ranges::transform(Vertices, std::back_inserter(Ys), [](const auto& rhs) { return rhs.Y(); });
			std::ranges::transform(Vertices, std::back_inserter(Zs), [](const auto& rhs) { return rhs.Z(); });
			return Math::Vec3(std::ranges::max(Xs) - std::ranges::min(Xs), 
				std::ranges::max(Ys) - std::ranges::min(Ys), 
				std::ranges::max(Zs) - std::ranges::min(Zs));
		}
		//!< ボックスの慣性テンソル 1 / 12 * (H^2+D^2,       0,       0)
		//!<							  (      0, W^2+D^2,       0)
		//!<							  (      0,       0, W^2+H^2)
		Math::Mat3 CalcInertiaTensor() const {
			const auto Ext = CalcExtent();
			const auto W2 = Ext.X() * Ext.X();
			const auto H2 = Ext.Y() * Ext.Y();
			const auto D2 = Ext.Z() * Ext.Z();
			constexpr auto Div = 1.0f / 12.0f;
			return {
				{ (H2 + D2) * Div, 0.0f, 0.0f },
				{ 0.0f, (W2 + D2) * Div, 0.0f },
				{ 0.0f, 0.0f, (W2 + H2) * Div }
			};
		}

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const override {
			return Super::GetAABB(Vertices, Pos, Rot);
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
			return Super::GetSupportPoint(Vertices, Pos, Rot, UDir, Bias);
		}
		virtual float GetFastestPointSpeed(const Math::Vec3& AngVel, const Math::Vec3& Dir) const override {
			return Super::GetFastestPointSpeed(Vertices, AngVel, Dir);
		}

	public:
		std::array<Math::Vec3, 8> Vertices;
	};

	class ShapeConvex : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeConvex() {}
		ShapeConvex(const std::vector<Math::Vec3>& Mesh) { Init(Mesh); }
		virtual ~ShapeConvex() {}

		virtual ShapeConvex& Init(const std::vector<Math::Vec3>& Mesh);

		virtual SHAPE_TYPE GetShapeType() const override { return SHAPE_TYPE::CONVEX; }

		virtual Collision::AABB GetAABB(const Math::Vec3& Pos, const Math::Quat& Rot) const override {
			return Super::GetAABB(Vertices, Pos, Rot);
		}

		virtual Math::Vec3 GetSupportPoint(const Math::Vec3& Pos, const Math::Quat& Rot, const Math::Vec3& UDir, const float Bias) const override {
			return Super::GetSupportPoint(Vertices, Pos, Rot, UDir, Bias);
		}
		virtual float GetFastestPointSpeed(const Math::Vec3& AngVel, const Math::Vec3& Dir) const override {
			return Super::GetFastestPointSpeed(Vertices, AngVel, Dir);
		}

	public:
		std::vector<Math::Vec3> Vertices;

		//!< 描画しない場合は不要、ここでは一応持たせておく
		std::vector<Collision::TriInds> Indices;
	};
	void CreateVertices_Diamond(std::vector<Math::Vec3>& Dst);
}

