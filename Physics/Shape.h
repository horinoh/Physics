#pragma once

#include "Math.h"
#include "Bound.h"

using namespace Math;
using namespace Collision;

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
		[[nodiscard]] virtual SHAPE GetShapeTyoe() const = 0;

		[[nodiscard]] virtual Vec3 GetCenterOfMass() const { return CenterOfMass; }
		[[nodiscard]] virtual Mat3 GetInertiaTensor() const = 0;

		[[nodiscard]] virtual AABB GetAABB(const Vec3& Pos, const Quat& Rot) const = 0;

#pragma region GJK
		//!< 指定の方向 (NDir : 正規化されていること) に一番遠い点を返す
		[[nodiscard]] virtual Vec3 GetSupportPoint(const Vec3& Pos, const Quat& Rot, const Vec3& NDir, const float Bias) const = 0;
		//!< 指定の方向に最も速く動いている頂点の速度を返す (長いものは回転により衝突の可能性がある (速度が無くても))
		[[nodiscard]] virtual float GetFastestPointSpeed(const Vec3& AngVel, const Vec3& Dir) const { return 0.0f; }
#pragma endregion

	public:
		Vec3 CenterOfMass = Vec3::Zero();
		AABB Aabb;
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

		virtual Mat3 GetInertiaTensor() const override { 
			//!< 球の慣性テンソル R^2 * 2 / 5  * Identity
			return Radius * Radius * 2.0f / 5.0f * Mat3::Identity();
		}

		virtual AABB GetAABB(const Vec3& Pos, [[maybe_unused]] const Quat& Rot) const override {
			return { Pos - Vec3(Radius), Pos + Vec3(Radius) };
		}

		virtual Vec3 GetSupportPoint(const Vec3& Pos, const Quat& Rot, const Vec3& NDir, const float Bias) const override {
			return Pos + NDir * (Radius + Bias);
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
		virtual ~ShapeBox() {}

		virtual SHAPE GetShapeTyoe() const override { return SHAPE::BOX; }

		virtual Mat3 GetInertiaTensor() const override {
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

		virtual AABB GetAABB(const Vec3& Pos, const Quat& Rot) const override {
			const std::array Points = {
				Extent,
				Vec3(Extent.X(), Extent.Y(), -Extent.Z()),
				Vec3(Extent.X(), -Extent.Y(), Extent.Z()),
				Vec3(Extent.X(), -Extent.Y(), -Extent.Z()),
				Vec3(-Extent.X(), Extent.Y(), Extent.Z()),
				Vec3(-Extent.X(), Extent.Y(), -Extent.Z()),
				Vec3(-Extent.X(), -Extent.Y(), Extent.Z()),
				-Extent 
			};

			AABB Ab;
			for (auto& i : Points) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}

			return Ab;
		}

		virtual Vec3 GetSupportPoint(const Vec3& Pos, const Quat& Rot, const Vec3& NDir, const float Bias) const override {
			const std::array Points = { 
				Extent,
				Vec3(Extent.X(), Extent.Y(), -Extent.Z()),
				Vec3(Extent.X(), -Extent.Y(), Extent.Z()), 
				Vec3(Extent.X(), -Extent.Y(), -Extent.Z()),
				Vec3(-Extent.X(), Extent.Y(), Extent.Z()), 
				Vec3(-Extent.X(), Extent.Y(), -Extent.Z()), 
				Vec3(-Extent.X(), -Extent.Y(), Extent.Z()),
				-Extent 
			};
			Vec3 MaxPt = Rot.Rotate(Points[0]) + Pos;
			auto MaxDist = NDir.Dot(MaxPt);
			for (auto i = 1; i < std::size(Points); ++i) {
				const auto Pt = Rot.Rotate(Points[i]) + Pos;
				const auto Dist = NDir.Dot(Pt);
				if (Dist > MaxDist) {
					MaxDist = Dist;
					MaxPt = Pt;
				}
			}
			return MaxPt + NDir * Bias;
		}
		virtual float GetFastestPointSpeed(const Vec3& AngVel, const Vec3& Dir) const override {
			const std::array Points = {
				Extent,
				Vec3(Extent.X(), Extent.Y(), -Extent.Z()),
				Vec3(Extent.X(), -Extent.Y(), Extent.Z()),
				Vec3(Extent.X(), -Extent.Y(), -Extent.Z()),
				Vec3(-Extent.X(), Extent.Y(), Extent.Z()),
				Vec3(-Extent.X(), Extent.Y(), -Extent.Z()),
				Vec3(-Extent.X(), -Extent.Y(), Extent.Z()),
				-Extent 
			};
			auto MaxSpeed = 0.0f;
			for (auto& i : Points) {
				const auto Speed = Dir.Dot(AngVel.Cross(i - GetCenterOfMass()));
				if (Speed > MaxSpeed) {
					MaxSpeed = Speed;
				}
			}
		}

	public:
		Vec3 Extent = Vec3::One();
	};

	//!< #TODO
	class ShapeConvex : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeConvex() {}
		virtual ~ShapeConvex() {}

		virtual SHAPE GetShapeTyoe() const override { return SHAPE::CONVEX; }

		virtual Vec3 GetCenterOfMass() const override { return Super::GetCenterOfMass(); }
		virtual Mat3 GetInertiaTensor() const override {
			//!< #TODO
			return Mat3::Identity();
		}

		virtual AABB GetAABB(const Vec3& Pos, const Quat& Rot) const override {
			AABB Ab;
			for (auto& i : Points) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}
			return Ab;
		}

		virtual Vec3 GetSupportPoint(const Vec3& Pos, const Quat& Rot, const Vec3& NDir, const float Bias) const override {
#if 0
			return *std::ranges::max_element(Points, [&](const auto lhs, const auto rhs) { 
				return NDir.Dot(Rot.Rotate(lhs) + Pos) < NDir.Dot(Rot.Rotate(rhs) + Pos); 
			}) + NDir * Bias;
#else
			Vec3 MaxPt = Rot.Rotate(Points[0]) + Pos;
			auto MaxDist = NDir.Dot(MaxPt);
			for (auto i = 1; i < std::size(Points); ++i) {
				const auto Pt = Rot.Rotate(Points[i]) + Pos;
				const auto Dist = NDir.Dot(Pt);
				if (Dist > MaxDist) {
					MaxDist = Dist;
					MaxPt = Pt;
				}
			}
			return MaxPt + NDir * Bias;
#endif
		}
		virtual float GetFastestPointSpeed(const Vec3& AngVel, const Vec3& Dir) const override {
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
		std::vector<Vec3> Points;
	};
}

#ifdef _DEBUG
void CreateVertices_Diamond(std::vector<Vec3>& Dst) {
	Dst.reserve(7 * 8);

	std::vector<Vec3> Pts;
	Pts.reserve(4 + 3);

	Pts.emplace_back(Vec3(0.1f, 0.0f, -1.0f));
	Pts.emplace_back(Vec3(1.0f, 0.0f, 0.0f));
	Pts.emplace_back(Vec3(1.0f, 0.0f, 0.1f));
	Pts.emplace_back(Vec3(0.4f, 0.0f, 0.4f));

	constexpr auto Rad = 2.0f * std::numbers::pi_v<float> *0.125f;
	const auto QuatHalf = Quat(Vec3(0.0f, 0.0f, 1.0f), Rad * 0.5f);
	Pts.emplace_back(QuatHalf.Rotate(Vec3(0.8f, 0.0f, 0.3f)));
	Pts.emplace_back(QuatHalf.Rotate(Pts[1]));
	Pts.emplace_back(QuatHalf.Rotate(Pts[2]));

	std::ranges::copy(Pts, std::back_inserter(Dst));

	const Quat Q(Vec3(0.0f, 0.0f, 1.0f), Rad);
	Quat QuatAccumulator;
	for (auto i = 1; i < 8; ++i) {
		QuatAccumulator = QuatAccumulator * Q;
		for (auto j = 0; j < 7; ++j) {
			Dst.emplace_back(QuatAccumulator.Rotate(Pts[j]));
		}
	}
}
#endif