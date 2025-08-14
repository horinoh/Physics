#pragma once

#include <numbers>
#include <span>

#include "LinAlg.h"
#include "Collision.h"

namespace Physics
{
	class Shape
	{
	public:
		virtual ~Shape() {}

		virtual Shape* Init() { return this; }

		enum class SHAPE_TYPE
		{
			SPHERE,
			BOX,
			CONVEX,
		};
		[[nodiscard]] virtual SHAPE_TYPE GetShapeType() const = 0;

		[[nodiscard]] virtual Collision::AABB GetAABB(const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot) const = 0;
		[[nodiscard]] inline Collision::AABB GetAABB(std::span<const LinAlg::Vec3> Vertices, const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot) const {
			Collision::AABB Ab;
			for (auto& i : Vertices) {
				Ab.Expand(Rot.Rotate(i) + Pos);
			}
			return Ab;
		}

		[[nodiscard]] virtual LinAlg::Vec3 CalcCenterOfMass() const {
			return LinAlg::Vec3::Zero();
		}
		[[nodiscard]] virtual LinAlg::Mat3 CalcInertiaTensor() const {
			return LinAlg::Mat3::Identity();
		}
		[[nodiscard]] inline LinAlg::Mat3 GetParallelAxisTheoremTensor() const {
			//!< �d�S���Y���Ă���ꍇ�̊����e���\���̒���
			const auto R = -GetCenterOfMass();
			const auto R2 = R.LengthSq();
			const auto XX = R.X() * R.X();
			const auto XY = R.X() * R.Y();
			const auto XZ = R.X() * R.Z();
			const auto YY = R.Y() * R.Y();
			const auto YZ = R.Y() * R.Z();
			const auto ZZ = R.Z() * R.Z();
			return {
				{ R2 - XX,      XY,      XZ },
				{      XY, R2 - YY,      YZ },
				{      XZ,      YZ, R2 - ZZ }
			};
		}

		//!< �w��̕��� (UDir : ���K������Ă��邱��) �Ɉ�ԉ����_��Ԃ�
		[[nodiscard]] virtual LinAlg::Vec3 GetSupportPoint(const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot, const LinAlg::Vec3& UDir, const float Bias) const = 0;
		[[nodiscard]] inline LinAlg::Vec3 GetSupportPoint(std::span<const LinAlg::Vec3> Vertices, const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot, const LinAlg::Vec3& UDir, const float Bias) const {
			std::vector<LinAlg::Vec3> Pts;
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

		//!< ��]�ɂ���� UDir �����ɍł����������Ă��钸�_�̑�����Ԃ�
		[[nodiscard]] virtual float GetFastestRotatingPointSpeed(const LinAlg::Vec3& AngVel, const LinAlg::Vec3& UDir) const { return 0.0f; }
		[[nodiscard]] inline float GetFastestRotatingPointSpeed(std::span<const LinAlg::Vec3> Vertices, const LinAlg::Vec3& AngVel, const LinAlg::Vec3& UDir) const {
			std::vector<float> Speeds;
			Speeds.reserve(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(Speeds),
				[&](const auto& rhs) {
					return UDir.Dot(AngVel.Cross(rhs - GetCenterOfMass()));
				});
			return std::ranges::max(Speeds);
		}

		[[nodiscard]] const LinAlg::Vec3& GetCenterOfMass() const { return CenterOfMass; }
		[[nodiscard]] const LinAlg::Mat3& GetInertiaTensor() const { return InertiaTensor; }
		[[nodiscard]] const LinAlg::Mat3& GetInvInertiaTensor() const { return InvInertiaTensor; }

	protected:
		LinAlg::Vec3 CenterOfMass = LinAlg::Vec3::Zero();
		LinAlg::Mat3 InertiaTensor = LinAlg::Mat3::Identity();
		LinAlg::Mat3 InvInertiaTensor = LinAlg::Mat3::Identity();
	};

	class ShapeSphere : public Shape
	{
	private:
		using Super = Shape;
	public:
		ShapeSphere() {}
		ShapeSphere(const float R) : Radius(R) {}
		virtual ~ShapeSphere() {}

		virtual Shape* Init() override {
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return this;
		}

		virtual SHAPE_TYPE GetShapeType() const override { return SHAPE_TYPE::SPHERE; }

		//!< ���̊����e���\�� 2 / 5  * (R^2,   0,   0)
		//!<						 (  0, R^2,   0)
		//!<						 (  0,   0, R^2)
		virtual LinAlg::Mat3 CalcInertiaTensor() const override {
			return Radius * Radius * 2.0f / 5.0f * LinAlg::Mat3::Identity();
		}

		virtual Collision::AABB GetAABB(const LinAlg::Vec3& Pos, [[maybe_unused]] const LinAlg::Quat& Rot) const override {
			return { Pos - LinAlg::Vec3(Radius), Pos + LinAlg::Vec3(Radius) };
		}

		virtual LinAlg::Vec3 GetSupportPoint(const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot, const LinAlg::Vec3& UDir, const float Bias) const override {
			return Pos + UDir * (Radius + Bias);
		}

	public:
		float Radius = 1.0f;
	};

	class ShapeConvexBase : public Shape
	{
	private:
		using Super = Shape;

		virtual Collision::AABB GetAABB(const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot) const override {
			return Super::GetAABB(Vertices, Pos, Rot);
		}
		virtual LinAlg::Vec3 GetSupportPoint(const LinAlg::Vec3& Pos, const LinAlg::Quat& Rot, const LinAlg::Vec3& UDir, const float Bias) const override {
			return Super::GetSupportPoint(Vertices, Pos, Rot, UDir, Bias);
		}
		virtual float GetFastestRotatingPointSpeed(const LinAlg::Vec3& AngVel, const LinAlg::Vec3& UDir) const override {
			return Super::GetFastestRotatingPointSpeed(Vertices, AngVel, UDir);
		}
	//protected:
	public:
		std::vector<LinAlg::Vec3> Vertices;
		//!< �`�悵�Ȃ��ꍇ�͕s�v�A�����ł͈ꉞ�������Ă���
		std::vector<Collision::TriInds> Indices;
	};
	class ShapeBox : public ShapeConvexBase
	{
	private:
		using Super = ShapeConvexBase;
	public:
		ShapeBox() {}
		ShapeBox(const float Ext) {
			const auto R = Ext * 0.5f;
			Vertices = {
				LinAlg::Vec3(R),
				LinAlg::Vec3(R, R, -R),
				LinAlg::Vec3(R, -R, R),
				LinAlg::Vec3(R, -R, -R),
				LinAlg::Vec3(-R, R, R),
				LinAlg::Vec3(-R, R, -R),
				LinAlg::Vec3(-R, -R, R),
				LinAlg::Vec3(-R)
			};
		}
		ShapeBox(const LinAlg::Vec3& Ext) {
			const auto WR = Ext.X() * 0.5f;
			const auto HR = Ext.Y() * 0.5f;
			const auto DR = Ext.Z() * 0.5f;
			Vertices = {
				LinAlg::Vec3(WR, HR, DR),
				LinAlg::Vec3(WR, HR, -DR),
				LinAlg::Vec3(WR, -HR, DR),
				LinAlg::Vec3(WR, -HR, -DR),
				LinAlg::Vec3(-WR, HR, DR),
				LinAlg::Vec3(-WR, HR, -DR),
				LinAlg::Vec3(-WR, -HR, DR),
				LinAlg::Vec3(-WR, -HR, -DR)
			};
		}
		virtual ~ShapeBox() {}

		virtual Shape* Init() override {
			InertiaTensor = CalcInertiaTensor() + GetParallelAxisTheoremTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return this;
		}

		virtual SHAPE_TYPE GetShapeType() const override { return SHAPE_TYPE::BOX; }

		LinAlg::Vec3 CalcExtent() const {
			std::vector<float> Xs, Ys, Zs;
			Xs.reserve(std::size(Vertices));
			Ys.reserve(std::size(Vertices));
			Zs.reserve(std::size(Vertices));
			std::ranges::transform(Vertices, std::back_inserter(Xs), [](const auto& rhs) { return rhs.X(); });
			std::ranges::transform(Vertices, std::back_inserter(Ys), [](const auto& rhs) { return rhs.Y(); });
			std::ranges::transform(Vertices, std::back_inserter(Zs), [](const auto& rhs) { return rhs.Z(); });
			return LinAlg::Vec3(std::ranges::max(Xs) - std::ranges::min(Xs), 
				std::ranges::max(Ys) - std::ranges::min(Ys), 
				std::ranges::max(Zs) - std::ranges::min(Zs));
		}
		//!< �{�b�N�X�̊����e���\�� 1 / 12 * (H^2+D^2,       0,       0)
		//!<							  (      0, W^2+D^2,       0)
		//!<							  (      0,       0, W^2+H^2)
		virtual LinAlg::Mat3 CalcInertiaTensor() const override {
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
	};

	//!< �V�����_�[�̊����e���\�� (1/12 (3R^2 + H^2),                 0,       0)
	//!<                       (                0, 1/12 (3R^2 + H^2),       0)
	//!<					   (                0,                 0, 1/2 R^2)
	class ShapeConvex : public ShapeConvexBase
	{
	private:
		using Super = ShapeConvexBase;
	public:
		ShapeConvex() {}
		ShapeConvex(const std::vector<LinAlg::Vec3>& Mesh);
		virtual ~ShapeConvex() {}

		virtual Shape* Init() override {
			CenterOfMass = CalcCenterOfMass();
			InertiaTensor = CalcInertiaTensor();
			InvInertiaTensor = InertiaTensor.Inverse();
			return this;
		}

		virtual SHAPE_TYPE GetShapeType() const override { return SHAPE_TYPE::CONVEX; }

		virtual LinAlg::Vec3 CalcCenterOfMass() const override;
		virtual LinAlg::Mat3 CalcInertiaTensor() const override;
	};
	void CreateVertices_Diamond(std::vector<LinAlg::Vec3>& Dst);
}

