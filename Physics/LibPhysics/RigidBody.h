#pragma once

#include "PhysicsMath.h"

namespace Physics
{
	class Shape;

	class RigidBody
	{
	public:
		RigidBody(const Shape* Sh, const float InvMass);

		[[nodiscard]] Math::Vec3 ToLocal(const Math::Vec3& rhs, const Math::Vec3& Center) const {
			return Rotation.Inverse().Rotate(rhs - Center);
		}
		[[nodiscard]] Math::Vec3 ToWorld(const Math::Vec3& rhs, const Math::Vec3& Center) const {
			return Center + Rotation.Rotate(rhs);
		}
		[[nodiscard]] Math::Vec3 ToLocalPos(const Math::Vec3& rhs) const { return ToLocal(rhs, GetWorldSpaceCenterOfMass()); }
		[[nodiscard]] Math::Vec3 ToWorldPos(const Math::Vec3& rhs) const { return ToWorld(rhs, GetWorldSpaceCenterOfMass()); }
		[[nodiscard]] Math::Vec3 ToLocalDir(const Math::Vec3& rhs) const { return ToLocal(rhs, Math::Vec3::Zero()); }
		[[nodiscard]] Math::Vec3 ToWorldDir(const Math::Vec3& rhs) const { return ToWorld(rhs, Math::Vec3::Zero()); }

		[[nodiscard]] Math::Mat3 ToWorld(const Math::Mat3& rhs) const {
			const auto Rot3 = static_cast<const Math::Mat3>(Rotation);
			//!< 本来は Rot3 * rhs * Rot3.Inverse() だが、スケーリングが無いので Rot3 * rhs * Rot3.Transpose() で良い
			return Rot3 * rhs * Rot3.Transpose();
		}

		//!< ローカル (モデル) 空間での重心
		[[nodiscard]] Math::Vec3 GetCenterOfMass() const;
		//!< ワールド空間での重心
		[[nodiscard]] Math::Vec3 GetWorldSpaceCenterOfMass() const { return Position + Rotation.Rotate(GetCenterOfMass()); }

		[[nodiscard]] Math::Mat3 GetWorldSpaceInertiaTensor() const { return ToWorld(InertiaTensor); }
		[[nodiscard]] Math::Mat3 GetWorldSpaceInverseInertiaTensor() const { return ToWorld(InvInertiaTensor); }

		void ApplyGravity(const float DeltaSec) {
			if (0.0f != InvMass) {
				LinearVelocity += Graivity * DeltaSec;				
			}
		}
		void ApplyLinearImpulse(const Math::Vec3& Impulse) {
			if (0.0f != InvMass) {
				LinearVelocity += Impulse * InvMass;
			}
		}
		void ApplyAngularImpulse(const Math::Vec3& Impulse) {
			if (0.0f != InvMass) {
				//!< w = Inv(I) * AngularJ 
				AngularVelocity += GetWorldSpaceInverseInertiaTensor() * Impulse;

				//!< 角速度に限界値を設ける (通常パフォーマンス的理由による)
				constexpr auto Limit = 30.0f;
				if (AngularVelocity.LengthSq() > Limit * Limit) {
					AngularVelocity.Adjust(Limit);
				}
			}
		}

		void ApplyImpulse(const Math::Vec3& ImpactPoint, const Math::Vec3& Impulse) {
			if (0.0f != InvMass) {
				ApplyLinearImpulse(Impulse);
				//!< AngularJ = Radius x LinearJ
				const auto Radius = ImpactPoint - GetWorldSpaceCenterOfMass();
				ApplyAngularImpulse(Radius.Cross(Impulse));
			}
		}

		void Update(const float DeltaSec);

		//!< モデル中心
		Math::Vec3 Position = Math::Vec3::Zero();
		Math::Quat Rotation = Math::Quat::Identity();

		Math::Vec3 LinearVelocity = Math::Vec3::Zero();
		Math::Vec3 AngularVelocity = Math::Vec3::Zero();

		float InvMass = 1.0f;
		Math::Mat3 InertiaTensor = Math::Mat3::Zero();
		Math::Mat3 InvInertiaTensor = Math::Mat3::Identity();

		float Elasticity = 0.5f;
		float Friction = 0.5f;

		inline static const Math::Vec3 Graivity = Math::Vec3(0.0f, -9.8f, 0.0f);

		const Shape* Shape = nullptr;
	};
}