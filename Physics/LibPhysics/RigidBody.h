#pragma once

#include "PhysicsMath.h"

namespace Physics
{
	class Shape;

	class RigidBody
	{
	public:
		void Init(const Shape* Sh);

		[[nodiscard]] Math::Vec3 GetCenterOfMass() const;
		[[nodiscard]] Math::Vec3 GetWorldSpaceCenterOfMass() const { return Position + Rotation.Rotate(GetCenterOfMass()); }

		[[nodiscard]] Math::Mat3 GetWorldSpaceInverseInertiaTensor() const { return ToWorld(InvInertiaTensor); }

		[[nodiscard]] Math::Vec3 ToLocal(const Math::Vec3& rhs) const {
			return Rotation.Inverse().Rotate(rhs - GetWorldSpaceCenterOfMass());
		}
		[[nodiscard]] Math::Vec3 ToWorld(const Math::Vec3& rhs) const {
			return GetWorldSpaceCenterOfMass() + Rotation.Rotate(rhs);
		}
		[[nodiscard]] Math::Mat3 ToWorld(const Math::Mat3& rhs) const {
			const auto Rot3 = static_cast<const Math::Mat3>(Rotation);
			//!< 本来は Rot3 * rhs * Rot3.Inverse() だが、スケーリングが無いので Rot3 * rhs * Rot3.Transpose() で良い
			return Rot3 * rhs * Rot3.Transpose();
		}

		void ApplyGravity(const float DeltaSec) {
#if 1
			if (0.0f != InvMass) {
				LinearVelocity += Graivity * DeltaSec;				
			}
#else
			//!< 真面目にやるとこうだが、計算量が増えるだけ
			ApplyLinearImpulse(Graivity * DeltaSec / InvMass);
#endif
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

				//!< 角速度に限界値を設ける場合
				//constexpr auto AngVelLim = 30.0f;
				//if (AngularVelocity.LengthSq() > AngVelLim * AngVelLim) {
				//	AngularVelocity.ToNormalized();
				//	AngularVelocity *= AngVelLim;
				//}
			}
		}

		void ApplyTotalImpulse(const Math::Vec3& ImpactPoint, const Math::Vec3& Impulse) {
			if (0.0f != InvMass) {
				ApplyLinearImpulse(Impulse);
				//!< AngularJ = Radius x LinearJ
				const auto Radius = ImpactPoint - GetWorldSpaceCenterOfMass();
				ApplyAngularImpulse(Radius.Cross(Impulse));
			}
		}

		void Update(const float DeltaSec);

		Math::Vec3 Position = Math::Vec3::Zero();
		Math::Quat Rotation = Math::Quat::Identity();

		Math::Vec3 LinearVelocity = Math::Vec3::Zero();
		Math::Vec3 AngularVelocity = Math::Vec3::Zero();

		float InvMass = 1.0f;
		Math::Mat3 InvInertiaTensor = Math::Mat3::Identity();

		float Elasticity = 0.5f;
		float Friction = 0.5f;

		inline static const Math::Vec3 Graivity = Math::Vec3(0.0f, -9.8f, 0.0f);

		const Shape* Shape = nullptr;
	};
}