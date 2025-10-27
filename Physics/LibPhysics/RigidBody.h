#pragma once

#include "LinAlg.h"
#include "Collision.h"

namespace Physics
{
	class Shape;

	class RigidBody
	{
	public:
		RigidBody(const Shape* Sh, const float InvMass);

		[[nodiscard]] LinAlg::Vec3 GetCenterOfMass() const;
		[[nodiscard]] LinAlg::Vec3 GetCenterOfMass_World() const { return Position + Rotation.Rotate(GetCenterOfMass()); }

		[[nodiscard]] LinAlg::Vec3 ToLocal(const LinAlg::Vec3& rhs, const LinAlg::Vec3& Center) const {
			return Rotation.Inverse().Rotate(rhs - Center);
		}
		[[nodiscard]] LinAlg::Vec3 ToLocalPos(const LinAlg::Vec3& rhs) const { return ToLocal(rhs, GetCenterOfMass_World()); }
		[[nodiscard]] LinAlg::Vec3 ToWorldPos(const LinAlg::Vec3& rhs) const { return ToWorld(rhs, GetCenterOfMass_World()); } 
		
		[[nodiscard]] LinAlg::Vec3 ToWorld(const LinAlg::Vec3& rhs, const LinAlg::Vec3& Center) const {
			return Center + Rotation.Rotate(rhs);
		}
		[[nodiscard]] LinAlg::Vec3 ToLocalDir(const LinAlg::Vec3& rhs) const { return ToLocal(rhs, LinAlg::Vec3::Zero()); }
		[[nodiscard]] LinAlg::Vec3 ToWorldDir(const LinAlg::Vec3& rhs) const { return ToWorld(rhs, LinAlg::Vec3::Zero()); }

		[[nodiscard]] LinAlg::Mat3 ToWorld(const LinAlg::Mat3& rhs) const {
			const auto Mat3 = static_cast<const LinAlg::Mat3>(Rotation);
			return Mat3 * rhs * Mat3.Transpose();
		}
		[[nodiscard]] LinAlg::Mat3 GetInertiaTensor_World() const { return ToWorld(InertiaTensor); }
		[[nodiscard]] LinAlg::Mat3 GetInertiaTensor_Inverse_World() const { return ToWorld(InertiaTensor_Inverse); }

		void ApplyGravity(const float DeltaSec) {
			if (0.0f != Mass_Inverse) {
				Velocity_Linear += Graivity * DeltaSec;				
			}
		}
		void ApplyImpulse_Linear(const LinAlg::Vec3& Impulse) {
			if (0.0f != Mass_Inverse) {
				//!< J = m v より v = J / m
				Velocity_Linear += Impulse * Mass_Inverse;
			}
		}
		void ApplyImpulse_Angular(const LinAlg::Vec3& Impulse) {
			if (0.0f != Mass_Inverse) {
				//!< J = I \omega より \omega = I^-1 J
				Velocity_Angular += GetInertiaTensor_Inverse_World() * Impulse;

				//!< 角速度に限界値を設ける (通常パフォーマンス的理由による)
				constexpr auto Limit = 30.0f;
				if (Velocity_Angular.LengthSq() > Limit * Limit) {
					Velocity_Angular.Adjust(Limit);
				}
			}
		}

		void ApplyImpulse(const LinAlg::Vec3& ImpactPoint, const LinAlg::Vec3& Impulse) {
			if (0.0f != Mass_Inverse) {
				ApplyImpulse_Linear(Impulse);

				//!< 角力積 J_ang = Radius \cross J
				const auto Radius = ImpactPoint - GetCenterOfMass_World();
				ApplyImpulse_Angular(Radius.Cross(Impulse));
			}
		}

		static void ApplyImpulse(const Collision::Contact& Ct);

		void Update(const float DeltaSec);

		[[nodiscard]] const inline LinAlg::Vec3& GetPosition() const { return Position; }
		[[nodiscard]] const inline LinAlg::Quat& GetRotation() const { return Rotation; }
		[[nodiscard]] inline LinAlg::Vec3& GetPosition() { return Position; }
		[[nodiscard]] inline LinAlg::Quat& GetRotation() { return Rotation; }

		[[nodiscard]] const inline LinAlg::Vec3& GetVelocity_Linear() const { return Velocity_Linear; }
		[[nodiscard]] const inline LinAlg::Vec3& GetVelocity_Angular() const { return Velocity_Angular; }
		[[nodiscard]] inline LinAlg::Vec3& GetVelocity_Linear() { return Velocity_Linear; }
		[[nodiscard]] inline LinAlg::Vec3& GetVelocity_Angular() { return Velocity_Angular; }

		[[nodiscard]] inline float GetMass_Inverse() const { return Mass_Inverse; }
		
		[[nodiscard]] inline float GetElasticity() const { return Elasticity; }
		[[nodiscard]] inline float GetFriction() const { return Friction; }
		inline void SetElasticity(const float rhs) { Elasticity = rhs; }
		inline void SetFriction(const float rhs) { Friction = rhs; }

		[[nodiscard]] const inline Shape* GetShape() const { return Shape; }

	protected:
		LinAlg::Vec3 Position = LinAlg::Vec3::Zero();
		LinAlg::Quat Rotation = LinAlg::Quat::Identity();

		LinAlg::Vec3 Velocity_Linear = LinAlg::Vec3::Zero();
		LinAlg::Vec3 Velocity_Angular = LinAlg::Vec3::Zero();

		float Mass_Inverse = 1.0f;
		LinAlg::Mat3 InertiaTensor = LinAlg::Mat3::Zero();
		LinAlg::Mat3 InertiaTensor_Inverse;

		float Elasticity = 0.5f;
		float Friction = 0.5f;

		const Shape* Shape = nullptr;
	
	public:
		inline static const LinAlg::Vec3 Graivity = LinAlg::Vec3(0.0f, -9.8f, 0.0f);
	};
}