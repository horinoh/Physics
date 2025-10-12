#pragma once

#include "LinAlg.h"

namespace Physics
{
	class Shape;

	class RigidBody
	{
	public:
		RigidBody(const Shape* Sh, const float InvMass);

		//!< ローカル (モデル) 空間での重心
		[[nodiscard]] LinAlg::Vec3 GetCenterOfMass() const;
		//!< ワールド空間での重心
		[[nodiscard]] LinAlg::Vec3 GetWorldCenterOfMass() const { return Position + Rotation.Rotate(GetCenterOfMass()); }

		[[nodiscard]] LinAlg::Vec3 ToLocal(const LinAlg::Vec3& rhs, const LinAlg::Vec3& Center) const {
			return Rotation.Inverse().Rotate(rhs - Center);
		}
		[[nodiscard]] LinAlg::Vec3 ToLocalPos(const LinAlg::Vec3& rhs) const { return ToLocal(rhs, GetWorldCenterOfMass()); }
		[[nodiscard]] LinAlg::Vec3 ToWorldPos(const LinAlg::Vec3& rhs) const { return ToWorld(rhs, GetWorldCenterOfMass()); } 
		
		[[nodiscard]] LinAlg::Vec3 ToWorld(const LinAlg::Vec3& rhs, const LinAlg::Vec3& Center) const {
			return Center + Rotation.Rotate(rhs);
		}
		[[nodiscard]] LinAlg::Vec3 ToLocalDir(const LinAlg::Vec3& rhs) const { return ToLocal(rhs, LinAlg::Vec3::Zero()); }
		[[nodiscard]] LinAlg::Vec3 ToWorldDir(const LinAlg::Vec3& rhs) const { return ToWorld(rhs, LinAlg::Vec3::Zero()); }

		[[nodiscard]] LinAlg::Mat3 ToWorld(const LinAlg::Mat3& rhs) const {
			/*
			* J_local = R^-1 J_world
			* w_local = I_local^-1 J_local 
			* より 
			* w_local = I_local^-1 (R^-1 J_world)
			* 
			* w_world = R w_local なので 
			* &= R (I_local^-1 R^-1 J_world)
			* &= (R I_local^-1 R^-1) J_world
			* 
			* w_world = (I_world^-1) J_world であるので
			* I_world^-1 = R I_local^-1 R^-1 ということになる
			* 
			* スケーリングが無いので R^-1 は R^t でよい
			* 回転した慣性テンソルは以下の式で求まる
			* I_world^-1 = R I_local^-1 R^t
			*/
			const auto Rot3 = static_cast<const LinAlg::Mat3>(Rotation);
			return Rot3 * rhs * Rot3.Transpose();
		}
		[[nodiscard]] LinAlg::Mat3 GetWorldInertiaTensor() const { return ToWorld(InertiaTensor); }
		[[nodiscard]] LinAlg::Mat3 GetWorldInverseInertiaTensor() const { return ToWorld(InvInertiaTensor); }

		void ApplyGravity(const float DeltaSec) {
			if (0.0f != InvMass) {
				LinearVelocity += Graivity * DeltaSec;				
			}
		}
		void ApplyLinearImpulse(const LinAlg::Vec3& Impulse) {
			if (0.0f != InvMass) {
				/*
				* J = m dv
				* dv = J / m
				*/
				LinearVelocity += Impulse * InvMass;
			}
		}
		void ApplyAngularImpulse(const LinAlg::Vec3& Impulse) {
			if (0.0f != InvMass) {
				/*
				* J = I dw
				* dw = I^-1 J
				*/
				AngularVelocity += GetWorldInverseInertiaTensor() * Impulse;

				//!< 角速度に限界値を設ける (通常パフォーマンス的理由による)
				constexpr auto Limit = 30.0f;
				if (AngularVelocity.LengthSq() > Limit * Limit) {
					AngularVelocity.Adjust(Limit);
				}
			}
		}

		void ApplyImpulse(const LinAlg::Vec3& ImpactPoint, const LinAlg::Vec3& Impulse) {
			if (0.0f != InvMass) {
				ApplyLinearImpulse(Impulse);

				/*
				* J_angular = R \cross J_linear
				*/
				const auto Radius = ImpactPoint - GetWorldCenterOfMass();
				ApplyAngularImpulse(Radius.Cross(Impulse));
			}
		}

		void Update(const float DeltaSec);

		//!< モデル中心
		LinAlg::Vec3 Position = LinAlg::Vec3::Zero();
		LinAlg::Quat Rotation = LinAlg::Quat::Identity();

		LinAlg::Vec3 LinearVelocity = LinAlg::Vec3::Zero();
		LinAlg::Vec3 AngularVelocity = LinAlg::Vec3::Zero();

		float InvMass = 1.0f;
		LinAlg::Mat3 InertiaTensor = LinAlg::Mat3::Zero();
		LinAlg::Mat3 InvInertiaTensor = LinAlg::Mat3::Identity();

		float Elasticity = 0.5f;
		float Friction = 0.5f;

		inline static const LinAlg::Vec3 Graivity = LinAlg::Vec3(0.0f, -9.8f, 0.0f);

		const Shape* Shape = nullptr;
	};
}