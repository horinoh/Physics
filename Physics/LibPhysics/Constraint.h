#pragma once

#include "PhysicsMath.h"

namespace Collision 
{
	struct Contact;
}

namespace Physics
{
	class RigidBody;

	class Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) = 0;
		virtual void Solve() = 0;
		virtual void PostSolve() {}

		static Math::Mat<12, 12> CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		static Math::Vec<12> CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);

		inline const Math::Mat<12, 12>& GetInverseMassMatrix() const { return InvMass; }
		Math::Vec<12> GetVelocties() const { return CreateVelocties(RigidBodyA, RigidBodyB); }
		void ApplyImpulse(const Math::Vec<12>& Impulse);

	protected:
		Physics::RigidBody* RigidBodyA = nullptr;
		Math::Vec3 AnchorA;
		Math::Vec3 AxisA;

		Physics::RigidBody* RigidBodyB = nullptr;
		Math::Vec3 AnchorB;
		Math::Vec3 AxisB;

		Math::Mat<12, 12> InvMass;
	};
	class ConstraintDistance : public Constraint 
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

		void Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor);

	protected:
		Math::Mat<1, 12> Jacobian;
		Math::Vec<1> CachedLambda;
		float Baumgarte = 0.0f;
	};

	class ConstraintPenetration : public Constraint 
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

		void Init(const Collision::Contact& Ct);

	protected:
		Math::Mat<3, 12> Jacobian;
		Math::Vec<3> CachedLambda;
		float Baumgarte = 0.0f;

		Math::Vec3 Normal;
		float Friction = 0.0f;
	};

	//!< Linear Complimentary Problem (LCP)
	//!<	A * x = b 
	//!<	�s�� A, �x�N�g�� b �����m�̎��A���m�̃x�N�g�� x �����߂�

	//!< �K�E�X�U�C�f���@�ł͈ȉ��̂����ꂩ�̏ꍇ�� LCP ���������Ƃ��ł��� (���ꂩ�爵���s��͑Ίp�D�ʂȂ̂ŁA�K�E�X�U�C�f���@���g�p�\)
	//!<	[1] ����l (positive definite) 
	//!<		v^t * M * v > 0 
	//!<		�x�N�g�� v �̓]�n�� v �ŋ��ނ悤�Ɋ|�����Ƃ��Ɍ��ʂ����ƂȂ�悤�ȍs�� M
	//!<
	//!<	[2] �Ίp�D�� (diagonally dominant) 
	//!<		|xy_ij| >= Sigma_j,j!=i |xy_ij|
	//!<		�Ίp�����̘a�̐�Βl���A��Ίp�����̘a�̐�Βl�ȏ�ƂȂ�悤�ȍs��
	template<size_t N>
	static Math::Vec<N> GaussSiedel(const Math::Mat<N, N>& A, const Math::Vec<N>& b)
	{
		auto x = Math::Vec<N>();

		for (auto It = 0; It < N; ++It) {
			for (auto i = 0; i < N; ++i) {
				const auto dx = (b[i] - A[i].Dot(x)) / A[i][i];
				if (dx * 0.01f == dx * 0.01f) {
					x[i] += dx;
				}
			}
		}

		return x;
	}
}