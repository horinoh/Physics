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
		//!< ���ʍs��
		//!< M = (M_A    0    0    0) ... A �̎��ʂ̋t�����Ίp���� (3x3)
		//!<     (     I_A    0    0) ... A �̊����e���\���̋t�s�� (3x3)
		//!<     (   0   0  M_B    0) ... B �̎��ʂ̋t�����Ίp����
		//!<     (   0   0    0  I_B) ... B �̊����e���\���̋t�s��
		using MassMatrix = Math::Mat<12, 12>;

		//!< V = (V_A) ... A �̑��x
		//!<     (W_A) ... A �̊p���x
		//!<     (V_B) ... B �̑��x
		//!<     (W_B) ... B �̊p���x
		using Velocities = Math::Vec<12>;
		
		virtual void PreSolve(const float DeltaSec) = 0;
		virtual void Solve() {};
		virtual void PostSolve() {}

	protected:
		Physics::RigidBody* RigidBodyA = nullptr;
	};
	class ConstraintAnchor : public Constraint
	{
	public:
		static MassMatrix CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		static Velocities CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);

		inline const MassMatrix& GetInverseMassMatrix() const { return InvMass; }
		Velocities GetVelocties() const { return CreateVelocties(RigidBodyA, RigidBodyB); }
		void ApplyImpulse(const Velocities& Impulse);

		ConstraintAnchor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		ConstraintAnchor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor);

	protected:
		Math::Vec3 AnchorA;

		Physics::RigidBody* RigidBodyB = nullptr;
		Math::Vec3 AnchorB;

		MassMatrix InvMass;
	};
	class ConstraintAnchorAxis : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintAnchorAxis& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis);

	protected:
		Math::Vec3 AxisA;
		Math::Vec3 AxisB;

		//!< ����u�Ԃ̉�]�� CurRot = QA.Inverse() * QB * InitRot.Inverse() �Ƃ���ƁA�����ʒu����̉�]�p�� Theta = 2.0f * asin(CurRot.Dot(Hinge)) �ŋ��܂�
		Math::Quat InvInitRot;
	};

	class ConstraintDistance : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

		ConstraintDistance& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor) {
			Super::Init(RbA, RbB, Anchor);
			return *this;
		}

	protected:
		//!< ���R�r�s�� (n * 12) n == �R���X�g���C���g��, 12 == 6 (�ړ�3�A��]3) ���̎��R�x * 2 �I�u�W�F�N�g
		//!< ���� (n == 1)
		Math::Mat<1, 12> Jacobian;
		Math::Vec<1> CachedLambda;

		//!< �K���Ȉʒu�֖߂��悤�ȗ͂�K�p���鎖�ňʒu�h���t�g���C�� (Baumgarte stabilization)
		//!< ��C�ɂ��ƃV�X�e���ɃG�l���M�[��ǉ���������ׁA���t���[�������ēK�p����
		float Baumgarte = 0.0f;
	};

	class ConstraintHinge : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

		ConstraintHinge& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis) {
			Super::Init(RbA, RbB, Anchor, Axis);
			return *this;
		}

	protected:
		//!< �����A�q���W���ɐ����� U, V (n == 3)
		Math::Mat<3, 12> Jacobian;
		Math::Vec<3> CachedLambda;

		float Baumgarte = 0.0f;
	};

	class ConstraintHingeLimited : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

		ConstraintHingeLimited& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis, const float LimAng = 45.0f) {
			Super::Init(RbA, RbB, Anchor, Axis);
			LimitAngle = LimAng;
			return *this;
		}

	protected:
		//!< �����A�q���W���ɐ����� U, V�A�p�x���� (n == 4)
		Math::Mat<4, 12> Jacobian;
		Math::Vec<4> CachedLambda;

		float Baumgarte = 0.0f;

		bool IsAngleViolated;
		float Angle;
		float LimitAngle;
	};

	class ConstraintBallSocket : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

		ConstraintBallSocket& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis) {
			Super::Init(RbA, RbB, Anchor, Axis);
			return *this;
		}

	protected:
		//!< �����A�� (n == 2)
		Math::Mat<2, 12> Jacobian;
		Math::Vec<2> CachedLambda;

		float Baumgarte = 0.0f;
	};

	class ConstraintBallSocketLimited : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

		ConstraintBallSocketLimited& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis, const float LimAngU = 45.0f, const float LimAngV = 45.0f) {
			Super::Init(RbA, RbB, Anchor, Axis);
			LimitAngleU = LimAngU;
			LimitAngleV = LimAngV;
			return *this;
		}

	protected:
		Math::Mat<4, 12> Jacobian;
		Math::Vec<4> CachedLambda;

		float Baumgarte = 0.0f;

		bool IsAngleUViolated;
		bool IsAngleVViolated;
		float AngleU;
		float AngleV;
		float LimitAngleU;
		float LimitAngleV;
	};

	class ConstraintMover : public Constraint
	{
	public:
		ConstraintMover& Init(const Physics::RigidBody* Rb) { RigidBodyA = const_cast<Physics::RigidBody*>(Rb); return *this; }
	};
	class ConstraintMoverRotate : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;
		virtual void PreSolve(const float DeltaSec) override;
	};
	class ConstraintMoverUpDown : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;
		virtual void PreSolve(const float DeltaSec) override;
	protected:
		float Timer = 0.0f;
	};

	class ConstraintMotor : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

		ConstraintMotor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis, const float Spd) {
			Super::Init(RbA, RbB, Anchor, Axis);
			Speed = Spd;
			return *this;
		}

	protected:
		//!< �����A�q���W���ɐ����� U, V�A�q���W�� (n == 4)
		Math::Mat<4, 12> Jacobian;

		Math::Vec3 Baumgarte;

		float Speed;
	};

	class ConstraintPenetration : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintPenetration() {}
		ConstraintPenetration(const Collision::Contact& Ct) { Init(Ct); }

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

		ConstraintPenetration& Init(const Collision::Contact& Ct);

	protected:
		//!< �@�� N�A�ږʂ� U, V (n == 3)
		Math::Mat<3, 12> Jacobian;
		Math::Vec<3> CachedLambda;

		float Baumgarte = 0.0f;

		Math::Vec3 Normal;
		float Friction = 0.0f;
	};
	class Manifold 
	{
	public:
		Manifold(const Collision::Contact& Ct);

		void Add(const Collision::Contact& CtOrig);
		void RemoveExpired();

	protected:
		friend class ManifoldCollector;

		//!< �R���X�g���N�g���ɃZ�b�g���� A, B �̏������o���Ă����A�ȍ~�̒ǉ��͂��̏��ɏ]��
		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		using ContactAndConstraint = std::pair<Collision::Contact, ConstraintPenetration>;
		std::vector<ContactAndConstraint> Constraints;
	};
	class ManifoldCollector
	{
	public:
		void Add(const Collision::Contact& Ct);
		void RemoveExpired() {
			for (auto& i : Manifolds) { i.RemoveExpired(); }
			const auto Range = std::ranges::remove_if(Manifolds, [](const auto& i) {return std::empty(i.Constraints); });
			Manifolds.erase(std::cbegin(Range), std::cend(Range));
		}

		void PreSolve(const float DeltaSec);
		void Solve();
		void PostSolve();

	protected:
		std::vector<Manifold> Manifolds;
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