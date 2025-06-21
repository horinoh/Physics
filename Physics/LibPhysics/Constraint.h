#pragma once

#include "PhysicsMath.h"

namespace Collision 
{
	struct Contact;
}

namespace Physics
{
	class RigidBody;

	//!< �R���X�g���C���g���
	class ConstraintBase
	{
	public:
		//!< ���R�r�s��̃Z�b�g�A�b�v
		virtual void PreSolve(const float DeltaSec) {};
		//!< �J��Ԃ��R�[�����邱�ƂŎ���������ׁA�֐������Ă���
		virtual void Solve() {};
		virtual void PostSolve() {}

	protected:
		//!< RigidBodyB ��K�v�Ƃ��Ȃ����� (ConstraintMover �n�Ȃ�) ������̂ŁA���ł� RigidBodyA �̂ݎ�������
		Physics::RigidBody* RigidBodyA = nullptr;
	};
	//!< 2���� A, B �Ƌt���ʍs�������
	class Constraint : public ConstraintBase
	{
	public:
		//!< ���ʍs��
		//!< M = (M_A    0    0    0) ... A �̎��ʂ̋t�����Ίp���� (3x3)
		//!<     (     I_A    0    0) ... A �̊����e���\���̋t�s�� (3x3)
		//!<     (   0   0  M_B    0) ... B �̎��ʂ̋t�����Ίp����
		//!<     (   0   0    0  I_B) ... B �̊����e���\���̋t�s��
		//!< #TODO �a�s���p�����œK���̗]�n����
		using MassMatrix = Math::Mat<3 * 4, 3 * 4>;

		//!< V = (V_A) ... A �̑��x
		//!<     (W_A) ... A �̊p���x
		//!<     (V_B) ... B �̑��x
		//!<     (W_B) ... B �̊p���x
		using Velocities = Math::Vec<3 * 4>;
	
	protected:
		Physics::RigidBody* RigidBodyB = nullptr;
		MassMatrix InvMass;
	};

	//!< ���ꂼ��̃A���J�[�ʒu������
	class ConstraintAnchor : public Constraint
	{
	public:
		ConstraintAnchor() {}
		ConstraintAnchor(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB) {
			Init(RbA, RbB);
		}
		ConstraintAnchor(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor) {
			Init(RbA, RbB, WAnchor);
		}

		static MassMatrix CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		static Velocities CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);

		inline const MassMatrix& GetInverseMassMatrix() const { return InvMass; }
		Velocities GetVelocties() const { return CreateVelocties(RigidBodyA, RigidBodyB); }
		void ApplyImpulse(const Velocities& Impulse);

		ConstraintAnchor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB);
		ConstraintAnchor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor);

	protected:
		Math::Vec3 LAnchorA;
		Math::Vec3 LAnchorB;
	};

	//!< ���������
	class ConstraintAnchorAxis : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintAnchorAxis() {}
		ConstraintAnchorAxis(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis) {
			Init(RbA, RbB, WAnchor, WAxis);
		}

		ConstraintAnchorAxis& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis);

	protected:
		Math::Vec3 LAxisA;

		Math::Quat InvInitRot;
	};

	class ConstraintDistance : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintDistance() {}
		ConstraintDistance(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor) {
			Init(RbA, RbB, WAnchor);
		}

		ConstraintDistance& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor) {
			Super::Init(RbA, RbB, WAnchor);
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< ���R�r�s�� (n * 12) n == �R���X�g���C���g��, 12 == 6 (�ړ�3�A��]3) ���̎��R�x * 2 �I�u�W�F�N�g
		//!< ���� (n == 1)
		Math::Mat<1, 3 * 4> Jacobian;
		Math::Vec<1> CachedLambda;

		//!< �K���Ȉʒu�֖߂��悤�ȗ͂�K�p���鎖�ňʒu�h���t�g���C�� (Baumgarte stabilization)
		//!< ��C�ɂ��ƃV�X�e���ɃG�l���M�[��ǉ���������ׁA���t���[�������ēK�p����
		float Baumgarte = 0.0f;
	};

	class ConstraintHinge : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;
		
		ConstraintHinge() {}
		ConstraintHinge(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis) {
			Init(RbA, RbB, WAnchor, WAxis);
		}

		ConstraintHinge& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< �����A�q���W���ɐ����� U, V (n == 3)
		Math::Mat<3, 3 * 4> Jacobian;
		Math::Vec<3> CachedLambda;

		float Baumgarte = 0.0f;
	};

	class ConstraintHingeLimited : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;
		
		ConstraintHingeLimited() {}
		ConstraintHingeLimited(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis, const float LimAng = 45.0f) {
			Init(RbA, RbB, WAnchor, WAxis, LimAng);
		}

		ConstraintHingeLimited& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis, const float LimAng = 45.0f) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			LimitAngle = LimAng;
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< �����A�q���W���ɐ����� U, V�A�p�x���� (n == 4)
		Math::Mat<4, 3 * 4> Jacobian;
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

		ConstraintBallSocket() {}
		ConstraintBallSocket(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis) {
			Init(RbA, RbB, WAnchor, WAxis);
		}

		ConstraintBallSocket& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;
	
	protected:
		//!< �����A�� (n == 2)
		Math::Mat<2, 3 * 4> Jacobian;
		Math::Vec<2> CachedLambda;

		float Baumgarte = 0.0f;
	};

	class ConstraintBallSocketLimited : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		ConstraintBallSocketLimited() {}
		ConstraintBallSocketLimited(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis, const float LimAngU = 45.0f, const float LimAngV = 45.0f) {
			Init(RbA, RbB, WAnchor, WAxis, LimAngU, LimAngV);
		}

		ConstraintBallSocketLimited& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis, const float LimAngU = 45.0f, const float LimAngV = 45.0f) {
			Super::Init(RbA, RbB, WAnchor, WAxis);

			LimitAngles = { LimAngU, LimAngV };

			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		Math::Mat<4, 3 * 4> Jacobian;
		Math::Vec<4> CachedLambda;

		float Baumgarte = 0.0f;

		std::array<bool, 2> IsAngleViolated = { false, false };
		std::array<float, 2> Angles;
		std::array<float, 2> LimitAngles;
	};

	class ConstraintMotor : public ConstraintAnchorAxis
	{
	public:
		using Super = ConstraintAnchorAxis;

		ConstraintMotor() {}
		ConstraintMotor(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis, const float Spd) {
			Init(RbA, RbB, WAnchor, WAxis, Spd);
		}

		ConstraintMotor& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis, const float Spd) {
			Super::Init(RbA, RbB, WAnchor, WAxis);
			Speed = Spd;
			return *this;
		}

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

	protected:
		//!< �����A�q���W���ɐ����� U, V�A�q���W�� (n == 4)
		Math::Mat<4, 3 * 4> Jacobian;

		Math::Vec3 Baumgarte;

		float Speed;
	};

	class ConstraintMover : public ConstraintBase
	{
	public:
		ConstraintMover() {}
		ConstraintMover(const Physics::RigidBody* Rb) { Init(Rb); }

		ConstraintMover& Init(const Physics::RigidBody* Rb) { RigidBodyA = const_cast<Physics::RigidBody*>(Rb); return *this; }
	};
	class ConstraintMoverUpDown : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;
		
		ConstraintMoverUpDown() {}
		ConstraintMoverUpDown(const Physics::RigidBody* Rb) { Init(Rb); }

		virtual void PreSolve(const float DeltaSec) override;
	protected:
		float Timer = 0.0f;
	}; 
	class ConstraintMoverRotateX : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;
		
		ConstraintMoverRotateX() {}
		ConstraintMoverRotateX(const Physics::RigidBody* Rb) { Init(Rb); }
		
		virtual void PreSolve(const float DeltaSec) override;
	};
	class ConstraintMoverRotateY : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;

		ConstraintMoverRotateY() {}
		ConstraintMoverRotateY(const Physics::RigidBody* Rb) { Init(Rb); }

		virtual void PreSolve(const float DeltaSec) override;
	};
	class ConstraintMoverRotateZ : public ConstraintMover
	{
	public:
		using Super = ConstraintMover;

		ConstraintMoverRotateZ() {}
		ConstraintMoverRotateZ(const Physics::RigidBody* Rb) { Init(Rb); }

		virtual void PreSolve(const float DeltaSec) override;
	};
	
	class ConstraintPenetration : public ConstraintAnchor
	{
	public:
		using Super = ConstraintAnchor;

		ConstraintPenetration() {}
		ConstraintPenetration(const Collision::Contact& Ct) { Init(Ct); }

		ConstraintPenetration& Init(const Collision::Contact& Ct);

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

	protected:
		//!< �@�� N�A�ږʂ� U, V (n == 3)
		Math::Mat<3, 3 * 4> Jacobian;
		Math::Vec<3> CachedLambda;

		float Baumgarte = 0.0f;

		Math::Vec3 LNormal;
		float Friction = 0.0f;
	};

	//!< 2 �I�u�W�F�N�g�Ԃ̏Փˏ��
	//!< �V�����Փ˓_��ǉ����Ă����A�����ɂȂ����Փ˂͍폜�����
	class Manifold
	{
	public:
		Manifold(const Collision::Contact& Ct);

		void Add(const Collision::Contact& CtOrig);
		void RemoveExpired();

	protected:
		friend class ManifoldCollector;
		static constexpr uint8_t MaxContacts = 4; //!< �o���Ă����Փː�

		//!< �R���X�g���N�g���ɃZ�b�g���� A, B �̏������o���Ă����A�ȍ~�̒ǉ��͂��̏��ɏ]��
		Physics::RigidBody* RigidBodyA = nullptr;
		Physics::RigidBody* RigidBodyB = nullptr;

		using ContactAndConstraint = std::pair<Collision::Contact, ConstraintPenetration>;
		std::vector<ContactAndConstraint> Constraints;
	};
	//!< 2 �I�u�W�F�N�g�Ԃ̏Փˏ�� (�}�j�t�H�[���h) �����W
	class ManifoldCollector
	{
	public:
		void Add(const Collision::Contact& Ct);
		void RemoveExpired() {
			//!< �����ɂȂ����Փ˓_���}�j�t�H�[���h����폜
			for (auto& i : Manifolds) { i.RemoveExpired(); }

			//!< �Փ˓_���P�������Ȃ��Ȃ����}�j�t�H�[���h���폜
			const auto Range = std::ranges::remove_if(Manifolds, 
				[](const auto& i) {
					return std::empty(i.Constraints);
				});
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

	//!< �K�E�X�U�C�f���@�ł͈ȉ��̂����ꂩ�̏ꍇ�� LCP ���������Ƃ��ł��� (�����ň����s��͑Ίp�D�ʂȂ̂ŁA�K�E�X�U�C�f���@���g�p�\)
	//!<	[1] ����l (positive definite) 
	//!<		v^t * M * v > 0 
	//!<		�x�N�g�� v �̓]�n�� v �ŋ��ނ悤�Ɋ|�����Ƃ��Ɍ��ʂ����ƂȂ�悤�ȍs�� M
	//!<
	//!<	[2] �Ίp�D�� (diagonally dominant) 
	//!<		|xy_ij| >= Sigma_j,j!=i |xy_ij|
	//!<		�Ίp�����̘a�̐�Βl���A��Ίp�����̘a�̐�Βl�ȏ�ƂȂ�悤�ȍs��
	template<size_t N>
	static Math::Vec<N> GaussSiedel(const Math::Mat<N, N>& A, const Math::Vec<N>& b, const uint32_t ItCount = 10)
	{
		auto x = Math::Vec<N>();

		for (uint32_t It = 0; It < ItCount; ++It) {
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