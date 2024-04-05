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
		//!< 質量行列
		//!< M = (M_A    0    0    0) ... A の質量の逆数が対角成分 (3x3)
		//!<     (     I_A    0    0) ... A の慣性テンソルの逆行列 (3x3)
		//!<     (   0   0  M_B    0) ... B の質量の逆数が対角成分
		//!<     (   0   0    0  I_B) ... B の慣性テンソルの逆行列
		using MassMatrix = Math::Mat<12, 12>;

		//!< V = (V_A) ... A の速度
		//!<     (W_A) ... A の角速度
		//!<     (V_B) ... B の速度
		//!<     (W_B) ... B の角速度
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

		//!< ある瞬間の回転を CurRot = QA.Inverse() * QB * InitRot.Inverse() とすると、初期位置からの回転角は Theta = 2.0f * asin(CurRot.Dot(Hinge)) で求まる
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
		//!< ヤコビ行列 (n * 12) n == コンストレイント数, 12 == 6 (移動3、回転3) 軸の自由度 * 2 オブジェクト
		//!< 距離 (n == 1)
		Math::Mat<1, 12> Jacobian;
		Math::Vec<1> CachedLambda;

		//!< 適正な位置へ戻すような力を適用する事で位置ドリフトを修正 (Baumgarte stabilization)
		//!< 一気にやるとシステムにエネルギーを追加しすぎる為、数フレームかけて適用する
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
		//!< 距離、ヒンジ軸に垂直な U, V (n == 3)
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
		//!< 距離、ヒンジ軸に垂直な U, V、角度制限 (n == 4)
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
		//!< 距離、軸 (n == 2)
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
		//!< 距離、ヒンジ軸に垂直な U, V、ヒンジ軸 (n == 4)
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
		//!< 法線 N、接面の U, V (n == 3)
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

		//!< コンストラクト時にセットした A, B の順序を覚えておく、以降の追加はこの順に従う
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
	//!<	行列 A, ベクトル b が既知の時、未知のベクトル x を求める

	//!< ガウスザイデル法では以下のいずれかの場合に LCP を解くことができる (これから扱う行列は対角優位なので、ガウスザイデル法が使用可能)
	//!<	[1] 正定値 (positive definite) 
	//!<		v^t * M * v > 0 
	//!<		ベクトル v の転地と v で挟むように掛けたときに結果が正となるような行列 M
	//!<
	//!<	[2] 対角優位 (diagonally dominant) 
	//!<		|xy_ij| >= Sigma_j,j!=i |xy_ij|
	//!<		対角成分の和の絶対値が、非対角成分の和の絶対値以上となるような行列
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