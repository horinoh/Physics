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
		virtual void Solve() {};
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

		ConstraintDistance& Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor);

	protected:
		//!< ヤコビ行列 (n * 12)
		//!<	n : コンストレイント数
		//!<	12 : 6(移動3、回転3)軸の自由度 * 2 オブジェクト
		//!< 距離 (n == 1)
		Math::Mat<1, 12> Jacobian;
		Math::Vec<1> CachedLambda;
		float Baumgarte = 0.0f;
	};

	class ConstraintHinge : public Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< 距離、ヒンジ軸に垂直な U, V (n == 3)
		Math::Mat<3, 12> Jacobian;
		Math::Vec<3> CachedLambda;
		float Baumgarte = 0.0f;

		//!< A, B の初期位置における InitQ = QA^-1 * QB をここに設定する
		Math::Quat InitialQuat;
		//!< 任意の時点の Q = QA^-1 * QB * InitQ.Inverse() とすると、初期位置からの回転角 Theta = 2.0f * asin(Q.Dot(Hinge));
	};

	class ConstraintLimitedHinge : public Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< 距離、ヒンジ軸に垂直な U, V、角度制限 (n == 4)
		Math::Mat<4, 12> Jacobian;
		Math::Vec<4> CachedLambda;
		float Baumgarte = 0.0f;

		Math::Quat InitialQuat;

		bool IsAngleViolated;
		float Angle;
	};

	class ConstraintBallSocket : public Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		//!< 距離、軸 (n == 2)
		Math::Mat<2, 12> Jacobian;
		Math::Vec<2> CachedLambda;
		float Baumgarte = 0.0f;

		Math::Quat InitialQuat;
	};

	class ConstraintLimitedBallSocket : public Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	protected:
		Math::Mat<4, 12> Jacobian;
		Math::Vec<4> CachedLambda;
		float Baumgarte = 0.0f;

		Math::Quat InitialQuat;

		bool IsAngleUViolated;
		bool IsAngleVViolated;
		float AngleU;
		float AngleV;
	};

	class ConstraintMover : public Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;

		ConstraintMover& Init(const Physics::RigidBody* Rb) { RigidBodyA = const_cast<Physics::RigidBody*>(Rb); return *this; }

	protected:
		float Timer = 0.0f;
	};

	class ConstraintPenetration : public Constraint
	{
	public:
		ConstraintPenetration() {}
		ConstraintPenetration(const Collision::Contact& Ct) { Init(Ct); }

		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;

		ConstraintPenetration& Init(const Collision::Contact& Ct);

	protected:
		//!< 法線 N、接面 U, V (n == 3)
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