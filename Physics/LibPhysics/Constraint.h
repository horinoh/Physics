#pragma once

#include "PhysicsMath.h"

namespace Physics
{
	class RigidBody;

	class Constraint
	{
	public:
		virtual void PreSolve(const float DeltaSec) = 0;
		virtual void Solve() = 0;
		virtual void PostSolve() {}

		Math::Mat<12, 12> GetInverseMassMatrix() const;
		Math::Vec<12> GetVelocties() const;
		void ApplyImpulse(const Math::Vec<12>& Impulse);

		Physics::RigidBody* RigidBodyA = nullptr;
		Math::Vec3 AnchorA;
		Math::Vec3 AxisA;

		Physics::RigidBody* RigidBodyB = nullptr;
		Math::Vec3 AnchorB;
		Math::Vec3 AxisB;
	};
	class ConstraintDistance : public Constraint 
	{
	public:
		virtual void PreSolve(const float DeltaSec) override;
		virtual void Solve() override;
		virtual void PostSolve() override;

	private:
		Math::Mat<1, 12> Jacobian;
		Math::Vec<1> CachedLambda;
		float Baumgarte;
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