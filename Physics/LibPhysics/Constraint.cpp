#include "Constraint.h"
#include "RigidBody.h"

//!< Lambda = -J * v / J * M.Inverse() * J.Transpose()

//!< M = (M_A    0    0    0) ... A の質量が対角成分の 3x3 行列
//!<     (     I_A    0    0) ... A の慣性テンソル 3x3 行列
//!<     (   0   0  M_B    0) ... B の質量が対角成分の 3x3 行列
//!<     (   0   0    0  I_B) ... B の慣性テンソル 3x3 行列
template<uint32_t N>
Math::Mat<N, N> Physics::Constraint::GetInverseMassMatrix()
{
	Math::Mat<N, N> InvMass;

	//!< M_A
	InvMass[0][0] = InvMass[1][1] = InvMass[2][2] = RigidBodyA->InvMass;

	//!< I_A
	const auto InvTensorA = RigidBodyA->GetWorldSpaceInverseInertiaTensor();
	for (auto i = 0; i < 3; ++i) {
		InvMass[3 + i][3 + 0] = InvTensorA[i][0];
		InvMass[3 + i][3 + 1] = InvTensorA[i][1];
		InvMass[3 + i][3 + 2] = InvTensorA[i][2];
	}

	//!< M_B
	InvMass[6][6] = InvMass[7][7] = InvMass[8][8] = RigidBodyB->InvMass;

	//!< I_B
	const auto InvTensorB = RigidBodyB->GetWorldSpaceInverseInertiaTensor();
	for (auto i = 0; i < 3; ++i) {
		InvMass[9 + i][9 + 0] = InvTensorB[i][0];
		InvMass[9 + i][9 + 1] = InvTensorB[i][1];
		InvMass[9 + i][9 + 2] = InvTensorB[i][2];
	}

	return InvMass;
}

//!< V = (V_A) ... A の速度
//!<     (W_A) ... A の角速度
//!<     (V_B) ... B の速度
//!<     (W_B) ... B の角速度
template<uint32_t N>
Math::Vec<N> Physics::Constraint::GetVelocties()
{
	auto v = Math::Vec<N>();

	//!< V_A
	v[0] = RigidBodyA->LinearVelocity.X();
	v[1] = RigidBodyA->LinearVelocity.Y();
	v[2] = RigidBodyA->LinearVelocity.Z();

	//!< W_A
	v[3] = RigidBodyA->AngularVelocity.X();
	v[4] = RigidBodyA->AngularVelocity.Y();
	v[5] = RigidBodyA->AngularVelocity.Z();

	//!< V_B
	v[6] = RigidBodyB->LinearVelocity.X();
	v[7] = RigidBodyB->LinearVelocity.Y();
	v[8] = RigidBodyB->LinearVelocity.Z();

	//!< W_B
	v[9] = RigidBodyB->AngularVelocity.X();
	v[10] = RigidBodyB->AngularVelocity.Y();
	v[11] = RigidBodyB->AngularVelocity.Z();

	return v;
}

template<uint32_t N>
void  Physics::Constraint::ApplyImpulse(const Math::Vec<N>& Impulse)
{
	RigidBodyA->ApplyLinearImpulse(Math::Vec3({ Impulse[0], Impulse[1], Impulse[2] }));
	RigidBodyA->ApplyAngularImpulse(Math::Vec3({ Impulse[3], Impulse[4], Impulse[5] }));

	RigidBodyB->ApplyLinearImpulse(Math::Vec3({ Impulse[6], Impulse[7], Impulse[8] }));
	RigidBodyB->ApplyAngularImpulse(Math::Vec3({ Impulse[9], Impulse[10], Impulse[11] }));
}

//!< Distance
void Physics::ConstraintDistance::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	const auto J1 = BA * 2.0f;
	Jacobian[0][0] = J1.X();
	Jacobian[0][1] = J1.Y();
	Jacobian[0][2] = J1.Z();
	const auto J2 = RA.Cross(J1);
	Jacobian[0][3] = J2.X();
	Jacobian[0][4] = J2.Y();
	Jacobian[0][5] = J2.Z();
	const auto J3 = AB * 2.0f;
	Jacobian[0][6] = J3.X();
	Jacobian[0][7] = J3.Y();
	Jacobian[0][8] = J3.Z();
	const auto J4 = RB.Cross(J3);
	Jacobian[0][9] = J4.X();
	Jacobian[0][10] = J4.Y();
	Jacobian[0][11] = J4.Z();

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	Baumgarte = 0.05f / DeltaSec * (std::max)(AB.Dot(AB) - 0.01f, 0.0f);
}
void Physics::ConstraintDistance::Solve()  
{
	const auto JT = Jacobian.Transpose();
	auto rhs = -Jacobian * GetVelocties<12>();
	rhs[0] -= Baumgarte;

	//const auto Lambda = GaussSiedel(Jacobian * GetInverseMassMatrix<12>() * JT, rhs);

	//ApplyImpulse(JT * Lambda);

	//CachedLambda += Lambda;
}
void Physics::ConstraintDistance::PostSolve() 
{
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -std::numeric_limits<float>::epsilon(), std::numeric_limits<float>::epsilon());
}
