#include "Constraint.h"
#include "RigidBody.h"
#include "Collision.h"

//!< Lambda = -J * v / J * M.Inverse() * J.Transpose()

//!< M = (M_A    0    0    0) ... A の質量が対角成分の 3x3 行列
//!<     (     I_A    0    0) ... A の慣性テンソル 3x3 行列
//!<     (   0   0  M_B    0) ... B の質量が対角成分の 3x3 行列
//!<     (   0   0    0  I_B) ... B の慣性テンソル 3x3 行列
Math::Mat<12, 12> Physics::Constraint::CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	Math::Mat<12, 12> InvM;

	//!< M_A
	InvM[0][0] = InvM[1][1] = InvM[2][2] = RbA->InvMass;

	//!< I_A
	const auto InvTensorA = RbA->GetWorldSpaceInverseInertiaTensor();
	for (auto i = 0; i < 3; ++i) {
		InvM[3 + i][3 + 0] = InvTensorA[i][0];
		InvM[3 + i][3 + 1] = InvTensorA[i][1];
		InvM[3 + i][3 + 2] = InvTensorA[i][2];
	}

	//!< M_B
	InvM[6][6] = InvM[7][7] = InvM[8][8] = RbB->InvMass;

	//!< I_B
	const auto InvTensorB = RbB->GetWorldSpaceInverseInertiaTensor();
	for (auto i = 0; i < 3; ++i) {
		InvM[9 + i][9 + 0] = InvTensorB[i][0];
		InvM[9 + i][9 + 1] = InvTensorB[i][1];
		InvM[9 + i][9 + 2] = InvTensorB[i][2];
	}

	return InvM;
}

//!< V = (V_A) ... A の速度
//!<     (W_A) ... A の角速度
//!<     (V_B) ... B の速度
//!<     (W_B) ... B の角速度
Math::Vec<12> Physics::Constraint::CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	auto V = Math::Vec<12>();

	//!< V_A
	V[0] = RbA->LinearVelocity.X();
	V[1] = RbA->LinearVelocity.Y();
	V[2] = RbA->LinearVelocity.Z();

	//!< W_A
	V[3] = RbA->AngularVelocity.X();
	V[4] = RbA->AngularVelocity.Y();
	V[5] = RbA->AngularVelocity.Z();

	//!< V_B
	V[6] = RbB->LinearVelocity.X();
	V[7] = RbB->LinearVelocity.Y();
	V[8] = RbB->LinearVelocity.Z();

	//!< W_B
	V[9] = RbB->AngularVelocity.X();
	V[10] = RbB->AngularVelocity.Y();
	V[11] = RbB->AngularVelocity.Z();

	return V;
}

void Physics::Constraint::ApplyImpulse(const Math::Vec<12>& Impulse)
{
	RigidBodyA->ApplyLinearImpulse(Math::Vec3({ Impulse[0], Impulse[1], Impulse[2] }));
	RigidBodyA->ApplyAngularImpulse(Math::Vec3({ Impulse[3], Impulse[4], Impulse[5] }));

	RigidBodyB->ApplyLinearImpulse(Math::Vec3({ Impulse[6], Impulse[7], Impulse[8] }));
	RigidBodyB->ApplyAngularImpulse(Math::Vec3({ Impulse[9], Impulse[10], Impulse[11] }));
}

void Physics::ConstraintDistance::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< ヤコビ行列を作成
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

	//!< 前のフレームの力(結果)を今フレームの計算前に適用することで、少ないフレームで安定状態へ持っていく (Warm starting)
	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	//!< 適正な位置へ戻すような力を数フレームかけて適用することで、位置ドリフトを修正 (Baumgarte stabilization)
	Baumgarte = 0.05f * (std::max)(AB.Dot(AB) - 0.01f, 0.0f) / DeltaSec;
}
void Physics::ConstraintDistance::Solve()  
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - Math::Vec<1>(Baumgarte);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintDistance::PostSolve() 
{
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -std::numeric_limits<float>::epsilon(), std::numeric_limits<float>::epsilon());
}

void Physics::ConstraintDistance::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	AnchorA = RigidBodyA->ToLocal(Anchor);

	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);
	AnchorB = RigidBodyB->ToLocal(Anchor);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);
}

void Physics::ConstraintPenetration::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< 法線に垂直な 2 軸を取得
	Math::Vec3 U, V;
	Normal.GetOrtho(U, V);
	//!< ワールドスペースへ
	const auto N = RigidBodyA->Rotation.Rotate(Normal);
	U = RigidBodyA->Rotation.Rotate(U);
	V = RigidBodyA->Rotation.Rotate(V);

	const auto J1 = -N;
	Jacobian[0][0] = J1.X();
	Jacobian[0][1] = J1.Y();
	Jacobian[0][2] = J1.Z();
	const auto J2 = RA.Cross(J1);
	Jacobian[0][3] = J2.X();
	Jacobian[0][4] = J2.Y();
	Jacobian[0][5] = J2.Z();
	const auto J3 = N;
	Jacobian[0][6] = J3.X();
	Jacobian[0][7] = J3.Y();
	Jacobian[0][8] = J3.Z();
	const auto J4 = RB.Cross(J3);
	Jacobian[0][9] = J4.X();
	Jacobian[0][10] = J4.Y();
	Jacobian[0][11] = J4.Z();

	//Friction = RigidBodyA->Friction * RigidBodyB->Friction;
	if (Friction > 0.0f) {
		{
			const auto J1 = -U;
			Jacobian[1][0] = J1.X();
			Jacobian[1][1] = J1.Y();
			Jacobian[1][2] = J1.Z();
			const auto J2 = RA.Cross(J1);
			Jacobian[1][3] = J2.X();
			Jacobian[1][4] = J2.Y();
			Jacobian[1][5] = J2.Z();
			const auto J3 = U;
			Jacobian[1][6] = J3.X();
			Jacobian[1][7] = J3.Y();
			Jacobian[1][8] = J3.Z();
			const auto J4 = RB.Cross(J3);
			Jacobian[1][9] = J4.X();
			Jacobian[1][10] = J4.Y();
			Jacobian[1][11] = J4.Z();
		}
		{
			const auto J1 = -V;
			Jacobian[2][0] = J1.X();
			Jacobian[2][1] = J1.Y();
			Jacobian[2][2] = J1.Z();
			const auto J2 = RA.Cross(J1);
			Jacobian[2][3] = J2.X();
			Jacobian[2][4] = J2.Y();
			Jacobian[2][5] = J2.Z();
			const auto J3 = V;
			Jacobian[2][6] = J3.X();
			Jacobian[2][7] = J3.Y();
			Jacobian[2][8] = J3.Z();
			const auto J4 = RB.Cross(J3);
			Jacobian[2][9] = J4.X();
			Jacobian[2][10] = J4.Y();
			Jacobian[2][11] = J4.Z();
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	Baumgarte = 0.25f * (std::min)((WAnchorB - WAnchorA).Dot(N) + 0.02f, 0.0f) / DeltaSec;
}
void Physics::ConstraintPenetration::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	auto B = -Jacobian * GetVelocties(); B[0] -= Baumgarte;

	auto Lambda = GaussSiedel(A, B);

	const auto Old = CachedLambda; {
		CachedLambda += Lambda;
		CachedLambda[0] = (std::max)(CachedLambda[0], 0.0f);
		if (Friction > 0.0f) {
			const auto uMg = Friction * std::fabsf(Physics::RigidBody::Graivity.Y()) / (RigidBodyA->InvMass + RigidBodyB->InvMass);
			const auto NForce = std::fabsf(Lambda[0] * Friction);
			const auto Force = (std::max)(uMg, NForce);
			CachedLambda[1] = (std::clamp)(CachedLambda[1], -Force, Force);
			CachedLambda[2] = (std::clamp)(CachedLambda[2], -Force, Force);
		}
	}
	Lambda = CachedLambda - Old;

	ApplyImpulse(JT * Lambda);
}

void Physics::ConstraintPenetration::Init(const Collision::Contact& Ct) 
{
	RigidBodyA = Ct.RigidBodyA;
	AnchorA = RigidBodyA->ToLocal(Ct.PointA);

	RigidBodyB = Ct.RigidBodyB;	
	AnchorB = RigidBodyB->ToLocal(Ct.PointB);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	//!< A 空間での法線
	Normal = RigidBodyA->Rotation.Inverse().Rotate(-Ct.Normal).Normalize();
	Friction = RigidBodyA->Friction * RigidBodyB->Friction;
}
