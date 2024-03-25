#include "Constraint.h"
#include "RigidBody.h"
#include "Collision.h"

#include "Log.h"

//!< Lambda = -J * v / (J * M.Inverse() * J.Transpose())
//!<	Lambda : ���O�����W���̖���搔 (Lagrange multiplier) 
//!<	J : ���R�r�s�� dC/dt = J * v = 0

//!< ���ʍs��
//!< M = (M_A    0    0    0) ... A �̎��ʂ̋t�����Ίp���� (3x3)
//!<     (     I_A    0    0) ... A �̊����e���\���̋t�s�� (3x3)
//!<     (   0   0  M_B    0) ... B �̎��ʂ̋t�����Ίp����
//!<     (   0   0    0  I_B) ... B �̊����e���\���̋t�s��
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
//!< V = (V_A) ... A �̑��x
//!<     (W_A) ... A �̊p���x
//!<     (V_B) ... B �̑��x
//!<     (W_B) ... B �̊p���x
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

	//!< ���R�r�s����쐬
	const auto J1 = -AB * 2.0f;
	const auto J2 = RA.Cross(J1);
	const auto J3 = -BA * 2.0f;
	const auto J4 = RB.Cross(J3);
	Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };

	//!< �O�̃t���[���̗� (CachedLambda) �����t���[���̌v�Z�O�ɓK�p���邱�ƂŁA���Ȃ��t���[���ň����Ԃ֎����Ă��� (Warm starting)
	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	//!< �K���Ȉʒu�֖߂��悤�ȗ͂�K�p���鎖�ňʒu�h���t�g���C�� (Baumgarte stabilization)
	//!< �V�X�e���ɂ킸���ɃG�l���M�[��ǉ�����ׁA�U�����Ă��܂����ւ̑Ώ� (Slop)
	const auto C = (std::max)(AB.Dot(AB) - 0.01f, 0.0f);
	//!< ��C�ɂ��ƃV�X�e���ɃG�l���M�[��ǉ���������ׁA���t���[�������ēK�p����
	Baumgarte = 0.05f * C / DeltaSec;
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
	//!< ���U��h���Ώ� (Warm starting)
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	constexpr auto Eps = std::numeric_limits<float>::epsilon();
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Eps, Eps);
}
Physics::ConstraintDistance& Physics::ConstraintDistance::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	AnchorA = RigidBodyA->ToLocal(Anchor);

	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);
	AnchorB = RigidBodyB->ToLocal(Anchor);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	return *this;
}

void Physics::ConstraintHinge::PreSolve(const float DeltaSec) 
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< ����
	const auto J1 = -AB * 2.0f;
	const auto J2 = RA.Cross(J1);
	const auto J3 = -BA * 2.0f;
	const auto J4 = RB.Cross(J3);
	Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvInitQ = InitialQuat.Inverse();
	const auto InvQA = QA.Inverse();

	//!< �q���W���ɐ����� U, V
	Math::Vec3 U, V;
	AxisA.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::Zero(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::AxisW());
	//!< (�����ł�) �]�u����Ӗ����Ȃ����A�ꉞ����Ă���
	const auto PT = P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitQ).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(0.0f, U.X(), U.Y(), U.Z());
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(0.0f, U.X(), U.Y(), U.Z());
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(0.0f, V.X(), V.Y(), V.Z());
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(0.0f, V.X(), V.Y(), V.Z());
		Jacobian[2] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	
	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	Baumgarte = 0.05f * C / DeltaSec;
}
void Physics::ConstraintHinge::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	auto B = -Jacobian * GetVelocties(); B[0] -= Baumgarte;

	auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintHinge::PostSolve()
{
	constexpr auto Limit = 20.0f;
	for (auto i = 0; i < CachedLambda.Size();++i) {
		if (CachedLambda[i] * 0.0f != CachedLambda[i] * 0.0f) { CachedLambda[i] = 0.0f; }
		CachedLambda[i] = (std::clamp)(CachedLambda[i], -Limit, Limit);
	}
}

void Physics::ConstraintLimitedHinge::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	const auto J1 = -AB * 2.0f;
	const auto J2 = RA.Cross(J1);
	const auto J3 = -BA * 2.0f;
	const auto J4 = RB.Cross(J3);
	Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvInitQ = InitialQuat.Inverse();
	const auto InvQA = QA.Inverse();

	Math::Vec3 U, V;
	AxisA.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::Zero(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::AxisW());
	const auto PT = P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitQ).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	//!< �p�x�����߁A�j�]���Ă��邩�ǂ�������
	const auto qrr = InvQA * QB * InvInitQ;
	//Angle = 2.0f * asinf(Math::Vec3(qrr.X(), qrr.Y(), qrr.Z()).Dot(AxisA)) * 180.0f / std::numbers::pi_v<float>;
	Angle = TO_DEGREE(2.0f * asinf(Math::Vec3(qrr.X(), qrr.Y(), qrr.Z()).Dot(AxisA)));
	IsAngleViolated = std::fabsf(Angle) > 45.0f;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(0.0f, U.X(), U.Y(), U.Z());
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(0.0f, U.X(), U.Y(), U.Z());
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(0.0f, V.X(), V.Y(), V.Z());
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(0.0f, V.X(), V.Y(), V.Z());
		Jacobian[2] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	if (IsAngleViolated) {
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(0.0f, AxisA.X(), AxisA.Y(), AxisA.Z());
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(0.0f, AxisA.X(), AxisA.Y(), AxisA.Z());
		Jacobian[3] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	else {
		Jacobian[3].ToZero();
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	Baumgarte = 0.05f * C / DeltaSec;
}
void Physics::ConstraintLimitedHinge::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	auto B = -Jacobian * GetVelocties(); B[0] -= Baumgarte;

	auto Lambda = GaussSiedel(A, B);

	//!< �p�x�����ɂ��A�g���N�𐧌�����
	if (IsAngleViolated) {
		if (Angle > 0.0f) {
			Lambda[3] = (std::min)(Lambda[3], 0.0f);
		}
		if (Angle < 0.0f) {
			Lambda[3] = (std::max)(Lambda[3], 0.0f);
		}
	}

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintLimitedHinge::PostSolve()
{
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	constexpr auto Limit = 20.0f;
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Limit, Limit);
	CachedLambda[1] = CachedLambda[2] = CachedLambda[3] = 0.0f;
}

void Physics::ConstraintBallSocket::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< ����
	const auto J1 = -AB * 2.0f;
	const auto J2 = RA.Cross(J1);
	const auto J3 = -BA * 2.0f;
	const auto J4 = RB.Cross(J3);
	Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvInitQ = InitialQuat.Inverse();
	const auto InvQA = QA.Inverse();

	const auto P = Math::Mat4(Math::Vec4::Zero(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::AxisW());
	//!< (�����ł�) �]�u����Ӗ����Ȃ����A�ꉞ����Ă���
	const auto PT = P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitQ).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = -0.5f * MatA * Math::Vec4(0.0f, AxisA.X(), AxisA.Y(), AxisA.Z());
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = 0.5f * MatB * Math::Vec4(0.0f, AxisA.X(), AxisA.Y(), AxisA.Z());
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	Baumgarte = 0.05f * C / DeltaSec;
}
void Physics::ConstraintBallSocket::Solve() 
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	auto B = -Jacobian * GetVelocties(); B[0] -= Baumgarte;

	auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintBallSocket::PostSolve()
{
	constexpr auto Limit = 20.0f;
	for (auto i = 0; i < CachedLambda.Size(); ++i) {
		if (CachedLambda[i] * 0.0f != CachedLambda[i] * 0.0f) { CachedLambda[i] = 0.0f; }
		CachedLambda[i] = (std::clamp)(CachedLambda[i], -Limit, Limit);
	}
}

void Physics::ConstraintLimitedBallSocket::PreSolve(const float DeltaSec)
{
}
void Physics::ConstraintLimitedBallSocket::Solve()
{
}
void Physics::ConstraintLimitedBallSocket::PostSolve()
{
}
void Physics::ConstraintMover::PreSolve(const float DeltaSec)
{
	RigidBodyA->LinearVelocity[1] = std::cosf(Timer) * 4.0f;
	Timer += DeltaSec;
}
void Physics::ConstraintPenetration::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorld(AnchorA);
	const auto WAnchorB = RigidBodyB->ToWorld(AnchorB);

	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< �@���ɐ����� 2 �����擾
	Math::Vec3 U, V;
	Normal.GetOrtho(U, V);
	//!< N, U, V �����[���h�X�y�[�X��
	const auto N = RigidBodyA->Rotation.Rotate(Normal);
	U = RigidBodyA->Rotation.Rotate(U);
	V = RigidBodyA->Rotation.Rotate(V);

	const auto J1 = -N;
	const auto J2 = RA.Cross(J1);
	const auto J3 = N;
	const auto J4 = RB.Cross(J3);
	Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };

	if (Friction > 0.0f) {
		{
			const auto J1 = -U;
			const auto J2 = RA.Cross(J1);
			const auto J3 = U;
			const auto J4 = RB.Cross(J3);
			Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
		}
		{
			const auto J1 = -V;
			const auto J2 = RA.Cross(J1);
			const auto J3 = V;
			const auto J4 = RB.Cross(J3);
			Jacobian[2] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::min)((WAnchorB - WAnchorA).Dot(N) + 0.02f, 0.0f);
	Baumgarte = 0.25f * C / DeltaSec;
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
Physics::ConstraintPenetration& Physics::ConstraintPenetration::Init(const Collision::Contact& Ct)
{
	RigidBodyA = Ct.RigidBodyA;
	AnchorA = RigidBodyA->ToLocal(Ct.PointA);

	RigidBodyB = Ct.RigidBodyB;
	AnchorB = RigidBodyB->ToLocal(Ct.PointB);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	//!< A ��Ԃł̖@��
	Normal = RigidBodyA->Rotation.Inverse().Rotate(Ct.Normal).Normalize();
	Friction = RigidBodyA->Friction * RigidBodyB->Friction;

	return *this;
}

Physics::Manifold::Manifold(const Collision::Contact& Ct) : RigidBodyA(Ct.RigidBodyA), RigidBodyB(Ct.RigidBodyB) { Add(Ct); }
void Physics::Manifold::Add(const Collision::Contact& CtOrig)
{
	//!< ���� A, B �̏������H������Ă���ꍇ�͒�������킹��
	auto Ct = CtOrig;
	if (RigidBodyA != CtOrig.RigidBodyA) { Ct.Swap(); }

	//!< �����Ƃقړ����ڐG�ʒu�̏ꍇ�͉������Ȃ�
	constexpr auto Eps2 = 0.02f * 0.02f;
	if (std::ranges::any_of(Constraints, [&](const auto& i) {
		return (i.first.PointA - Ct.PointA).LengthSq() < Eps2 || (i.first.PointB - Ct.PointB).LengthSq() < Eps2;
	})) {
		return;
	}

	const auto NewItem = ContactAndConstraint({ Ct, ConstraintPenetration(Ct) });
	if (std::size(Constraints) < 4) {
		//!< �󂫂�����ꍇ�͒ǉ�
		Constraints.emplace_back(NewItem);
	}
	else {
		//!< ���ϒl
		const auto Avg = (std::accumulate(std::cbegin(Constraints), std::cend(Constraints), Math::Vec3::Zero(), [](const auto& Acc, const auto& i) {
			return Acc + i.first.PointA; 
		}) + Ct.PointA) / 5.0f;

		//!< (�V�K���܂�) ���ϒl�Ɉ�ԋ߂��v�f���폜����ׁA��U�ǉ����Ă��܂�
		Constraints.emplace_back(NewItem);
		//!< ���ϒl�Ɉ�ԋ߂��v�f�������폜
		Constraints.erase(std::ranges::min_element(Constraints, [&](const auto& lhs, const auto& rhs) {
			return (Avg - lhs.first.PointA).LengthSq() < (Avg - rhs.first.PointA).LengthSq();
		}));
	}
}
void Physics::Manifold::RemoveExpired()
{
	constexpr auto Eps2 = 0.02f * 0.02f;
	const auto Range = std::ranges::remove_if(Constraints, [&](const auto& i) {
		const auto& Ct = i.first;
		//!< A ���[�J���ōl����
		const auto AB = Ct.RigidBodyA->ToLocal(Ct.PointB - Ct.PointA);
		const auto N = Ct.RigidBodyA->Rotation.Rotate(Ct.Normal);
		const auto PenetrateDepth = N.Dot(AB);
		return PenetrateDepth > 0.0f || (AB - N * PenetrateDepth).LengthSq() >= Eps2;
	});
	Constraints.erase(std::cbegin(Range), std::cend(Range));
}

void Physics::ManifoldCollector::Add(const Collision::Contact& Ct)
{
	//!< �������� A, B �Ԃ̃}�j�t�H�[���h�����݂��邩
	auto It = std::ranges::find_if(Manifolds, [&](const auto& i) {
		return (i.RigidBodyA == Ct.RigidBodyA && i.RigidBodyB == Ct.RigidBodyB) || (i.RigidBodyA == Ct.RigidBodyB && i.RigidBodyB == Ct.RigidBodyA);
	});
	if (std::cend(Manifolds) != It) {
		//!< �����Ȃ�Փ˂�ǉ�
		It->Add(Ct);
	}
	else {
		//!< �����łȂ���΁A�}�j�t�H�[���h(�ƏՓ�)��ǉ�
		Manifolds.emplace_back(Manifold(Ct));
	}
}

void Physics::ManifoldCollector::PreSolve(const float DeltaSec) 
{
#ifdef _DEBUG
	for (auto i = 0; i < std::size(Manifolds);++i) {
		LOG(data(std::format("Manifold[{}] Contacts = {}\n", i, std::size(Manifolds[i].Constraints))));
	}
#endif

	for (auto& i : Manifolds) {
		for (auto& j : i.Constraints) {
			j.second.PreSolve(DeltaSec);
		}
	}
}
void Physics::ManifoldCollector::Solve()
{
	for (auto& i : Manifolds) {
		for (auto& j : i.Constraints) {
			j.second.Solve();
		}
	}
}
void Physics::ManifoldCollector::PostSolve()
{
	for (auto& i : Manifolds) {
		for (auto& j : i.Constraints) {
			j.second.PostSolve();
		}
	}
}