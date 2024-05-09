#include "Constraint.h"
#include "RigidBody.h"
#include "Collision.h"

#include "Log.h"

//!< Lambda = -J * v / (J * M.Inverse() * J.Transpose())
//!<	Lambda : ���O�����W���̖���搔 (Lagrange multiplier) 
//!<	J : ���R�r�s�� dC/dt = J * v = 0

Physics::Constraint::MassMatrix Physics::ConstraintAnchor::CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	MassMatrix InvM;

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
Physics::Constraint::Velocities Physics::ConstraintAnchor::CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	return { RbA->LinearVelocity.X(), RbA->LinearVelocity.Y(), RbA->LinearVelocity.Z(),
	RbA->AngularVelocity.X(), RbA->AngularVelocity.Y(), RbA->AngularVelocity.Z(),
	RbB->LinearVelocity.X(), RbB->LinearVelocity.Y(), RbB->LinearVelocity.Z(),
	RbB->AngularVelocity.X(), RbB->AngularVelocity.Y(), RbB->AngularVelocity.Z() };
}

void Physics::ConstraintAnchor::ApplyImpulse(const Physics::Constraint::Velocities& Impulse)
{
	RigidBodyA->ApplyLinearImpulse({ Impulse[0], Impulse[1], Impulse[2] });
	RigidBodyA->ApplyAngularImpulse({ Impulse[3], Impulse[4], Impulse[5] });

	RigidBodyB->ApplyLinearImpulse({ Impulse[6], Impulse[7], Impulse[8] });
	RigidBodyB->ApplyAngularImpulse({ Impulse[9], Impulse[10], Impulse[11] });
}
Physics::ConstraintAnchor& Physics::ConstraintAnchor::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	return *this;
}
Physics::ConstraintAnchor& Physics::ConstraintAnchor::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor)
{
	Init(RbA, RbB);

	//!< �����ʒu�̓��[�J���ŕۑ����Ă����A(�I�u�W�F�N�g�����������l������) �s�x���[���h�ւƕϊ����Ďg��
	LAnchorA = RigidBodyA->ToLocalPos(WAnchor);
	LAnchorB = RigidBodyB->ToLocalPos(WAnchor);

	return *this;
}

Physics::ConstraintAnchorAxis& Physics::ConstraintAnchorAxis::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& WAnchor, const Math::Vec3& WAxis)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

	LAnchorA = RigidBodyA->ToLocalPos(WAnchor);
	LAnchorB = RigidBodyB->ToLocalPos(WAnchor);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	LAxisA = RigidBodyA->ToLocalDir(WAxis);

	//!< A, B �̏����ʒu�ɂ����� QA.Inverse() * QB (�̋t�l���������߂邱�Ƃɂǂ����Ȃ�̂ŁA�����ł���Ă��܂�)
	InvInitRot = (RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation).Inverse();

	return *this;
}

void Physics::ConstraintDistance::PreSolve(const float DeltaSec)
{
	//!< ���[���h�֕ϊ� (�I�u�W�F�N�g�����������l�����ēs�x���K�v������)
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< ���R�r�s����쐬
	{
		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -BA * 2.0f;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	//!< �O�̃t���[���̗� (CachedLambda) �����t���[���̌v�Z�O�ɓK�p���邱�ƂŁA���Ȃ��t���[���ň����Ԃ֎����Ă��� (Warm starting)
	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	//!< �K���Ȉʒu�֖߂��悤�ȗ͂�K�p���鎖�ňʒu�h���t�g���C�� (Baumgarte stabilization)
	const auto C = (std::max)(AB.Dot(AB) - 0.01f, 0.0f);
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

void Physics::ConstraintHinge::PreSolve(const float DeltaSec) 
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< ����
	{
		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -BA * 2.0f;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvQA = QA.Inverse();

	//!< �q���W���ɐ����� U, V
	Math::Vec3 U, V;
	LAxisA.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::AxisX(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::Zero());
	//!< (�����ł�) �]�u����Ӗ�������
	const auto& PT = P; //P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	//!< U
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(U);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(U);
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	//!< V
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(V);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(V);
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
	const auto B = -Jacobian * GetVelocties() - Math::Vec<3>(Baumgarte, 0.0f, 0.0f);

	const auto Lambda = GaussSiedel(A, B);

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

void Physics::ConstraintHingeLimited::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	{
		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -BA * 2.0f;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvQA = QA.Inverse();

	Math::Vec3 U, V;
	LAxisA.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::AxisX(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::Zero());
	const auto& PT = P; // P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	//!< ���݂̉�]
	const auto CurRot = InvQA * QB * InvInitRot;
	//!< �����ʒu����̉�]�p�x�����߁A�j�]���Ă��邩�ǂ�������
	Angle = TO_DEGREE(2.0f * std::asinf(CurRot.ToVec3().Dot(LAxisA)));
	IsAngleViolated = std::fabsf(Angle) > LimitAngle;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(U);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(U);
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(V);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(V);
		Jacobian[2] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	//!< �p�x����
	if (IsAngleViolated) {
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(LAxisA);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(LAxisA);
		Jacobian[3] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	else {
		Jacobian[3].ToZero();
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	Baumgarte = 0.05f * C / DeltaSec;
}
void Physics::ConstraintHingeLimited::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - Math::Vec<4>(Baumgarte, 0.0f, 0.0f, 0.0f);

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
void Physics::ConstraintHingeLimited::PostSolve()
{
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	constexpr auto Limit = 20.0f;
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Limit, Limit);
	CachedLambda[1] = CachedLambda[2] = CachedLambda[3] = 0.0f;
}

void Physics::ConstraintBallSocket::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	{
		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -BA * 2.0f;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvQA = QA.Inverse();

	const auto P = Math::Mat4(Math::Vec4::AxisX(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::Zero());
	const auto& PT = P; // P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = -0.5f * MatA * Math::Vec4(LAxisA);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = 0.5f * MatB * Math::Vec4(LAxisA);
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
	const auto B = -Jacobian * GetVelocties() - Math::Vec<2>(Baumgarte, 0.0f);

	const auto Lambda = GaussSiedel(A, B);

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

void Physics::ConstraintBallSocketLimited::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

	const auto AB = WAnchorB - WAnchorA;
	const auto BA = WAnchorA - WAnchorB;
	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	{
		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -BA * 2.0f;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	const auto& QA = RigidBodyA->Rotation;
	const auto& QB = RigidBodyB->Rotation;
	const auto InvQA = QA.Inverse();

	const auto P = Math::Mat4(Math::Vec4::AxisX(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::Zero());
	const auto& PT = P; // P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = -0.5f * MatA * Math::Vec4(LAxisA);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = 0.5f * MatB * Math::Vec4(LAxisA);
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	Math::Vec3 U, V;
	LAxisA.GetOrtho(U, V);

	const auto CurRot = InvQA * QB * InvInitRot;
	AngleU = TO_DEGREE(2.0f * std::asinf(CurRot.ToVec3().Dot(U)));
	IsAngleUViolated = std::fabsf(AngleU) > LimitAngleU;
	AngleV = TO_DEGREE(2.0f * std::asinf(CurRot.ToVec3().Dot(V)));
	IsAngleVViolated = std::fabsf(AngleV) > LimitAngleV;

	if (IsAngleUViolated) {
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(U);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(U);
		Jacobian[2] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	else {
		Jacobian[2].ToZero();
	}
	if (IsAngleVViolated) {
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(V);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(V);
		Jacobian[3] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	else {
		Jacobian[3].ToZero();
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	Baumgarte = 0.05f * C / DeltaSec;
}
void Physics::ConstraintBallSocketLimited::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - Math::Vec<4>(Baumgarte, 0.0f, 0.0f, 0.0f);

	auto Lambda = GaussSiedel(A, B);

	if (IsAngleUViolated) {
		if (AngleU > 0.0f) {
			Lambda[2] = (std::min)(Lambda[2], 0.0f);
		}
		if (AngleU < 0.0f) {
			Lambda[2] = (std::max)(Lambda[2], 0.0f);
		}
	}
	if (IsAngleVViolated) {
		if (AngleV > 0.0f) {
			Lambda[3] = (std::min)(Lambda[3], 0.0f);
		}
		if (AngleU < 0.0f) {
			Lambda[3] = (std::max)(Lambda[3], 0.0f);
		}
	}

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintBallSocketLimited::PostSolve()
{
	constexpr auto Limit = 20.0f;
	for (auto i = 0; i < CachedLambda.Size(); ++i) {
		if (i > 0) CachedLambda[i] = 0.0f;
		if (CachedLambda[i] * 0.0f != CachedLambda[i] * 0.0f) { CachedLambda[i] = 0.0f; }
		CachedLambda[i] = (std::clamp)(CachedLambda[i], -Limit, Limit);
	}
}

void Physics::ConstraintMoverRotate::PreSolve(const float DeltaSec)
{
	RigidBodyA->AngularVelocity[1] = TO_RADIAN(60.0f);
}
void Physics::ConstraintMoverUpDown::PreSolve(const float DeltaSec)
{
	RigidBodyA->LinearVelocity[1] = std::cosf(Timer) * 4.0f;
	Timer += DeltaSec;
}

void Physics::ConstraintMotor::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

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
	const auto InvQA = QA.Inverse();

	const auto WAxis = RigidBodyA->ToWorldDir(LAxisA);
	Math::Vec3 U, V;
	WAxis.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::AxisX(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::Zero());
	const auto& PT = P; // P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(U);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(U);
		Jacobian[1] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(V);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(V);
		Jacobian[2] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}
	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = MatA * Math::Vec4(WAxis);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(WAxis);
		Jacobian[3] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	//!< A ���[�J���ł̑��Ή�]
	const auto RelRot = RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation * InvInitRot;
	//!< ���[���h�ł̑��Ή�]��
	const auto WRelAxis = RigidBodyA->ToWorldDir(RelRot.XYZ());
	const auto C = AB.Dot(AB);
	Baumgarte = { 0.05f * C / DeltaSec, 0.05f * U.Dot(WRelAxis) / DeltaSec, 0.05f * V.Dot(WRelAxis) / DeltaSec };
}
void Physics::ConstraintMotor::Solve() 
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	Math::Vec<12> Vel;
	{
		const auto WAxis = RigidBodyA->ToWorldDir(LAxisA) * Speed;
		Vel[3] = -WAxis[0];
		Vel[4] = -WAxis[1];
		Vel[5] = -WAxis[2];
		Vel[9] = WAxis[0];
		Vel[10] = WAxis[1];
		Vel[11] = WAxis[2];
	}
	//!< �^���������x�����Z���Ă���K�p���邱�ƂŁA�ړI�̑��x�ɂȂ�悤�ȗ͐ς�^����悤�ɂȂ�
	const auto B = -Jacobian * (GetVelocties() - Vel) - Math::Vec<4>(Baumgarte.X(), Baumgarte.Y(), Baumgarte.Z(), 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);
}

Physics::ConstraintPenetration& Physics::ConstraintPenetration::Init(const Collision::Contact& Ct)
{
	Super::Init(Ct.RigidBodyA, Ct.RigidBodyB);

	LAnchorA = Ct.LPointA;
	LAnchorB = Ct.LPointB;

	//!< A ���[�J��
	LNormal = RigidBodyA->ToLocalDir(Ct.WNormal).Normalize();

	Friction = RigidBodyA->Friction * RigidBodyB->Friction;

	return *this;
}
void Physics::ConstraintPenetration::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);

	const auto RA = WAnchorA - RigidBodyA->GetWorldSpaceCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldSpaceCenterOfMass();

	//!< �@�������[���h�X�y�[�X��
	const auto WNormal = RigidBodyA->ToWorldDir(LNormal);
	Math::Vec3 U, V;
	WNormal.GetOrtho(U, V);

	const auto J1 = -WNormal;
	const auto J2 = RA.Cross(J1);
	const auto J3 = WNormal;
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

	const auto C = (std::min)((WAnchorB - WAnchorA).Dot(WNormal) + 0.02f, 0.0f);
	Baumgarte = 0.25f * C / DeltaSec;
}
void Physics::ConstraintPenetration::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - Math::Vec<3>(Baumgarte, 0.0f, 0.0f);

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

Physics::Manifold::Manifold(const Collision::Contact& Ct) : RigidBodyA(Ct.RigidBodyA), RigidBodyB(Ct.RigidBodyB) { Add(Ct); }
void Physics::Manifold::Add(const Collision::Contact& CtOrig)
{
	//!< ���� A, B �̏������H������Ă���ꍇ�͒�������킹��
	auto Ct = CtOrig;
	if (RigidBodyA != CtOrig.RigidBodyA) { Ct.Swap(); }

	//!< �����Ƃقړ����ڐG�ʒu�̏ꍇ�͉������Ȃ�
	constexpr auto Eps2 = 0.02f * 0.02f;
	if (std::ranges::any_of(Constraints, [&](const auto& i) {
		return (i.first.RigidBodyA->ToWorldPos(i.first.LPointA) - Ct.RigidBodyA->ToWorldPos(Ct.LPointA)).LengthSq() < Eps2 ||
			(i.first.RigidBodyB->ToWorldPos(i.first.LPointB) - Ct.RigidBodyB->ToWorldPos(Ct.LPointB)).LengthSq() < Eps2;
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
			return Acc + i.first.LPointA; 
		}) + Ct.LPointA) / 5.0f;

		//!< (�V�K���܂�) ���ϒl�Ɉ�ԋ߂��v�f���폜����ׁA��U�ǉ����Ă��܂�
		Constraints.emplace_back(NewItem);
		//!< ���ϒl�Ɉ�ԋ߂��v�f�������폜
		Constraints.erase(std::ranges::min_element(Constraints, [&](const auto& lhs, const auto& rhs) {
			return (Avg - lhs.first.LPointA).LengthSq() < (Avg - rhs.first.LPointA).LengthSq();
		}));
	}
}
void Physics::Manifold::RemoveExpired()
{
	constexpr auto Eps2 = 0.02f * 0.02f;
	const auto Range = std::ranges::remove_if(Constraints, [&](const auto& i) {
		const auto& Ct = i.first;
		const auto AB = Ct.RigidBodyB->ToWorldPos(Ct.LPointB) - Ct.RigidBodyA->ToWorldPos(Ct.LPointA);
		const auto WNormal = Ct.RigidBodyA->ToWorldDir(Ct.WNormal);
		const auto PenetrateDepth = WNormal.Dot(AB);
		return PenetrateDepth > 0.0f || (AB - WNormal * PenetrateDepth).LengthSq() >= Eps2;
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
	//for (auto i = 0; i < std::size(Manifolds);++i) {
	//	LOG(std::data(std::format("Manifold[{}] Contacts = {}\n", i, std::size(Manifolds[i].Constraints))));
	//}
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