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

void Physics::ConstraintDistance::PreSolve(const float DeltaSec)
{
	//!< ���[���h�֕ϊ� (�I�u�W�F�N�g�����������l�����ēs�x���K�v������)
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
Physics::ConstraintDistance& Physics::ConstraintDistance::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

	//!< �����ʒu�̓��[�J���ŕۑ����Ă����A(�I�u�W�F�N�g�����������l������) �s�x���[���h�ւƕϊ����Ďg��
	AnchorA = RigidBodyA->ToLocal(Anchor);
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
	const auto InvQA = QA.Inverse();

	//!< �q���W���ɐ����� U, V
	Math::Vec3 U, V;
	AxisA.GetOrtho(U, V);

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
	auto B = -Jacobian * GetVelocties(); B[0] -= Baumgarte;

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
Physics::ConstraintHinge& Physics::ConstraintHinge::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

	AnchorA = RigidBodyA->ToLocal(Anchor);
	AnchorB = RigidBodyB->ToLocal(Anchor);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	AxisA = RigidBodyA->Rotation.Inverse().Rotate(Axis);

	//!< A, B �̏����ʒu�ɂ����� QA.Inverse() * QB (�̋t�l���������߂邱�Ƃɂǂ����Ȃ�̂ŁA�����ł���Ă��܂�)
	InvInitRot = (RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation).Inverse();

	return *this;
}

void Physics::ConstraintHingeLimited::PreSolve(const float DeltaSec)
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
	const auto InvQA = QA.Inverse();

	Math::Vec3 U, V;
	AxisA.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::AxisX(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::Zero());
	const auto& PT = P; // P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	//!< ���݂̉�]
	const auto CurRot = InvQA * QB * InvInitRot;
	//!< �����ʒu����̉�]�p�x�����߁A�j�]���Ă��邩�ǂ�������
	Angle = TO_DEGREE(2.0f * std::asinf(CurRot.ToVec3().Dot(AxisA)));
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
		const auto J2 = MatA * Math::Vec4(AxisA);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(AxisA);
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
void Physics::ConstraintHingeLimited::PostSolve()
{
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	constexpr auto Limit = 20.0f;
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Limit, Limit);
	CachedLambda[1] = CachedLambda[2] = CachedLambda[3] = 0.0f;
}
Physics::ConstraintHingeLimited& Physics::ConstraintHingeLimited::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis, const float LimAng)
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

	AnchorA = RigidBodyA->ToLocal(Anchor);
	AnchorB = RigidBodyB->ToLocal(Anchor);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	AxisA = RigidBodyA->Rotation.Inverse().Rotate(Axis);

	InvInitRot = (RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation).Inverse();

	LimitAngle = LimAng;

	return *this;
}


void Physics::ConstraintBallSocket::PreSolve(const float DeltaSec)
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
	const auto InvQA = QA.Inverse();

	const auto P = Math::Mat4(Math::Vec4::Zero(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::AxisW());
	const auto& PT = P; // P.Transpose();

	const auto MatB = P * InvQA.ToLMat4() * (QB * InvInitRot).ToRMat4() * PT * 0.5f;
	const auto MatA = -MatB;

	{
		const auto J1 = Math::Vec3::Zero();
		const auto J2 = -0.5f * MatA * Math::Vec4(AxisA);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = 0.5f * MatB * Math::Vec4(AxisA);
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
}
void Physics::ConstraintBallSocketLimited::Solve()
{
}
void Physics::ConstraintBallSocketLimited::PostSolve()
{
}

void Physics::ConstraintMoverRotate::PreSolve(const float DeltaSec)
{
	RigidBodyA->AngularVelocity[1] = TO_RADIAN(30.0f);
}
void Physics::ConstraintMoverUpDown::PreSolve(const float DeltaSec)
{
	RigidBodyA->LinearVelocity[1] = std::cosf(Timer) * 4.0f;
	Timer += DeltaSec;
}

void Physics::ConstraintMotor::PreSolve(const float DeltaSec)
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
	const auto InvQA = QA.Inverse();

	const auto Axis = RigidBodyA->Rotation.Rotate(AxisA);
	Math::Vec3 U, V;
	Axis.GetOrtho(U, V);

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
		const auto J2 = MatA * Math::Vec4(Axis);
		const auto J3 = Math::Vec3::Zero();
		const auto J4 = MatB * Math::Vec4(Axis);
		Jacobian[3] = { J1.X(), J1.Y(), J1.Z(), J2.X(), J2.Y(), J2.Z(), J3.X(), J3.Y(), J3.Z(), J4.X(), J4.Y(), J4.Z() };
	}

	//!< A ���[�J���ł̑��Ή�]
	const auto RelRot = RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation * InvInitRot;
	//!< ���[���h�ł̑��Ή�]��
	const auto WRelAxis = RigidBodyA->Rotation.Rotate(RelRot.XYZ());
	const auto C = AB.Dot(AB);
	Baumgarte3 = { 0.05f * C / DeltaSec, 0.05f * U.Dot(WRelAxis) / DeltaSec, 0.05f * V.Dot(WRelAxis) / DeltaSec };
}
void Physics::ConstraintMotor::Solve() 
{
	const auto WAxis = RigidBodyA->Rotation.Rotate(AxisA) * Speed; 

	Math::Vec<12> WDT;
	WDT[3] = -WAxis[0];
	WDT[4] = -WAxis[1];
	WDT[5] = -WAxis[2];
	WDT[9] = WAxis[0];
	WDT[10] = WAxis[1];
	WDT[11] = WAxis[2];
	
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	auto B = -Jacobian * (GetVelocties() - WDT); 
	B[0] -= Baumgarte3[0];
	B[1] -= Baumgarte3[1];
	B[2] -= Baumgarte3[2];

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);
}
Physics::ConstraintMotor& Physics::ConstraintMotor::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const Math::Vec3& Anchor, const Math::Vec3& Axis, const float Spd) 
{
	RigidBodyA = const_cast<Physics::RigidBody*>(RbA);
	RigidBodyB = const_cast<Physics::RigidBody*>(RbB);

	AnchorA = RigidBodyA->ToLocal(Anchor);
	AnchorB = RigidBodyB->ToLocal(Anchor);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

	AxisA = RigidBodyA->Rotation.Inverse().Rotate(Axis);

	InvInitRot = (RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation).Inverse();

	Speed = Spd;

	return *this;
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
#if 1
	//!< N, U, V �����[���h�X�y�[�X��
	const auto N = RigidBodyA->Rotation.Rotate(Normal);
	U = RigidBodyA->Rotation.Rotate(U);
	V = RigidBodyA->Rotation.Rotate(V);
#else
	const auto& N = Normal;
	//const auto N = RigidBodyA->Rotation.Rotate(Normal);
	//U = RigidBodyA->Rotation.Rotate(U);
	//V = RigidBodyA->Rotation.Rotate(V);
#endif

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
	RigidBodyB = Ct.RigidBodyB;

	AnchorA = RigidBodyA->ToLocal(Ct.PointA);
	AnchorB = RigidBodyB->ToLocal(Ct.PointB);

	InvMass = CreateInverseMassMatrix(RigidBodyA, RigidBodyB);

#if 1
	Normal = RigidBodyA->Rotation.Rotate(Ct.Normal).Normalize();
#else
	//!< A ���[�J��
	Normal = RigidBodyA->Rotation.Inverse().Rotate(Ct.Normal).Normalize();
#endif

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
	//for (auto i = 0; i < std::size(Manifolds);++i) {
	//	LOG(data(std::format("Manifold[{}] Contacts = {}\n", i, std::size(Manifolds[i].Constraints))));
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