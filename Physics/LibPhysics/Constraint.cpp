#include "Constraint.h"
#include "RigidBody.h"
#include "Collision.h"

#include "Log.h"

//!< Vec3 の各要素を引数展開するマクロ
#define XYZ(VEC) (VEC).X(), (VEC).Y(), (VEC).Z()

//!< Lambda = -J * v / (J * M.Inverse() * J.Transpose())
//!<	Lambda : ラグランジュの未定乗数 (Lagrange multiplier) 
//!<	J : ヤコビ行列 dC/dt = J * v = 0

Physics::Constraint::MassMatrix Physics::ConstraintAnchor::CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	MassMatrix InvM;

	//!< M_A
	InvM[0][0] = InvM[1][1] = InvM[2][2] = RbA->InvMass;

	//!< I_A
	const auto InvTensorA = RbA->GetWorldInverseInertiaTensor();
	for (auto i = 0; i < 3; ++i) {
		InvM[3 + i][3 + 0] = InvTensorA[i][0];
		InvM[3 + i][3 + 1] = InvTensorA[i][1];
		InvM[3 + i][3 + 2] = InvTensorA[i][2];
	}

	//!< M_B
	InvM[6][6] = InvM[7][7] = InvM[8][8] = RbB->InvMass;

	//!< I_B
	const auto InvTensorB = RbB->GetWorldInverseInertiaTensor();
	for (auto i = 0; i < 3; ++i) {
		InvM[9 + i][9 + 0] = InvTensorB[i][0];
		InvM[9 + i][9 + 1] = InvTensorB[i][1];
		InvM[9 + i][9 + 2] = InvTensorB[i][2];
	}

	return InvM;
}
Physics::Constraint::Velocities Physics::ConstraintAnchor::CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	return { XYZ(RbA->LinearVelocity), XYZ(RbA->AngularVelocity), XYZ(RbB->LinearVelocity), XYZ(RbB->AngularVelocity) };
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
Physics::ConstraintAnchor& Physics::ConstraintAnchor::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor)
{
	Init(RbA, RbB);

	//!< 初期位置はローカルで保存しておき (オブジェクトが動く事を考慮して) 都度ワールドへと変換して使う
	LAnchorA = RigidBodyA->ToLocalPos(WAnchor);
	LAnchorB = RigidBodyB->ToLocalPos(WAnchor);

	return *this;
}

Physics::ConstraintAnchorAxis& Physics::ConstraintAnchorAxis::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis)
{
	Super::Init(RbA, RbB, WAnchor);

	LAxisA = RigidBodyA->ToLocalDir(WAxis);

	//!< A, B の初期位置における (QA.Inverse() * QB).Inverse()
	//!< ある瞬間の回転を CurRot = QA.Inverse() * QB * InitRot.Inverse() とすると、
	//!< 初期位置からの回転角は Theta = 2.0f * std::asinf(CurRot.Dot(Hinge)) で求まる
	InvInitRot = (RigidBodyA->Rotation.Inverse() * RigidBodyB->Rotation).Inverse();

	return *this;
}

void Physics::ConstraintDistance::PreSolve(const float DeltaSec)
{
	//!< ワールドへ変換 (オブジェクトが動く事を考慮して都度やる必要がある)
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		//!< それぞれの剛体における、重心からアンカー位置 (ワールド) へのベクトル
		const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass();

		//!< 距離コンストレイント ヤコビ行列を作成
		//!< J = (2 * (WAnchorA - WAnchorB), 2 * RA.Cross(WAnchorA - WAnchorB), 2 * (WAnchorB - WAnchorA), 2 * RB.Cross(WAnchorB - WAnchorA))
		//!< J = (2 * -AB, 2 * RA.Cross(-AB), 2 * AB, 2 * RB.Cross(AB))
		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

#pragma region WARM_STARTING
	//!< 前のフレームの力積を、今フレームの計算前に適用することで、少ないイテレーションで安定状態へ導く
	ApplyImpulse(Jacobian.Transpose() * CachedLambda);
#pragma endregion

#pragma region BAUMGARTE_STABILIZATION
	//!< 適正な位置へ戻すような力を適用する事で修正を試みる
	//!< C は侵された制約距離 (デルタ時間では C / DeltaSec)、許容誤差 (0.01f) を超えていれば有効 (振動防止)
	const auto C = (std::max)(AB.Dot(AB) - 0.01f, 0.0f);
	//!< 完全に修正するとエネルギー与え過ぎで不安定になるので Beta (== 0.05f) で調整
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
#pragma endregion
}
void Physics::ConstraintDistance::Solve()  
{
	//!< J * InvMass * Transpose(J) * Impulse = -J * v
	//!< ここで A = J * InvMass * Transpose(J), B = -J * v と置くと A * Impulse = B
	//!< これをガウスザイデル法で解き、Impulse を求める
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties()
#pragma region BAUMGARTE_STABILIZATION
		//!< 侵された制約を修正するような速度を与える
		- LinAlg::Vec<1>(Baumgarte);
#pragma endregion

	//!< A * Lambda = B となる Lambda を求める
	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);

#pragma region WARM_STARTING
	//!< ウォームスタート用に今回の力積を蓄積しておく
	CachedLambda += Lambda;
#pragma endregion
}
void Physics::ConstraintDistance::PostSolve() 
{
	if (std::isnan(CachedLambda[0])) { CachedLambda[0] = 0.0f; }

#pragma region WARM_STARTING
	//!< 短時間に大きな力積が加わった場合に不安定になるのを防ぐために、リーズナブルな範囲に制限する
	constexpr auto Eps = (std::numeric_limits<float>::epsilon)();
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Eps, Eps);
#pragma endregion
}

void Physics::ConstraintHinge::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass();
		//!< 距離コンストレイント ヤコビ行列
		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	//!< 四元数コンストレイント
	//!< J_1 = (0, A * u, 0, B * u)
	//!< J_2 = (0, A * v, 0, B * v)
	//!< ただし A = - 1/2 * L(conj(q_1)) * R(q_2 * conj(q_0)) * P^t, B = -A
	{
		const auto& QA = RigidBodyA->Rotation;
		const auto& QB = RigidBodyB->Rotation;
		const auto InvQA = QA.Inverse();

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		//!< 本来は P の転置行列を求める (ここでは意味がないのでやらない)
		const auto& PT = P; //P.Transpose();

		const auto MatB = P * InvQA.ToLeftMat4() * (QB * InvInitRot).ToRightMat4() * PT * 0.5f;
		const auto MatA = -MatB;

		//!< ヒンジ軸に垂直な U, V 軸
		LinAlg::Vec3 U, V;
		LAxisA.GetOrtho(U, V);
		//!< U
		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = MatA * LinAlg::Vec4(U);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = MatB * LinAlg::Vec4(U);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
		//!< V
		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = MatA * LinAlg::Vec4(V);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = MatB * LinAlg::Vec4(V);
			Jacobian[2] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintHinge::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<3>(Baumgarte, 0.0f, 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintHinge::PostSolve()
{
	if (std::isnan(CachedLambda[0])) { CachedLambda[0] = 0.0f; }
	if (std::isnan(CachedLambda[1])) { CachedLambda[1] = 0.0f; }
	if (std::isnan(CachedLambda[2])) { CachedLambda[2] = 0.0f; }

	constexpr auto Limit = 20.0f;
	CachedLambda = { 
		(std::clamp)(CachedLambda[0], -Limit, Limit), 
		(std::clamp)(CachedLambda[1], -Limit, Limit), 
		(std::clamp)(CachedLambda[2], -Limit, Limit) 
	};
}

void Physics::ConstraintHingeLimited::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass();
		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	//!< 四元数コンストレイント (角度制限付き)
	{
		const auto& QA = RigidBodyA->Rotation;
		const auto& QB = RigidBodyB->Rotation;
		const auto InvQA = QA.Inverse();

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		const auto& PT = P;

		const auto MatA = -P * InvQA.ToLeftMat4() * (QB * InvInitRot).ToRightMat4() * PT * 0.5f;
		const auto MatB = -MatA;

		LinAlg::Vec3 U, V;
		LAxisA.GetOrtho(U, V);
		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = MatA * LinAlg::Vec4(U);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = MatB * LinAlg::Vec4(U);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = MatA * LinAlg::Vec4(V);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = MatB * LinAlg::Vec4(V);
			Jacobian[2] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}

		//!< 角度制限 (角度違反している場合に追加の制約)
		{
			//!< 現在の回転
			const auto CurRot = InvQA * QB * InvInitRot;
			//!< 初期位置からの回転角度を求め、違反しているかどうか
			Angle = LinAlg::ToDegree(2.0f * std::asinf(CurRot.ToVec3().Dot(LAxisA)));
			IsAngleViolated = std::fabsf(Angle) > LimitAngle;
			if (IsAngleViolated) {
				//!< 角度制限自体は Solve() で行う
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(LAxisA);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(LAxisA);
				Jacobian[3] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}
			else {
				Jacobian[3].ToZero();
			}
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintHingeLimited::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<4>(Baumgarte, 0.0f, 0.0f, 0.0f);

	auto Lambda = GaussSiedel(A, B);

	//!< 角度制限によるトルクの制限 (剛体を許容された領域に押すような回復的トルク)
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
	if (std::isnan(CachedLambda[0])) { CachedLambda[0] = 0.0f; }

	constexpr auto Limit = 20.0f;
	CachedLambda = { (std::clamp)(CachedLambda[0], -Limit, Limit), 0.0f, 0.0f, 0.0f };
}

void Physics::ConstraintBallSocket::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass();

		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	{
		const auto& QA = RigidBodyA->Rotation;
		const auto& QB = RigidBodyB->Rotation;
		const auto InvQA = QA.Inverse();

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		const auto& PT = P;

		const auto MatB = P * InvQA.ToLeftMat4() * (QB * InvInitRot).ToRightMat4() * PT * 0.5f;
		const auto MatA = -MatB;

		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = -0.5f * MatA * LinAlg::Vec4(LAxisA);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = 0.5f * MatB * LinAlg::Vec4(LAxisA);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintBallSocket::Solve() 
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<2>(Baumgarte, 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintBallSocket::PostSolve()
{
	if (std::isnan(CachedLambda[0])) { CachedLambda[0] = 0.0f; }
	if (std::isnan(CachedLambda[1])) { CachedLambda[1] = 0.0f; }

	constexpr auto Limit = 20.0f;
	CachedLambda = {
		(std::clamp)(CachedLambda[0], -Limit, Limit),
		(std::clamp)(CachedLambda[1], -Limit, Limit)
	};
}

void Physics::ConstraintBallSocketLimited::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass();

		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	{
		const auto& QA = RigidBodyA->Rotation;
		const auto& QB = RigidBodyB->Rotation;
		const auto InvQA = QA.Inverse();

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		const auto& PT = P;

		const auto MatB = P * InvQA.ToLeftMat4() * (QB * InvInitRot).ToRightMat4() * PT * 0.5f;
		const auto MatA = -MatB;

		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = -0.5f * MatA * LinAlg::Vec4(LAxisA);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = 0.5f * MatB * LinAlg::Vec4(LAxisA);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}

		{
			LinAlg::Vec3 U, V;
			LAxisA.GetOrtho(U, V);

			const auto CurRot = InvQA * QB * InvInitRot;
			Angles = {
				LinAlg::ToDegree(2.0f * std::asinf(CurRot.ToVec3().Dot(U))),
				LinAlg::ToDegree(2.0f * std::asinf(CurRot.ToVec3().Dot(V)))
			};
			IsAngleViolated = {
				std::fabsf(Angles[0]) > LimitAngles[0],
				std::fabsf(Angles[1]) > LimitAngles[1]
			};

			if (IsAngleViolated[0]) {
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(U);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(U);
				Jacobian[2] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}
			else {
				Jacobian[2].ToZero();
			}
			if (IsAngleViolated[1]) {
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(V);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(V);
				Jacobian[3] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}
			else {
				Jacobian[3].ToZero();
			}
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintBallSocketLimited::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<4>(Baumgarte, 0.0f, 0.0f, 0.0f);

	auto Lambda = GaussSiedel(A, B);

	for (auto i = 0; i < std::size(IsAngleViolated); ++i) {
		if (IsAngleViolated[i]) {
			const auto LmdIdx = i + 2;
			if (Angles[i] > 0.0f) {
				Lambda[LmdIdx] = (std::min)(Lambda[LmdIdx], 0.0f);
			}
			if (Angles[i] < 0.0f) {
				Lambda[LmdIdx] = (std::max)(Lambda[LmdIdx], 0.0f);
			}
		}
	}

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintBallSocketLimited::PostSolve()
{
	if (std::isnan(CachedLambda[0])) { CachedLambda[0] = 0.0f; }

	constexpr auto Limit = 20.0f;
	CachedLambda = { (std::clamp)(CachedLambda[0], -Limit, Limit), 0.0f, 0.0f, 0.0f };
}

void Physics::ConstraintMotor::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
		const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass();

		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -J1;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
	}

	{
		const auto& QA = RigidBodyA->Rotation;
		const auto& QB = RigidBodyB->Rotation;
		const auto InvQA = QA.Inverse();
		const auto QBInvInitRot = QB * InvInitRot;

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		const auto& PT = P;

		const auto MatB = P * InvQA.ToLeftMat4() * QBInvInitRot.ToRightMat4() * PT * 0.5f;
		const auto MatA = -MatB;

		{
			//!< モーター軸 (W)
			const auto W = RigidBodyA->ToWorldDir(LAxisA);

			//!< モーター軸に垂直な U, V 軸
			LinAlg::Vec3 U, V;
			W.GetOrtho(U, V);
			{
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(U);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(U);
				Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}
			{
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(V);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(V);
				Jacobian[2] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}
			{
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(W);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(W);
				Jacobian[3] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}

			//!< A ローカルでの相対回転
			const auto RelRot = InvQA * QBInvInitRot;
			//!< ワールドでの相対回転軸
			const auto WRelAxis = RigidBodyA->ToWorldDir(RelRot.Imag());
			const auto C = AB.Dot(AB);
			constexpr auto Beta = 0.05f;
			Baumgarte = { C, U.Dot(WRelAxis), V.Dot(WRelAxis) } * Beta / DeltaSec;
		}
	}
}
void Physics::ConstraintMotor::Solve() 
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	
	//!< 与えたい速度 (両者が相対的に一定速度で回転)
	LinAlg::Vec<12> Vel;
	{
		const auto WAxis = RigidBodyA->ToWorldDir(LAxisA);
		const auto WASpd = WAxis * Speed;
		//!< A の角速度
		Vel[3] = -WASpd[0];
		Vel[4] = -WASpd[1];
		Vel[5] = -WASpd[2];
		//!< B の角速度
		Vel[9] = WASpd[0];
		Vel[10] = WASpd[1];
		Vel[11] = WASpd[2];
	}
	//!< 減算してから適用することで、ソルバーは騙されて与えたい相対速度になるような力積を作り出す
	const auto B = -Jacobian * (GetVelocties() - Vel) - LinAlg::Vec<4>(Baumgarte.X(), Baumgarte.Y(), Baumgarte.Z(), 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(JT * Lambda);
}

void Physics::ConstraintMoverUpDown::PreSolve(const float DeltaSec)
{
	//!< Y 軸方向に半径 4 のコサイン軌道
	constexpr auto Radius = 4.0f;
	RigidBodyA->LinearVelocity[1] = std::cosf(Timer) * Radius;
	Timer += DeltaSec;
}
void Physics::ConstraintMoverRotateX::PreSolve(const float DeltaSec) 
{
	RigidBodyA->AngularVelocity[0] = LinAlg::ToRadian(60.0f);
}
void Physics::ConstraintMoverRotateY::PreSolve(const float DeltaSec)
{
	RigidBodyA->AngularVelocity[1] = LinAlg::ToRadian(60.0f);
}
void Physics::ConstraintMoverRotateZ::PreSolve(const float DeltaSec)
{
	RigidBodyA->AngularVelocity[2] = LinAlg::ToRadian(60.0f);
}
Physics::ConstraintPenetration& Physics::ConstraintPenetration::Init(const Collision::Contact& Ct)
{
	Super::Init(Ct.RigidBodyA, Ct.RigidBodyB);

	LAnchorA = Ct.LPointA;
	LAnchorB = Ct.LPointB;

	//!< A ローカル
	LNormal = RigidBodyA->ToLocalDir(Ct.WNormal).Normalize();

	Friction = RigidBodyA->Friction * RigidBodyB->Friction;

	return *this;
}
void Physics::ConstraintPenetration::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	//!< 法線をワールドスペースへ
	const auto WNormal = RigidBodyA->ToWorldDir(LNormal);
	const auto RA = WAnchorA - RigidBodyA->GetWorldCenterOfMass();
	const auto RB = WAnchorB - RigidBodyB->GetWorldCenterOfMass(); 
	{
		//!< J = (-N, RA x N, N, RB x N)
		const auto J1 = -WNormal;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -J1;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
	}

	//!< 摩擦がある場合、接線方向のコンストレイントを考慮
	if (Friction > 0.0f) {
		//!< 直交ベクトル U, V を求める
		LinAlg::Vec3 U, V;
		WNormal.GetOrtho(U, V);
		//!< U
		{
			const auto J1 = -U;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
		//!< V
		{
			const auto J1 = -V;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[2] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	const auto C = (std::min)(AB.Dot(WNormal) + 0.02f, 0.0f);
	constexpr auto Beta = 0.25f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintPenetration::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<3>(Baumgarte, 0.0f, 0.0f);

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

Physics::Manifold::Manifold(const Collision::Contact& Ct) 
	: RigidBodyA(Ct.RigidBodyA), RigidBodyB(Ct.RigidBodyB) 
{
	Add(Ct); 
}
void Physics::Manifold::Add(const Collision::Contact& CtOrig)
{
	//!< 剛体 A, B の順序が食い違っている場合は辻褄を合わせる
	auto Ct = CtOrig;
	if (RigidBodyA != CtOrig.RigidBodyA) { Ct.Swap(); }

	//!< 法線を反転する必要がある #TODO 何故？ P43
	Ct.WNormal *= -1.0f; 

	//!< 既存とほぼ同じ接触位置の場合は何もしない (既存を残し新規を破棄する、古い方が収束している傾向がある為)
	constexpr auto Eps2 = 0.02f * 0.02f;
	if (std::ranges::any_of(Constraints,
		[&](const auto& i) {
			return (i.first.WPointA - Ct.WPointA).LengthSq() < Eps2 || (i.first.WPointB - Ct.WPointB).LengthSq() < Eps2;
		})) {
		return;
	}

	//!< 新規要素 (接触点と制約のペア)
	const auto NewItem = ContactAndConstraint({ Ct, ConstraintPenetration(Ct) });
	if (std::size(Constraints) < MaxContacts) {
		//!< 空きがある場合は追加
		Constraints.emplace_back(NewItem);
	}
	else {
		//!< (新規を) 一旦追加してしまう
		Constraints.emplace_back(NewItem);
		
		//!< (新規を含めた) 平均値を求める
		const auto Avg = (std::accumulate(std::cbegin(Constraints), std::cend(Constraints), LinAlg::Vec3::Zero(),
			[](const auto& Acc, const auto& i) {
				return Acc + i.first.WPointA;
			})) / static_cast<float>(std::size(Constraints));

		//!< 平均値に一番近い要素を取り除く (大きく貫通しているであろう要素ほど残ることになる)
		Constraints.erase(std::ranges::min_element(Constraints,
			[&](const auto& lhs, const auto& rhs) {
				return (Avg - lhs.first.WPointA).LengthSq() < (Avg - rhs.first.WPointA).LengthSq();
			}));
	}
}
void Physics::Manifold::RemoveExpired()
{
	constexpr auto Eps2 = 0.02f * 0.02f;
	const auto Range = std::ranges::remove_if(Constraints, 
		[&](const auto& rhs) {
			const auto& Ct = rhs.first;
			//!< 衝突時のローカル位置を、現在のトランスフォームで変換 (衝突時のローカル位置 LPointA, LPointB を覚えておく必要がある)
			const auto AB = Ct.RigidBodyB->ToWorldPos(Ct.LPointB) - Ct.RigidBodyA->ToWorldPos(Ct.LPointA);
			const auto& WNrm = Ct.WNormal;
			const auto PenetrateDepth = WNrm.Dot(AB);
			//!< AB と A の法線が同じ向き (めり込んでいない) なら除外
			if (PenetrateDepth > 0.0f) {
				return true;
			}
			//!< 接線方向の距離が一定以上ズレたなら除外
			const auto ABNrm = WNrm * PenetrateDepth;
			const auto ABTan = (AB - ABNrm);
			return ABTan.LengthSq() >= Eps2;
		});
	Constraints.erase(std::cbegin(Range), std::cend(Range));
}

//!< 衝突情報を追加
void Physics::ManifoldCollector::Add(const Collision::Contact& Ct)
{
	//!< 剛体 A, B 間のマニフォールドが既に存在するか
	auto It = std::ranges::find_if(Manifolds,
		[&](const auto& i) {
			return (i.RigidBodyA == Ct.RigidBodyA && i.RigidBodyB == Ct.RigidBodyB) || (i.RigidBodyA == Ct.RigidBodyB && i.RigidBodyB == Ct.RigidBodyA);
		});
	if (std::cend(Manifolds) != It) {
		//!< (既存の) マニフォールドへ衝突情報を追加
		It->Add(Ct);
	}
	else {
		//!< マニフォールドと衝突情報を追加
		Manifolds.emplace_back(Manifold(Ct));
	}
}

void Physics::ManifoldCollector::PreSolve(const float DeltaSec) 
{
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

#undef XYZ