#include "Constraint.h"
#include "RigidBody.h"
#include "Collision.h"

#include "Log.h"

//!< Vec3 の各要素を引数展開するマクロ
#define XYZ(VEC) (VEC).X(), (VEC).Y(), (VEC).Z()

/*
* C(q_1, q_2,\ldots,q_n) = 0
* 
* 一階微分をすると
* \dot{C} = \frac{dC}{dq} \frac{dq}{dt} = \frac{dC}{dq} v = 0
* \frac{dC}{dq} はヤコビ行列 (J) として知られ、勾配を表す
* \dot{C} = J v = 0
* 
* 二階微分をすると
* \ddot{C} = \dot{J} v + J a = 0 ... (1)
* 
* ここでニュートンの運動方程式から
* F = M a (ここでは n 個の剛体があるので M は質量行列)
* a = M^{-1} F
* 
* これを (1) に代入すると
* \ddot{C} = \dot{J} v + J M^{-1} F = 0
* J M^{-1} F = -\dot{J} v ... (2)
* 
* ダランベールの原理から、制約力は仕事をしない
* 制約力は速度に垂直になる
* F v = 0
* 
* 制約力は勾配 J に並行の為
* F は J の定数倍で表され
* F = J^T \lambda_f
* ここで \lambda_f はラグランジュの未定乗数 (ベクトル量)
*
* (2) に代入して \lambda_f について解くと
* J M^{-1} J^T \lambda_f = -\dot{J} v
* \lambda_f = -\dot{J} v / J M^{-1} J^T
* 
* 力積ベース (力の積分) のラグランジュの未定乗数は以下のようになる
* J M^{-1} J^T \lambda_i = -J v
* \lambda_i = -J v / J M^{-1} J^T
*/
Physics::Constraint::MassMatrix Physics::ConstraintAnchor::CreateInverseMassMatrix(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	MassMatrix InvM;

	//!< M_A
	InvM[0][0] = InvM[1][1] = InvM[2][2] = RbA->GetMass_Inverse();

	//!< I_A
	const auto InvTensorA = RbA->GetInertiaTensor_Inverse_World();
	for (auto i = 0; i < 3; ++i) {
		InvM[3 + i][3 + 0] = InvTensorA[i][0];
		InvM[3 + i][3 + 1] = InvTensorA[i][1];
		InvM[3 + i][3 + 2] = InvTensorA[i][2];
	}

	//!< M_B
	InvM[6][6] = InvM[7][7] = InvM[8][8] = RbB->GetMass_Inverse();

	//!< I_B
	const auto InvTensorB = RbB->GetInertiaTensor_Inverse_World();
	for (auto i = 0; i < 3; ++i) {
		InvM[9 + i][9 + 0] = InvTensorB[i][0];
		InvM[9 + i][9 + 1] = InvTensorB[i][1];
		InvM[9 + i][9 + 2] = InvTensorB[i][2];
	}

	return InvM;
}
Physics::Constraint::Velocities Physics::ConstraintAnchor::CreateVelocties(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB)
{
	return { XYZ(RbA->GetVelocity_Linear()), XYZ(RbA->GetVelocity_Angular()), XYZ(RbB->GetVelocity_Linear()), XYZ(RbB->GetVelocity_Angular()) };
}

void Physics::ConstraintAnchor::ApplyImpulse(const Physics::Constraint::Velocities& Impulse)
{
	RigidBodyA->ApplyImpulse_Linear({ Impulse[0], Impulse[1], Impulse[2] });
	RigidBodyA->ApplyImpulse_Angular({ Impulse[3], Impulse[4], Impulse[5] });

	RigidBodyB->ApplyImpulse_Linear({ Impulse[6], Impulse[7], Impulse[8] });
	RigidBodyB->ApplyImpulse_Angular({ Impulse[9], Impulse[10], Impulse[11] });
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

/*
* a を b の向きに合わせる回転 q_r は
* q_r = q_a^* q_b (ここで q^* は q の共役)
* 
* 初期回転は以下のようになる
* q_0 = q_a0^* q_b0
* (後で使うので q_0^* の形で求めて覚えておく)
*/
Physics::ConstraintAnchorAxis& Physics::ConstraintAnchorAxis::Init(const Physics::RigidBody* RbA, const Physics::RigidBody* RbB, const LinAlg::Vec3& WAnchor, const LinAlg::Vec3& WAxis)
{
	Super::Init(RbA, RbB, WAnchor);

	LAxisA = RigidBodyA->ToLocalDir(WAxis);

	InvInitRot = (RigidBodyA->GetRotation().Inverse() * RigidBodyB->GetRotation()).Inverse();

	return *this;
}

void Physics::ConstraintDistance::PreSolve(const float DeltaSec)
{
	//!< ワールドへ変換 (オブジェクトが動く事を考慮して都度やる必要がある)
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	//!< アンカー間
	const auto AB = WAnchorB - WAnchorA;
	{
		//!< それぞれの剛体における、重心からアンカー位置 (ワールド) へのベクトル
		const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
		const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World();

		/*
		* R_1, R_2 の距離がゼロとなる制約なので C = (R_2 - R_1).LengthSq() = Dot(R_2 - R_1, R_2 - R_1) = 0
		* 
		* C = (R_2 - R_1) (R_2 - R_1) = 0 ... R_1, R_2 は A, B のワールドアンカー位置
		*  &= R_2 R_2 - R_2 R_1 - R_1 R_2 + R_1 R_1
		*  &= \dot{R_2} R_2 + R_2 \dot{R_2} - \dot{R_2} R_1 - R_2 \dot{R_1} - \dot{R_1} R_2 - R_1 \dot{R_2} + \dot{R_1} R_1 + R_1 \dot{R_1}
		*  &= 2 (R_2 - R_1) \dot{R_2} + 2 (R_1 - R_2) \dot{R_1}
		* \dot{R_1} = v_1 + w_1 \cross R_1, \dot{R_2} = v_2 + w_2 \cross R_2 より
		*  &= 2 (R_2 - R_1) (v_2 + w_2 \cross R_2) + 2 (R_1 - R_2) (v_1 + w_1 \cross R_1)
		*  &= 
		*  \begin{pmatrix}
		*  2 (R_1 - R_2) & 2 R_1 \cross (R_1 - R_2) & 2 (R_2 - R_1) & 2 R_2 \cross (R_2 - R_1)
		*  \end{pmatrix}
		*  \begin{pmatrix}
		*  v_1 \\
		*  w_1 \\ 
		*  v_2 \\ 
		*  w_2
		*  \end{pmatrix}
		* 
		* なので J = 2 (-AB), 2 RA.Cross(-AB), 2 AB, 2 RB.Cross(AB)
		*/
		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	//!< 事前計算しておく
	Jacobian_Transpose = Jacobian.Transpose();
	J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

#pragma region WARM_STARTING
	//!< 前フレームの値を今フレームの計算前に適用することで、少ない繰返し回数で安定状態へ収束させる
	ApplyImpulse(Jacobian_Transpose * CachedLambda);
#pragma endregion

#pragma region BAUMGARTE_STABILIZATION
	/*
	* ここではアンカー位置が一致する状態を目指しているので AB の長さが 0 になるような修正 C / DeltaSec を行う (C = AB.Dot(AB))
	* システムにエネルギーを与える為わずかに振動することになる、これを回避する為許容範囲 (Torelance) 内の場合は影響を与えないようにしている
	* 完全に修正するとシステムにエネルギーを与え過ぎ不安定になるので C に \beta を乗算して数フレームかけて修正する
	*/
	constexpr auto Torelance = 0.01f;
	const auto C = (std::max)(AB.Dot(AB) - Torelance, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
#pragma endregion
}

/*
* J M^{-1} J^T \lambda = -J v
* J M^{-1} J^T = A, \lambda = x, -J v = B とすると
* A x = B となる、これを x (==\lambda) についてガウスザイデル法で解く
*/
void Physics::ConstraintDistance::Solve()  
{
	const auto& A = J_IM_JT;
	const auto B = -Jacobian * GetVelocties()
#pragma region BAUMGARTE_STABILIZATION
		//!< 安定化の為の修正 (バウムガルテの安定化法)
		- LinAlg::Vec<1>(Baumgarte);
#pragma endregion

	//!< ガウスザイデル法で x (==\lambda) を求める
	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(Jacobian_Transpose * Lambda);

#pragma region WARM_STARTING
	//!< 次フレーム用に今回の力積の大きさを蓄積しておく (ウォームスタート)
	CachedLambda += Lambda;
#pragma endregion
}
void Physics::ConstraintDistance::PostSolve() 
{
	if (std::isnan(CachedLambda[0])) { CachedLambda[0] = 0.0f; }

#pragma region WARM_STARTING
	//!< 衝突時など (短時間に大きな力積が加わった場合) に大きな修正力積になり、
	//!< ウォームスタートが効きすぎて不安定になる事があるので CachedLambda の値を適切な範囲に制限する
	constexpr auto Eps = (std::numeric_limits<float>::epsilon)();
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Eps, Eps);
#pragma endregion
}

/*
* 四元数コンストレイント
*
* a, b の回転を表す四元数を q_a, q_b とすると
*
* a を b の向きに合わせる回転 q_r は
* q_r = q_a^* q_b (ここで q^* は q の共役)
*
* 相対的な回転を制限する制約は q_r q_0^* が単位四元数になるので
* C = q_r q_0^* =
* \begin{pmatrix}
* 1 \\
* 0 \\
* 0 \\
* 0
* \end{pmatrix}
* (単位四元数)
*
* 従って 行列 P を以下のように定義すると
* P =
* \begin{pmatrix}
* 0 & 1 & 0 & 0 \\
* 0 & 0 & 1 & 0 \\
* 0 & 0 & 0 & 1
* \end{pmatrix}
*
* C = P (q_r q_0^*) = 0 と記述できる
*
* ヒンジ軸 h に垂直な 2 軸を u, v とすると
* C_1 = P (q_r q_0^*) u = 0
* C_2 = P (q_r q_0^*) v = 0
*
* q_r = q_a^* q_b なので
* \dot{C_1} = P \dot{q_a^* q_b} q_0^* u + P (q_r q_0^*) \dot{u}
* &= P \dot{q_a^*} q_b q_0^* u + P q_a^* \dot{q_b} q_0^* u + P (q_r q_0^*) \dot{u}
*
* \dot{u} = w \cross r
* ヒンジで制約では r = 0 なので　\dot{u} = w \cross r　= 0
*
* また \dot{q} = \frac{1}{2} w q
* ただし w =
* \begin{pmatrix}
* 0 \\
* q_x \\
* q_y \\
* q_z
* \end{pmatrix}
* (w は q の虚数部からなる四元数)
* であるので
*
* &= P \frac{1}{2} ((w_a q_a)^* q_b q_0^* u + q_a^* w_b q_b q_0^* u)
* &= \frac{1}{2} P (q_a^* w_a^* q_b q_0^* u + q_a^* w_b q_b q_0^* u)
*
* w^* = -w なので (w は虚数部のみからなる為)
*
* &= \frac{1}{2} P (-q_a^* w_a q_b q_0^* u + q_a^* w_b q_b q_0^* u)
*
* L(), R() は四元数 a, b から 4次元行列を作成する、以下が成立するという特性を持つため
* a b = L(a) b = R(b) a
*
* &= \frac{1}{2} P (-L(q_a^*) R(q_b q_0^*) w_a u + L(q_a^*) R(q_b q_0^*) w_b u)
* &= \frac{1}{2} P (-L(q_a^*) R(q_b q_0^*) P^t u w_a + L(q_a^*) R(q_b q_0^*) P^t u w_b)
*
* &=
* \begin{pmatrix}
* 0 & -\frac{1}{2} P L(q_a^*) R(q_b q_0^*) P^t u & 0 & \frac{1}{2} P L(q_a^*) R(q_b q_0^*) P^t u
* \end{pmatrix}
* \begin{pmatrix}
* v_a \\
* w_a \\
* v_b \\
* w_b
* \end{pmatrix}
*
* よってヤコビ行列は
*
* J_1 =
* \begin{pmatrix}
* 0 & -\frac{1}{2} P L(q_a^*) R(q_b q_0^*) P^t u & 0 & \frac{1}{2} P L(q_a^*) R(q_b q_0^*) P^t u
* \end{pmatrix}
*
* J_2 =
* \begin{pmatrix}
* 0 & -\frac{1}{2} P L(q_a^*) R(q_b q_0^*) P^t v & 0 & \frac{1}{2} P L(q_a^*) R(q_b q_0^*) P^t v
* \end{pmatrix}
*/
void Physics::ConstraintHinge::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;
	{
		const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
		const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World();
		//!< 距離コンストレイント
		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}
	//!< 四元数コンストレイント
	{
		const auto& QA = RigidBodyA->GetRotation();
		const auto& QB = RigidBodyB->GetRotation();
		const auto InvQA = QA.Inverse();

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		//!< 本来は P の転置行列を求める (ここでは転置する意味がないのでやらない)
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

	Jacobian_Transpose = Jacobian.Transpose();
	J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

	ApplyImpulse(Jacobian_Transpose * CachedLambda);

	constexpr auto Torelance = 0.01f;
	const auto C = (std::max)((AB).Dot(AB) - Torelance, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintHinge::Solve()
{
	const auto& A = J_IM_JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<3>(Baumgarte, 0.0f, 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(Jacobian_Transpose * Lambda);

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

	//!< 距離コンストレイント
	{
		const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
		const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World();
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
		const auto& QA = RigidBodyA->GetRotation();
		const auto& QB = RigidBodyB->GetRotation();
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

		/*
		* 角度制限 (角度違反している場合に追加の制約)
		* 
		* a を b の向きに合わせる回転 q_r は
		* q_r = q_a^* q_b (ここで q^* は q の共役)
		* 
		* 初期回転を q_0 とすると
		* q_0 = q_a0^* q_b0
		* 
		* 両四元数の差を q_rr とすると
		* q_rr = q_r q_0^*
		* 
		* 回転角は
		* \theta = 2.0f * asinf(q_rr h) ... q_rr は虚数部ベクトルを用いる、h はヒンジ軸
		*/
		{
			//!< 現在の回転
			const auto CurRot = InvQA * QB * InvInitRot;
			//!< 初期位置からの回転角度を求め、破綻しているかどうか
			Angle = LinAlg::ToDegree(2.0f * std::asinf(CurRot.ToVec3().Dot(LAxisA)));
			IsAngleViolated = std::fabsf(Angle) > LimitAngle;
			if (IsAngleViolated) {
				//!< 破綻している場合のみ制約を追加
				//!< ここではヤコビ行列を求めておくだけ、制限は Solve() 内で行う
				const auto J1 = LinAlg::Vec3::Zero();
				const auto J2 = MatA * LinAlg::Vec4(LAxisA);
				const auto J3 = LinAlg::Vec3::Zero();
				const auto J4 = MatB * LinAlg::Vec4(LAxisA);
				Jacobian[3] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
			}
			else {
				//!< 破綻してない場合は何もしない
				Jacobian[3].ToZero();
			}
		}
	}

	Jacobian_Transpose = Jacobian.Transpose();
	J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

	ApplyImpulse(Jacobian_Transpose * CachedLambda);

	constexpr auto Torelance = 0.01f;
	const auto C = (std::max)((AB).Dot(AB) - Torelance, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintHingeLimited::Solve()
{
	const auto& A = J_IM_JT;
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

	ApplyImpulse(Jacobian_Transpose * Lambda);

	CachedLambda += Lambda;
}
//!< ウォームスタートは距離制約分のみ
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
	
	//!< 距離コンストレイント
	{
		const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
		const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World();

		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}
	//!< 四元数コンストレイント
	{
		const auto& QA = RigidBodyA->GetRotation();
		const auto& QB = RigidBodyB->GetRotation();
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

	Jacobian_Transpose = Jacobian.Transpose();
	J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

	ApplyImpulse(Jacobian_Transpose * CachedLambda);

	constexpr auto Torelance = 0.01f;
	const auto C = (std::max)((AB).Dot(AB) - Torelance, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintBallSocket::Solve() 
{
	const auto& A = J_IM_JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<2>(Baumgarte, 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(Jacobian_Transpose * Lambda);

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

	//!< 距離コンストレイント
	{
		const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
		const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World();

		{
			const auto J1 = -AB * 2.0f;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	//!< 四元数コンストレイント
	{
		const auto& QA = RigidBodyA->GetRotation();
		const auto& QB = RigidBodyB->GetRotation();
		const auto InvQA = QA.Inverse();

		const auto P = LinAlg::Mat4(LinAlg::Vec4::AxisX(), LinAlg::Vec4::AxisY(), LinAlg::Vec4::AxisZ(), LinAlg::Vec4::Zero());
		const auto& PT = P;

		const auto MatB = P * InvQA.ToLeftMat4() * (QB * InvInitRot).ToRightMat4() * PT * 0.5f;
		const auto MatA = -MatB;

		//!< ここまでは BallSocket と同じ
		{
			const auto J1 = LinAlg::Vec3::Zero();
			const auto J2 = -0.5f * MatA * LinAlg::Vec4(LAxisA);
			const auto J3 = LinAlg::Vec3::Zero();
			const auto J4 = 0.5f * MatB * LinAlg::Vec4(LAxisA);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}

		//!< U, V に角度制限
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

	Jacobian_Transpose = Jacobian.Transpose();
	J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

	ApplyImpulse(Jacobian_Transpose * CachedLambda);

	constexpr auto Torelance = 0.01f;
	const auto C = (std::max)((AB).Dot(AB) - Torelance, 0.0f);
	constexpr auto Beta = 0.05f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintBallSocketLimited::Solve()
{
	const auto& A = J_IM_JT;
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

	ApplyImpulse(Jacobian_Transpose * Lambda);

	CachedLambda += Lambda;
}
//!< ウォームスタートは距離制約分のみ
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

	//!< 距離コンストレイント
	{
		const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
		const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World();

		const auto J1 = -AB * 2.0f;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -J1;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
	}

	//!< 四元数コンストレイント
	{
		const auto& QA = RigidBodyA->GetRotation();
		const auto& QB = RigidBodyB->GetRotation();
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

			Jacobian_Transpose = Jacobian.Transpose();
			J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

			//!< A ローカルでの相対回転
			const auto RelRot = InvQA * QBInvInitRot;

			//!< ワールドでの回転軸
			const auto WRelAxis = RigidBodyA->ToWorldDir(RelRot.Imag());
			const auto C = AB.Dot(AB);
			constexpr auto Beta = 0.05f;
			Baumgarte = { C, U.Dot(WRelAxis), V.Dot(WRelAxis) } * Beta / DeltaSec;
		}
	}
}
void Physics::ConstraintMotor::Solve() 
{
	const auto& A = J_IM_JT;
	
	//!< 与えたい速度を求める (両者が相対的に一定速度で回転するような速度)
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

	//!< 求めた速度を減算してから適用することで、ソルバーは騙されて与えたい相対速度になるような力積を作り出す
	const auto B = -Jacobian * (GetVelocties() - Vel) - LinAlg::Vec<4>(Baumgarte.X(), Baumgarte.Y(), Baumgarte.Z(), 0.0f);

	const auto Lambda = GaussSiedel(A, B);

	ApplyImpulse(Jacobian_Transpose * Lambda);
}

void Physics::ConstraintMoverUpDown::PreSolve(const float DeltaSec)
{
	//!< (ここでは) Y 軸方向に半径 4 のコサイン軌道
	constexpr auto Radius = 4.0f;
	RigidBodyA->GetVelocity_Linear()[1] = std::cosf(Timer) * Radius;
	Timer += DeltaSec;
}
void Physics::ConstraintMoverRotateX::PreSolve(const float DeltaSec) 
{
	RigidBodyA->GetVelocity_Angular()[0] = LinAlg::ToRadian(60.0f);
}
void Physics::ConstraintMoverRotateY::PreSolve(const float DeltaSec)
{
	RigidBodyA->GetVelocity_Angular()[1] = LinAlg::ToRadian(60.0f);
}
void Physics::ConstraintMoverRotateZ::PreSolve(const float DeltaSec)
{
	RigidBodyA->GetVelocity_Angular()[2] = LinAlg::ToRadian(60.0f);
}
Physics::ConstraintPenetration& Physics::ConstraintPenetration::Init(const Collision::Contact& Ct)
{
	Super::Init(Ct.RigidBodyA, Ct.RigidBodyB);

	LAnchorA = Ct.LPointA;
	LAnchorB = Ct.LPointB;

	//!< A ローカル接触法線
	LNormal = RigidBodyA->ToLocalDir(Ct.WNormal).Normalize();

	Friction = RigidBodyA->GetFriction() * RigidBodyB->GetFriction();
	Mg = std::fabsf(Physics::RigidBody::Graivity.Y()) / (RigidBodyA->GetMass_Inverse() + RigidBodyB->GetMass_Inverse());

	return *this;
}
void Physics::ConstraintPenetration::PreSolve(const float DeltaSec)
{
	const auto WAnchorA = RigidBodyA->ToWorldPos(LAnchorA);
	const auto WAnchorB = RigidBodyB->ToWorldPos(LAnchorB);
	const auto AB = WAnchorB - WAnchorA;

	//!< 法線をワールドスペースへ
	const auto WNormal = RigidBodyA->ToWorldDir(LNormal);
	const auto RA = WAnchorA - RigidBodyA->GetCenterOfMass_World();
	const auto RB = WAnchorB - RigidBodyB->GetCenterOfMass_World(); 
	{
		/*
		* C = N (R_2 - R_1) = 0 ... R_1, R_2 は A, B のワールドアンカー位置
		* \dot{C} = n \dot{R_2} - n \dot{R_1} = 0
		* \dot{R_1} = v_1 + w_1 \cross R_1, \dot{R_2} = v_2 + w_2 \cross R_2 より
		* \dot{C} = n (v_2 + w_2 \cross R_2) - n (v_1 + w_1 \cross R_1)
		* & = n v_2 + w_2 (R_2 \cross n) - n v_1 - w_1 (R_1 \cross n) ... a (b \cross c) = b (c \cross a) = c (a \cross b) なので
		* &=
		* \begin{ pmatrix }
		* -n & -R_1 \cross n & n & R_2 \cross n
		* \end{ pmatrix }
		* \begin{ pmatrix }
		* v_1 \\
		* w_1 \\
		* v_2 \\
		* w_2
		* \end{ pmatrix }
		* なので J = (-N, -RA.Cross(N), N, RB.Cross(N)) となる
		*/
		const auto J1 = -WNormal;
		const auto J2 = RA.Cross(J1);
		const auto J3 = -J1;
		const auto J4 = RB.Cross(J3);
		Jacobian[0] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
	}

	//!< 摩擦がある場合、接線方向のコンストレイントを考慮
	//!< 力を \mu mg に制限する以外は、法線の場合と同様
	if (Friction > 0.0f) {
		//!< 接線 (直交) ベクトル U, V を求める
		LinAlg::Vec3 U, V;
		WNormal.GetOrtho(U, V);
		
		//!< U
		{
			// J = (-U, -RA.Cross(U), U, RB.Cross(U)) 
			const auto J1 = -U;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[1] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
		//!< V
		{
			// J = (-V, -RA.Cross(V), V, RB.Cross(V)) 
			const auto J1 = -V;
			const auto J2 = RA.Cross(J1);
			const auto J3 = -J1;
			const auto J4 = RB.Cross(J3);
			Jacobian[2] = { XYZ(J1), XYZ(J2), XYZ(J3), XYZ(J4) };
		}
	}

	Jacobian_Transpose = Jacobian.Transpose();
	J_IM_JT = Jacobian * GetInverseMassMatrix() * Jacobian_Transpose;

	ApplyImpulse(Jacobian_Transpose * CachedLambda);

	constexpr auto Torelance = 0.02f;
	const auto C = (std::min)(AB.Dot(WNormal) + Torelance, 0.0f);
	constexpr auto Beta = 0.25f;
	Baumgarte = Beta * C / DeltaSec;
}
void Physics::ConstraintPenetration::Solve()
{
	const auto& A = J_IM_JT;
	const auto B = -Jacobian * GetVelocties() - LinAlg::Vec<3>(Baumgarte, 0.0f, 0.0f);

	auto Lambda = GaussSiedel(A, B);

	const auto Old = CachedLambda; {
		CachedLambda += Lambda;
		//!< N
		CachedLambda[0] = (std::max)(CachedLambda[0], 0.0f);
		if (Friction > 0.0f) {
			//!< 法線方向の力から、摩擦力を計算
			const auto Force = (std::max)(Mg, std::fabsf(Lambda[0])) * Friction;
			//!< U
			CachedLambda[1] = (std::clamp)(CachedLambda[1], -Force, Force);
			//!< V
			CachedLambda[2] = (std::clamp)(CachedLambda[2], -Force, Force);
		}
	}
	Lambda = CachedLambda - Old;

	ApplyImpulse(Jacobian_Transpose * Lambda);
}

//!< 1 フレームのみの制約だとウォームスタートの恩恵が無い為、接触をキャッシュする
Physics::Manifold::Manifold(const Collision::Contact& Ct) 
	: RigidBodyA(Ct.RigidBodyA), RigidBodyB(Ct.RigidBodyB) 
{
	Add(Ct); 
}
void Physics::Manifold::Add(const Collision::Contact& CtOrig)
{
	//!< A, B の順序が食い違っている場合はスワップして合わせる
	auto Ct = CtOrig;
	if (RigidBodyA != CtOrig.RigidBodyA) { 
		Ct.Swap(); 
	}

	Ct.WNormal *= -1.0f;

	//!< 既存とほぼ同じ接触位置の場合は追加対象とせず破棄 (既存を残す、古い方がウォームスタートの恩恵がある為)
	constexpr auto Eps2 = 0.02f * 0.02f;
	if (std::ranges::any_of(ContactAndConstraints,
		[&](const auto& i) {
			return (i.first.WPointA - Ct.WPointA).LengthSq() < Eps2 || (i.first.WPointB - Ct.WPointB).LengthSq() < Eps2;
		})) {
		return;
	}

	//!< 新規要素 (接触情報と貫通制約のペア)
	const auto NewItem = ContactAndConstraint({ Ct, ConstraintPenetration(Ct) });
	if (std::size(ContactAndConstraints) < MaxContactAndConstraints) {
		//!< 空きがあればそのまま追加
		ContactAndConstraints.emplace_back(NewItem);
	}
	else {
		//!< (新規を) 一旦追加してしまう
		ContactAndConstraints.emplace_back(NewItem);
		
		//!< (新規を含めた) 平均値を求める
		const auto Avg = (std::accumulate(std::cbegin(ContactAndConstraints), std::cend(ContactAndConstraints), LinAlg::Vec3::Zero(),
			[](const auto& Acc, const auto& i) {
				return Acc + i.first.WPointA;
			})) / static_cast<float>(std::size(ContactAndConstraints));

		//!< 平均値に一番近い要素を取り除く (大きく貫通しているものほど深刻として残す)
		ContactAndConstraints.erase(std::ranges::min_element(ContactAndConstraints,
			[&](const auto& lhs, const auto& rhs) {
				return (Avg - lhs.first.WPointA).LengthSq() < (Avg - rhs.first.WPointA).LengthSq();
			}));
	}
}

//!< 古くなった接触情報を削除
void Physics::Manifold::RemoveExpired()
{
	constexpr auto Eps2 = 0.02f * 0.02f;
	const auto Range = std::ranges::remove_if(ContactAndConstraints, 
		[&](const auto& rhs) {
			const auto& Ct = rhs.first;
			//!< 衝突時のローカル位置を、現在のトランスフォームでワールド位置へ変換
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
	ContactAndConstraints.erase(std::cbegin(Range), std::cend(Range));
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
		//!< 新規として追加
		Manifolds.emplace_back(Manifold(Ct));
	}
}

void Physics::ManifoldCollector::PreSolve(const float DeltaSec) 
{
	for (auto& i : Manifolds) {
		for (auto& j : i.ContactAndConstraints) {
			j.second.PreSolve(DeltaSec);
		}
	}
}
void Physics::ManifoldCollector::Solve()
{
	for (auto& i : Manifolds) {
		for (auto& j : i.ContactAndConstraints) {
			j.second.Solve();
		}
	}
}
void Physics::ManifoldCollector::PostSolve()
{
	for (auto& i : Manifolds) {
		for (auto& j : i.ContactAndConstraints) {
			j.second.PostSolve();
		}
	}
}

#undef XYZ