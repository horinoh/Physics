#include "Constraint.h"
#include "RigidBody.h"
#include "Collision.h"

//!< Lambda = -J * v / (J * M.Inverse() * J.Transpose())
//!<	Lambda : ラグランジュの未定乗数 (Lagrange multiplier) 
//!<	J : ヤコビ行列 dC/dt = J * v = 0

//!< 質量行列
//!< M = (M_A    0    0    0) ... A の質量の逆数が対角成分 (3x3)
//!<     (     I_A    0    0) ... A の慣性テンソルの逆行列 (3x3)
//!<     (   0   0  M_B    0) ... B の質量の逆数が対角成分
//!<     (   0   0    0  I_B) ... B の慣性テンソルの逆行列
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
Math::Mat4 Physics::Constraint::Left(const Math::Quat& lhs)
{
	return Math::Mat4(Math::Vec4(lhs.W(), -lhs.X(), -lhs.Y(), -lhs.Z()), 
		Math::Vec4(lhs.X(), lhs.W(), -lhs.Z(), lhs.Y()), 
		Math::Vec4(lhs.Y(), lhs.Z(), lhs.W(), -lhs.X()), 
		Math::Vec4(lhs.Z(), -lhs.Y(), lhs.X(), lhs.W())).Transpose();
}
Math::Mat4 Physics::Constraint::Right(const Math::Quat& lhs)
{
	return Math::Mat4(Math::Vec4(lhs.W(), -lhs.X(), -lhs.Y(), -lhs.Z()),
		Math::Vec4(lhs.X(), lhs.W(), lhs.Z(), -lhs.Y()),
		Math::Vec4(lhs.Y(), -lhs.Z(), lhs.W(), lhs.X()),
		Math::Vec4(lhs.Z(), lhs.Y(), -lhs.X(), lhs.W())).Transpose();
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

	//!< 前のフレームの力 (CachedLambda) を今フレームの計算前に適用することで、少ないフレームで安定状態へ持っていく (Warm starting)
	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

	//!< 適正な位置へ戻すような力を適用する事で位置ドリフトを修正 (Baumgarte stabilization)
	//!< システムにわずかにエネルギーを追加する為、振動してしまう事への対処 (Slop)
	const auto C = (std::max)(AB.Dot(AB) - 0.01f, 0.0f);
	//!< 一気にやるとシステムにエネルギーを追加しすぎる為、数フレームかけて適用する
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
	//!< 発散を防ぐ対処 (Warm starting)
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

	//!< A 空間での法線
	Normal = RigidBodyA->Rotation.Inverse().Rotate(-Ct.Normal).Normalize();
	Friction = RigidBodyA->Friction * RigidBodyB->Friction;

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

	const auto QA = RigidBodyA->Rotation;
	const auto QB = RigidBodyB->Rotation;
	const auto InVQ = Q.Inverse();
	const auto InvQA = QA.Inverse();

	Math::Vec3 U, V;
	AxisA.GetOrtho(U, V);

	const auto P = Math::Mat4(Math::Vec4::Zero(), Math::Vec4::AxisY(), Math::Vec4::AxisZ(), Math::Vec4::AxisW());
	//!< (ここでは) 転置する意味がないが、一応やっておく
	const auto PT = P.Transpose();

	const auto MatA = P * Left(InvQA) * Right(QB * InVQ) * PT * -0.5f;
	const auto MatB = P * Left(InvQA) * Right(QB * InVQ) * PT * 0.5f;

	//!< 角度を求める、また破綻しているかどうか調査
	const auto qrr = InvQA * QB * InVQ;
	const auto RelativeAngle = 2.0f * asinf(Math::Vec3(qrr.X(), qrr.Y(), qrr.Z()).Dot(AxisA)) * 180.0f / std::numbers::pi_v<float>;
	IsAngleViolated = std::fabsf(RelativeAngle) > 45.0f;

	ApplyImpulse(Jacobian.Transpose() * CachedLambda);

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
	
	{
		const auto J1 = Math::Vec3::Zero();
		Jacobian[1][0] = J1.X();
		Jacobian[1][1] = J1.Y();
		Jacobian[1][2] = J1.Z();
		const auto J2 = MatA * Math::Vec4(0.0f, U.X(), U.Y(), U.Z());
		Jacobian[1][3] = J2.Y();
		Jacobian[1][4] = J2.Z();
		Jacobian[1][5] = J2.W();
		const auto J3 = Math::Vec3::Zero();
		Jacobian[1][6] = J3.X();
		Jacobian[1][7] = J3.Y();
		Jacobian[1][8] = J3.Z();
		const auto J4 = MatB * Math::Vec4(0.0f, U.X(), U.Y(), U.Z());
		Jacobian[1][9] = J4.Y();
		Jacobian[1][10] = J4.Z();
		Jacobian[1][11] = J4.W();
	}
	{
		const auto J1 = Math::Vec3::Zero();
		Jacobian[2][0] = J1.X();
		Jacobian[2][1] = J1.Y();
		Jacobian[2][2] = J1.Z();
		const auto J2 = MatA * Math::Vec4(0.0f, V.X(), V.Y(), V.Z());
		Jacobian[2][3] = J2.Y();
		Jacobian[2][4] = J2.Z();
		Jacobian[2][5] = J2.W();
		const auto J3 = Math::Vec3::Zero();
		Jacobian[2][6] = J3.X();
		Jacobian[2][7] = J3.Y();
		Jacobian[2][8] = J3.Z();
		const auto J4 = MatB * Math::Vec4(0.0f, V.X(), V.Y(), V.Z());
		Jacobian[2][9] = J4.Y();
		Jacobian[2][10] = J4.Z();
		Jacobian[2][11] = J4.W();
	}
	if (IsAngleViolated) {
		const auto J1 = Math::Vec3::Zero();
		Jacobian[3][0] = J1.X();
		Jacobian[3][1] = J1.Y();
		Jacobian[3][2] = J1.Z();
		const auto J2 = MatA * Math::Vec4(0.0f, AxisA.X(), AxisA.Y(), AxisA.Z());
		Jacobian[3][3] = J2.Y();
		Jacobian[3][4] = J2.Z();
		Jacobian[3][5] = J2.W();
		const auto J3 = Math::Vec3::Zero();
		Jacobian[3][6] = J3.X();
		Jacobian[3][7] = J3.Y();
		Jacobian[3][8] = J3.Z();
		const auto J4 = MatB * Math::Vec4(0.0f, AxisA.X(), AxisA.Y(), AxisA.Z());
		Jacobian[3][9] = J4.Y();
		Jacobian[3][10] = J4.Z();
		Jacobian[3][11] = J4.W();
	}

	const auto C = (std::max)((AB).Dot(AB) - 0.01f, 0.0f);
	Baumgarte = 0.05f * C / DeltaSec;
}
void Physics::ConstraintHinge::Solve()
{
	const auto JT = Jacobian.Transpose();
	const auto A = Jacobian * GetInverseMassMatrix() * JT;
	auto B = -Jacobian * GetVelocties(); B[0] -= Baumgarte;

	auto Lambda = GaussSiedel(A, B);

	//!< 角度制限により、トルクを制限する
	if (IsAngleViolated) {
		if (RelativeAngle > 0.0f) {
			Lambda[3] = (std::min)(Lambda[3], 0.0f);
		}
		if (RelativeAngle < 0.0f) {
			Lambda[3] = (std::max)(Lambda[3], 0.0f);
		}
	}

	ApplyImpulse(JT * Lambda);

	CachedLambda += Lambda;
}
void Physics::ConstraintHinge::PostSolve()
{
	if (CachedLambda[0] * 0.0f != CachedLambda[0] * 0.0f) { CachedLambda[0] = 0.0f; }
	constexpr auto Limit = 20.0f;
	CachedLambda[0] = (std::clamp)(CachedLambda[0], -Limit, Limit);
	CachedLambda[1] = CachedLambda[2] = CachedLambda[3] = 0.0f;
}

void Physics::Manifold::Add(const Collision::Contact& CtOrig)
{
	//!< 剛体 A, B の順序が食い違っている場合は辻褄を合わせる
	auto Ct = CtOrig;
	if (RigidBodyA != CtOrig.RigidBodyA) { Ct.Swap(); }

	//!< 既存とほぼ同じ接触位置の場合は何もしない
	constexpr auto Esp2 = 0.02f * 0.02f;
	if (std::ranges::any_of(Constraints, [&](const auto& i) {
		return (i.first.PointA - Ct.PointA).LengthSq() < Esp2 || (i.first.PointB - Ct.PointB).LengthSq() < Esp2;
	})) {
		return;
	}

	const auto NewItem = ContactAndConstraint({ Ct, ConstraintPenetration(Ct) });
	if (std::size(Constraints) < 4) {
		//!< 空きがある場合は追加
		Constraints.emplace_back(NewItem);
	}
	else {
		//!< 平均値に最も近いものを入れ替える
		const auto Avg = (Constraints[0].first.PointA + Constraints[1].first.PointA + Constraints[2].first.PointA + Constraints[3].first.PointA + Ct.PointA) * 0.2f;
		*std::ranges::min_element(Constraints, [&](const auto& lhs, const auto& rhs) {
			return (Avg - lhs.first.PointA).LengthSq() < (Avg - rhs.first.PointA).LengthSq();
		}) = NewItem;
	}
}
void Physics::Manifold::RemoveExpired()
{
	constexpr auto Eps2 = 0.02f * 0.02f;
	const auto Range = std::ranges::remove_if(Constraints, [&](const auto& i) {
		const auto AB = i.first.PointB - i.first.PointA;
		const auto N = i.first.RigidBodyA->Rotation.Rotate(i.first.Normal);
		const auto PenDepth = N.Dot(AB);
		return PenDepth > 0.0f || (AB - N * PenDepth).LengthSq() >= Eps2;
	});
	Constraints.erase(std::cbegin(Range), std::cend(Range));
}

void Physics::ManifoldCollector::Add(const Collision::Contact& Ct)
{
	//!< 同じ剛体 A, B 間のマニフォールドか存在するか
	auto It = std::ranges::find_if(Manifolds, [&](const auto& i) {
		return (i.RigidBodyA==Ct.RigidBodyA && i.RigidBodyB == Ct.RigidBodyB) || (i.RigidBodyA == Ct.RigidBodyB && i.RigidBodyB == Ct.RigidBodyA);
	});
	if (std::cend(Manifolds) != It) {
		//!< 既存なら衝突を追加
		It->Add(Ct);
	}
	else {
		//!< 既存でなければ、マニフォールド(と衝突)を追加
		Manifolds.emplace_back().Add(Ct);
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