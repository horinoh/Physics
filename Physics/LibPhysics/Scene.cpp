#include <algorithm>
#include <vector>

#include "Scene.h"
#include "Shape.h"
#include "RigidBody.h"
#include "Constraint.h"

#include "Log.h"

const Math::Vec3 Physics::Scene::BroadPhaseAxis = Math::Vec3::One().Normalize();

#ifdef USE_BRUTE_FORCE
void Physics::Scene::BruteForce(const float DeltaSec, std::vector<Collision::Contact>& Contacts)
{
	const auto RbCount = std::size(RigidBodies);
	Contacts.reserve(RbCount * RbCount);
	Contacts.clear();
	for (auto i = 0; i < RbCount; ++i) {
		for (auto j = i + 1; j < RbCount; ++j) {
			const auto RbA = RigidBodies[i];
			const auto RbB = RigidBodies[j];
			if (0.0f != RbA->InvMass || 0.0f != RbB->InvMass) {
				Collision::Contact Ct;
				if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
					if (0.0f == Ct.TimeOfImpact) {
						Manifolds.Add(Ct);
					}
					else {
						Contacts.emplace_back(Ct);
					}
				}
			}
		}
	}
	std::ranges::sort(Contacts, std::ranges::less{}, &Collision::Contact::TimeOfImpact);
}
#else
//!< SAP (Sweep And Prune)
void Physics::Scene::BroadPhase(const float DeltaSec, std::vector<CollidablePair>& CollidablePairs)
{
	std::vector<Collision::BoundEdge> BoundEdges;
	{
		const auto RbCount = std::size(RigidBodies);
		BoundEdges.reserve(RbCount * 2);
		for (auto i = 0; i < RbCount; ++i) {
			const auto Rb = RigidBodies[i].get();

			auto Aabb = Rb->Shape->GetAABB(Rb->Position, Rb->Rotation);
			//!< デルタ時間に移動する分だけ AABB を拡張する
			const auto DVel = Rb->LinearVelocity * DeltaSec;
			Aabb.Expand(Aabb.Min + DVel);
			Aabb.Expand(Aabb.Max + DVel);

			//!< さらに少し拡張 (取りこぼし防止？)
			const auto Epsilon = 0.01f;
			const auto DEp = Math::Vec3::One() * Epsilon;
			Aabb.Expand(Aabb.Min - DEp);
			Aabb.Expand(Aabb.Max + DEp);

			//!< 境界端 (上限、下限) を覚えておく
			BoundEdges.emplace_back(Collision::BoundEdge({ i, BroadPhaseAxis.Dot(Aabb.Min), true })); //!< 下限
			BoundEdges.emplace_back(Collision::BoundEdge({ i, BroadPhaseAxis.Dot(Aabb.Max), false }));//!< 上限
		}

		//!< 軸に射影した内積値でソート
		std::ranges::sort(BoundEdges, std::ranges::less{}, &Collision::BoundEdge::Value);
	}

	//!< 射影 AABB から、潜在的衝突ペアリストを構築
	const auto BoundsCount = std::size(BoundEdges);
	for (auto i = 0; i < BoundsCount; ++i) {
		const auto& A = BoundEdges[i];
		//!< 下限なら対となる上限が見つかるまで探す
		if (A.IsMin) {
			for (auto j = i + 1; j < BoundsCount; ++j) {
				const auto& B = BoundEdges[j];
				//!< 対となる上限 (同じインデックス) が見つかれば終了
				if (A.Index == B.Index) {
					break;
				}
				//!< 他オブジェクトが見つかった場合は、潜在的衝突相手として収集
				if (B.IsMin) {
					CollidablePairs.emplace_back(CollidablePair({ A.Index, B.Index }));
				}
			}
		}
	}
}
void Physics::Scene::NarrowPhase(const float DeltaSec, const std::vector<CollidablePair>& CollidablePairs, std::vector<Collision::Contact>& Contacts)
{
	Contacts.clear();
	Contacts.reserve(std::size(CollidablePairs));

	//!< 潜在的衝突相手と、実際に衝突しているかを調べる
	for (const auto& i : CollidablePairs) {
		const auto RbA = RigidBodies[i.first].get();
		const auto RbB = RigidBodies[i.second].get();
		
		if (0.0f == RbA->InvMass && 0.0f == RbB->InvMass) {
			continue;
		}

		Collision::Contact Ct;
		if (RbA->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE && RbB->Shape->GetShapeType() == Physics::Shape::SHAPE_TYPE::SPHERE) {
			if (Collision::Intersection::SphereSphere(RbA, RbB, DeltaSec, Ct)) {
				Contacts.emplace_back(Ct);
			}
		}
		else {
			if (Collision::Intersection::RigidBodyRigidBody(RbA, RbB, DeltaSec, Ct)) {
				if (0.0f == Ct.TimeOfImpact) {
					//!< 静的衝突
					Manifolds.Add(Ct);
				}
				else {
					//!< 動的衝突
					Contacts.emplace_back(Ct);
				}
			}
		}
	}
	//!< TOI でソート
	std::ranges::sort(Contacts, std::ranges::less{}, &Collision::Contact::TimeOfImpact);
}
#endif

// 静的コンストレイント、動的コンストレイントを解決
void Physics::Scene::SolveConstraint(const float DeltaSec, const uint32_t ItCount)
{
	for (auto& i : Constraints) {
		i->PreSolve(DeltaSec);
	}
	Manifolds.PreSolve(DeltaSec);

	//!< Solve() は繰り返すことで収束させる (１度には２剛体間のコンストレイントしか解決しない為)
	for (uint32_t c = 0; c < ItCount; ++c) {
		for (auto& i : Constraints) {
			i->Solve();
		}
		Manifolds.Solve();
	}

	for (auto& i : Constraints) {
		i->PostSolve();
	}
	Manifolds.PostSolve();
}

void Physics::Scene::Update(const float DeltaSec)
{
	Manifolds.RemoveExpired();

	//!< 重力
	for (auto& i : RigidBodies) {
		i->ApplyGravity(DeltaSec);
	}

	//!< 衝突 Contacts を収集
	std::vector<Collision::Contact> Contacts;
	{
#ifdef USE_BRUTE_FORCE
		//!< 総当たり (ブルートフォース)
		BruteForce(DeltaSec, Contacts);
#else
		//!< ブロードフェーズ、ナローフェーズ
		std::vector<CollidablePair> CollidablePairs;
		BroadPhase(DeltaSec, CollidablePairs);
#ifdef _DEBUG
		{
			//!< BruteForce に比べてどのくらい削減できているか
			const auto BFC = BruteForceCount();
			LOG(std::data(std::format("Collidable / BruteForce = {} / {} = {} %\n", std::size(CollidablePairs), BFC, std::size(CollidablePairs) * 100 / BFC)));
		}
#endif
		NarrowPhase(DeltaSec, CollidablePairs, Contacts);
#endif
	}

	//!< コンストレイントの解決
	SolveConstraint(DeltaSec, 5);

	//!< TOI 毎に時間をスライスして、シミュレーションを進める
	auto AccumTime = 0.0f;
	for (const auto& i : Contacts) {
		//!< 次の衝突までの時間
		const auto DeltaTime = i.TimeOfImpact - AccumTime;

		//!< 次の衝突までシミュレーションを進める
		for (auto& j : RigidBodies) {
			j->Update(DeltaTime);
		}

		//!< 衝突の解決
		ResolveContact(i);

		AccumTime += DeltaTime;
	}

	//!< 残りのシミュレーションを進める
	const auto DeltaTime = DeltaSec - AccumTime;
	if (0.0f < DeltaTime) {
		for (auto& i : RigidBodies) {
			i->Update(DeltaTime);
		}
	}

	//!< 無限落下防止
#if 1
	for (auto& i : RigidBodies) {
		if (i->Position.Y() < -100.0f) {
			i->Position[1] = -100.0f;
			i->InvMass = 0.0f;
			i->LinearVelocity = i->AngularVelocity = Math::Vec3::Zero();
		}
	}
#endif
}