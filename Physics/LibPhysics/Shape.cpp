#include "Shape.h"
#include "Convex.h"

#include "Log.h"

Physics::ShapeConvex::ShapeConvex(const std::vector<LinAlg::Vec3>& Mesh) 
{
	//!< メッシュ頂点から凸砲頂点を作る
	Convex::BuildConvexHull(Mesh, Vertices, Indices);
}

LinAlg::Vec3 Physics::ShapeConvex::CalcCenterOfMass() const 
{
#if false
	//!< 一様ランダムから重心を作成
	return Convex::Uniform::CalcCenterOfMass(Collision::AABB(Vertices), Vertices, Indices);
#elif false
	//!< モンテカルロから重心を作成
	return Convex::MonteCarlo::CalcCenterOfMass(Collision::AABB(Vertices), Vertices, Indices);
#else
	//!< 四面体から重心を作成
	return Convex::Tetrahedron::CalcCenterOfMass(Vertices, Indices);
#endif
}
LinAlg::Mat3 Physics::ShapeConvex::CalcInertiaTensor() const
{
#if false
	//!< 一様ランダムから慣性テンソルを作成
	PERFORMANCE_COUNTER("InertiaTensor : Uniform");
	return Convex::Uniform::CalcInertiaTensor(Collision::AABB(Vertices), Vertices, Indices, GetCenterOfMass());
#elif false
	//!< モンテカルロから慣性テンソルを作成
	PERFORMANCE_COUNTER("InertiaTensor : MonteCarlo");
	return Convex::MonteCarlo::CalcInertiaTensor(Collision::AABB(Vertices), Vertices, Indices, GetCenterOfMass());
#else
	//!< 四面体から慣性テンソルを作成
	PERFORMANCE_COUNTER("InertiaTensor : Tetrahedron");
	return Convex::Tetrahedron::CalcInertiaTensor(Vertices, Indices, GetCenterOfMass());
#endif
}

void Physics::CreateVertices_Diamond(std::vector<LinAlg::Vec3>& Dst)
{
	Dst.reserve(7 * 8);

	std::vector<LinAlg::Vec3> Pts;
	Pts.reserve(4 + 3);

	Pts.emplace_back(LinAlg::Vec3(0.1f, 0.0f, -1.0f));
	Pts.emplace_back(LinAlg::Vec3(1.0f, 0.0f, 0.0f));
	Pts.emplace_back(LinAlg::Vec3(1.0f, 0.0f, 0.1f));
	Pts.emplace_back(LinAlg::Vec3(0.4f, 0.0f, 0.4f));

	constexpr auto Rad = 2.0f * std::numbers::pi_v<float> * 0.125f;
	const auto QuatHalf = LinAlg::Quat(LinAlg::Vec3(0.0f, 0.0f, 1.0f), Rad * 0.5f);
	Pts.emplace_back(QuatHalf.Rotate(LinAlg::Vec3(0.8f, 0.0f, 0.3f)));
	Pts.emplace_back(QuatHalf.Rotate(Pts[1]));
	Pts.emplace_back(QuatHalf.Rotate(Pts[2]));

	std::ranges::copy(Pts, std::back_inserter(Dst));

	const LinAlg::Quat Q(LinAlg::Vec3(0.0f, 0.0f, 1.0f), Rad);
	LinAlg::Quat QuatAccumulator;
	for (auto i = 1; i < 8; ++i) {
		QuatAccumulator = QuatAccumulator * Q;
		for (auto j = 0; j < 7; ++j) {
			Dst.emplace_back(QuatAccumulator.Rotate(Pts[j]));
		}
	}
}
