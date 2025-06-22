#include "Shape.h"
#include "Convex.h"

#include "Log.h"

Physics::ShapeConvex& Physics::ShapeConvex::Init(const std::vector<LinAlg::Vec3>& Mesh)
{
	//!< メッシュ頂点から凸砲頂点を作る
	Convex::BuildConvexHull(Mesh, Vertices, Indices);

#if false
	//!< 一様ランダムから、重心、慣性テンソルを作成
	{
		PERFORMANCE_COUNTER("InertiaTensor : Uniform");
		const auto Ab = Collision::AABB(Vertices);
		CenterOfMass = Convex::Uniform::CalcCenterOfMass(Ab, Vertices, Indices);
		InertiaTensor = Convex::Uniform::CalcInertiaTensor(Ab, Vertices, Indices, CenterOfMass);
	}
#elif false
	//!< モンテカルロから、重心、慣性テンソルを作成
	{
		PERFORMANCE_COUNTER("InertiaTensor : MonteCarlo");
		const auto Ab = Collision::AABB(Vertices);
		CenterOfMass = Convex::MonteCarlo::CalcCenterOfMass(Ab, Vertices, Indices);
		InertiaTensor = Convex::MonteCarlo::CalcInertiaTensor(Ab, Vertices, Indices, CenterOfMass);
	}
#else
	//!< 四面体から、重心、慣性テンソルを作成
	{
		PERFORMANCE_COUNTER("InertiaTensor : Tetrahedron");
		CenterOfMass = Convex::Tetrahedron::CalcCenterOfMass(Vertices, Indices);
		InertiaTensor = Convex::Tetrahedron::CalcInertiaTensor(Vertices, Indices, CenterOfMass);
	}
#endif

	LOG(std::data(std::string("CenterOfMass = \n")));
	LOG(std::data(CenterOfMass.ToString()));

	LOG(std::data(std::string("InertiaTensor = \n")));
	LOG(std::data(InertiaTensor.ToString()));

	InvInertiaTensor = InertiaTensor.Inverse();

	return *this;
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
