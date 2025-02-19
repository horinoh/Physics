#include "Shape.h"
#include "Convex.h"

#include "Log.h"

Physics::ShapeConvex& Physics::ShapeConvex::Init(const std::vector<Math::Vec3>& MeshVert)
{
	Convex::BuildConvexHull(MeshVert, Vertices, Indices);

#if false
	const auto Aabb = Collision::AABB(Vertices);
	CenterOfMass = Convex::Uniform::CalcCenterOfMass(Aabb, Vertices, Indices);
	InertiaTensor = Convex::Uniform::CalcInertiaTensor(Aabb, Vertices, Indices, CenterOfMass);
#elif false
	CenterOfMass = Convex::MonteCarlo::CalcCenterOfMass(Aabb, Vertices, Indices);
	InertiaTensor = Convex::MonteCarlo::CalcInertiaTensor(Aabb, Vertices, Indices, CenterOfMass);
#else
	CenterOfMass = Convex::Tetrahedron::CalcCenterOfMass(Vertices, Indices);
	InertiaTensor = Convex::Tetrahedron::CalcInertiaTensor(Vertices, Indices, CenterOfMass);
#endif

	LOG(std::data(std::string("CenterOfMass = \n")));
	LOG(std::data(CenterOfMass.ToString()));

	LOG(std::data(std::string("InertiaTensor = \n")));
	LOG(std::data(InertiaTensor.ToString()));

	InvInertiaTensor = InertiaTensor.Inverse();

	return *this;
}
void Physics::CreateVertices_Box(std::vector<Math::Vec3>& Dst, const float W, const float H, const float D)
{
	static const std::array Box = {
		Math::Vec3(-W, -H, -D),
		Math::Vec3(W, -H, -D),
		Math::Vec3(-W, H, -D),
		Math::Vec3(W, H, -D),

		Math::Vec3(-W, -H, D),
		Math::Vec3(W, -H, D),
		Math::Vec3(-W, H, D),
		Math::Vec3(W, H, D),
	};
	Dst.reserve(std::size(Box));
	std::ranges::copy(Box, std::back_inserter(Dst));
}
void Physics::CreateVertices_Diamond(std::vector<Math::Vec3>& Dst)
{
	Dst.reserve(7 * 8);

	std::vector<Math::Vec3> Pts;
	Pts.reserve(4 + 3);

	Pts.emplace_back(Math::Vec3(0.1f, 0.0f, -1.0f));
	Pts.emplace_back(Math::Vec3(1.0f, 0.0f, 0.0f));
	Pts.emplace_back(Math::Vec3(1.0f, 0.0f, 0.1f));
	Pts.emplace_back(Math::Vec3(0.4f, 0.0f, 0.4f));

	constexpr auto Rad = 2.0f * std::numbers::pi_v<float> * 0.125f;
	const auto QuatHalf = Math::Quat(Math::Vec3(0.0f, 0.0f, 1.0f), Rad * 0.5f);
	Pts.emplace_back(QuatHalf.Rotate(Math::Vec3(0.8f, 0.0f, 0.3f)));
	Pts.emplace_back(QuatHalf.Rotate(Pts[1]));
	Pts.emplace_back(QuatHalf.Rotate(Pts[2]));

	std::ranges::copy(Pts, std::back_inserter(Dst));

	const Math::Quat Q(Math::Vec3(0.0f, 0.0f, 1.0f), Rad);
	Math::Quat QuatAccumulator;
	for (auto i = 1; i < 8; ++i) {
		QuatAccumulator = QuatAccumulator * Q;
		for (auto j = 0; j < 7; ++j) {
			Dst.emplace_back(QuatAccumulator.Rotate(Pts[j]));
		}
	}
}
