#include "Shape.h"
#include "Convex.h"

#include "Log.h"

Physics::ShapeConvex::ShapeConvex(const std::vector<LinAlg::Vec3>& Mesh) 
{
	//!< ���b�V�����_����ʖC���_�����
	Convex::BuildConvexHull(Mesh, Vertices, Indices);
}

LinAlg::Vec3 Physics::ShapeConvex::CalcCenterOfMass() const 
{
#if false
	//!< ��l�����_������d�S���쐬
	return Convex::Uniform::CalcCenterOfMass(Collision::AABB(Vertices), Vertices, Indices);
#elif false
	//!< �����e�J��������d�S���쐬
	return Convex::MonteCarlo::CalcCenterOfMass(Collision::AABB(Vertices), Vertices, Indices);
#else
	//!< �l�ʑ̂���d�S���쐬
	return Convex::Tetrahedron::CalcCenterOfMass(Vertices, Indices);
#endif
}
LinAlg::Mat3 Physics::ShapeConvex::CalcInertiaTensor() const
{
#if false
	//!< ��l�����_�����犵���e���\�����쐬
	PERFORMANCE_COUNTER("InertiaTensor : Uniform");
	return Convex::Uniform::CalcInertiaTensor(Collision::AABB(Vertices), Vertices, Indices, GetCenterOfMass());
#elif false
	//!< �����e�J�������犵���e���\�����쐬
	PERFORMANCE_COUNTER("InertiaTensor : MonteCarlo");
	return Convex::MonteCarlo::CalcInertiaTensor(Collision::AABB(Vertices), Vertices, Indices, GetCenterOfMass());
#else
	//!< �l�ʑ̂��犵���e���\�����쐬
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
