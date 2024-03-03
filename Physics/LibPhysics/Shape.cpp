#include "Shape.h"

void Physics::CreateVertices_Diamond(std::vector<Math::Vec3>& Dst)
{
	Dst.reserve(7 * 8);

	std::vector<Math::Vec3> Pts;
	Pts.reserve(4 + 3);

	Pts.emplace_back(Math::Vec3(0.1f, 0.0f, -1.0f));
	Pts.emplace_back(Math::Vec3(1.0f, 0.0f, 0.0f));
	Pts.emplace_back(Math::Vec3(1.0f, 0.0f, 0.1f));
	Pts.emplace_back(Math::Vec3(0.4f, 0.0f, 0.4f));

	constexpr auto Rad = 2.0f * std::numbers::pi_v<float> *0.125f;
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
