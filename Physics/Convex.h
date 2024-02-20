#pragma once

#include "Math.h"
using namespace Math;

#include <algorithm>

#include "Shape.h"
#include "RigidBody.h"
using namespace Physics;

namespace Convex
{
	//!< 頂点を「なるべく」包含するような四面体を作成
	static void BuildTetrahedron(const std::vector<Vec3>& Pts, std::vector<Vec3>& HullVerts, std::vector<TriInds>& HullInds)
	{
		//!< 特定の軸(ここではX)に一番遠い点
		std::array<Vec3, 4> P = { *Distance::Farthest(Pts, Vec3::AxisX()) };
		//< 前出の逆向きの軸軸に一番遠い点
		P[1] = *Distance::Farthest(Pts, -P[0]);
		//!< 前出の 2 点からなる線分に一番遠い点
		P[2] = *Distance::Farthest(Pts, P[0], P[1]);
		//!< 前出の 3 点からなる三角形に一番遠い点
		P[3] = *Distance::Farthest(Pts, P[0], P[1], P[2]);

		//!< CCW になるように調整
		if (Distance::PointTriangle(P[0], P[1], P[2], P[3]) > 0.0f) {
			std::swap(P[0], P[1]);
		}

		//!< 四面体の頂点
		HullVerts.emplace_back(P[0]);
		HullVerts.emplace_back(P[1]);
		HullVerts.emplace_back(P[2]);
		HullVerts.emplace_back(P[3]);

		//!< 四面体のインデックス
		HullInds.emplace_back(TriInds({ 0, 1, 2 }));
		HullInds.emplace_back(TriInds({ 0, 2, 3 }));
		HullInds.emplace_back(TriInds({ 2, 1, 3 }));
		HullInds.emplace_back(TriInds({ 1, 0, 3 }));
	}

	//!< 指定の点が凸包の内部点かどうか
	[[nodiscard]] static bool IsInternal(const Vec3& Pt, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices)
	{
		//!< 全ての三角形に対し、負の側にあれば内部点
		return std::ranges::all_of(Indices, [&](const auto rhs) {
			return Distance::PointTriangle(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]) <= 0.0f;
		});
	}
	//!< 凸包の内部点を削除
	static void RemoveInternal(const std::vector<Vec3>& HullVerts, const std::vector<TriInds>& HullInds, std::vector<Vec3>& Pts)
	{
		//!< 内部点を除外
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				//!< 三角形に対し負の側にあれば内部点
				return std::ranges::all_of(HullInds, [&](const auto rhs) {
					return Distance::PointTriangle(Pt, HullVerts[rhs[0]], HullVerts[rhs[1]], HullVerts[rhs[2]]) <= 0.0f;
				});
			});
			Pts.erase(std::begin(Range), std::end(Range));
		}

		//!< 既存点と同一とみなせる点は除外
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				//!< 同一とみなせる点
				return std::ranges::any_of(HullVerts, [&](const auto rhs) {
					return rhs.NearlyEqual(Pt);
				});
			});
			Pts.erase(std::begin(Range), std::end(Range));
		}
	}
	//!< ハイポリを食わせるとかなり時間がかかる上に結局ハイポリの凸包ができるだけなのでコリジョンとして現実的ではない、ローポリを食わせること
	//!< 検証済
	static void BuildConvexHull(const std::vector<Vec3>& Pts, std::vector<Vec3>& HullVerts, std::vector<TriInds>& HullInds)
	{
		LOG(data(std::format("Building convex hull...\n")));

		//!< まずは「なるべく」包含するような四面体を作成
		BuildTetrahedron(Pts, HullVerts, HullInds);

		//!< 内部点の除外 -> 外部点が残る
		auto External = Pts;
		RemoveInternal(HullVerts, HullInds, External);

		//!< 外部点が無くなるまで繰り返す
		while (!std::empty(External)) {
			LOG(data(std::format("Rest vertices = {}\n", size(External))));

			//!< 最遠点を見つける
			const auto ExFarIt = Distance::Farthest(External, External[0]);

			//!< 最遠店を向いている三角形 (A とする) と、向いていない三角形 (B とする) の境界となる辺を収集します
			std::vector<EdgeIndsCount> EdgeCounts;
			{
				//!< B を前方、A を後方に分割する
				const auto Range = std::ranges::partition(HullInds, [&](const auto& i) {
					return Distance::PointTriangle(*ExFarIt, HullVerts[i[0]], HullVerts[i[1]], HullVerts[i[2]]) <= 0.0f;
				});

				//!< A と B の境界となる辺を収集する (A の中から他の三角形と辺を共有しないユニークな辺のみを収集すれば良い)
				std::for_each(std::begin(Range), std::end(HullInds), [&](const auto& i) {
					const std::array Edges = {
						EdgeInds({ i[0], i[1] }),
						EdgeInds({ i[1], i[2] }),
						EdgeInds({ i[2], i[0] }),
					};
					for (auto& j : Edges) {
						//!< 既出の辺かどうかを調べる (真逆も既出として扱う)
						const auto It = std::ranges::find_if(EdgeCounts, [&](const auto& rhs) {
							return (rhs.first[0] == j[0] && rhs.first[1] == j[1]) || (rhs.first[0] == j[1] && rhs.first[1] == j[0]);
						});
						if (std::end(EdgeCounts) == It) {
							//!< 新規の辺なのでカウンタゼロとして追加
							EdgeCounts.emplace_back(EdgeIndsCount({ j, 0 }));
						}
						else {
							//!< (追加済みの辺が) 既出の辺となったらカウンタをインクリメントして情報を更新しておく
							++It->second;
						}
					}
				});
				//!< (辺は収集済みなので) ここまで来たら A は削除してよい  
				HullInds.erase(std::begin(Range), std::end(HullInds));
			}

			//!< 凸包の更新
			{
				//!<【バーテックス】最遠点を頂点として追加する
				HullVerts.emplace_back(*ExFarIt);

				//!<【インデックス】最遠点とユニーク辺からなる三角形群を追加
				const auto FarIndex = static_cast<uint32_t>(std::size(HullVerts) - 1); //!< (さっき追加した) 最後の要素が最遠点のインデックス

				//!< 非ユニークな辺 (カウンタが 0 より大きい) を削除
				const auto Range = std::ranges::remove_if(EdgeCounts, [](const auto& lhs) { return lhs.second > 0; });
				EdgeCounts.erase(std::begin(Range), std::end(EdgeCounts));

				//!< ユニークな辺と最遠点からなる三角形を追加
				std::ranges::for_each(EdgeCounts, [&](const auto& i) {
					HullInds.emplace_back(TriInds({ i.first[0], i.first[1], FarIndex }));
				});
			}

			//!< 外部点の更新
			{
				//!< ここまで済んだら最遠点は削除してよい
				External.erase(ExFarIt);

				//!< 更新した凸包に対して内部点を削除する
				RemoveInternal(HullVerts, HullInds, External);
			}
		}
	}

	//!< #TODO 要検証
	[[nodiscard]] static Vec3 CalcCenterOfMass(const AABB& Aabb, const std::vector<Vec3>& HullVerts, const std::vector<TriInds>& HullInds) {
		constexpr auto SampleCount = 100;

		auto Sampled = 0;
		auto CenterOfMass = Vec3::Zero();
		const Vec3 d = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					const auto Pt = Vec3(Aabb.Min.X() + d.X() * x, Aabb.Min.Y() + d.Y() * y, Aabb.Min.Z() + d.Z() * z);
					if (IsInternal(Pt, HullVerts, HullInds)) {
						CenterOfMass += Pt;
						++Sampled;
					}
				}
			}
		}
		return CenterOfMass / static_cast<float>(Sampled);
	}
	//!< #TODO 要検証
	[[nodiscard]] static Mat3 CalcInertiaTensor(const AABB& Aabb, const std::vector<Vec3>& HullVerts, const std::vector<TriInds>& HullInds, const Vec3& CenterOfMass) {
		constexpr auto SampleCount = 100;

		auto Sampled = 0;
		auto InertiaTensor = Mat3::Zero();
		const Vec3 d = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					const auto Pt = Vec3(Aabb.Min.X() + d.Z() * x, Aabb.Min.Y() + d.Y() * y, Aabb.Min.Z() + d.Z() * z) - CenterOfMass;
					if (IsInternal(Pt, HullVerts, HullInds)) {

						InertiaTensor[0][0] += Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z();
						InertiaTensor[1][1] += Pt.Z() * Pt.Z() + Pt.X() * Pt.X();
						InertiaTensor[2][2] += Pt.X() * Pt.X() + Pt.Y() * Pt.Y();

						InertiaTensor[0][1] += -Pt.X() * Pt.Y();
						InertiaTensor[0][2] += -Pt.X() * Pt.Z();
						InertiaTensor[1][2] += -Pt.Y() * Pt.Z();

						InertiaTensor[1][0] += -Pt.X() * Pt.Y();
						InertiaTensor[2][0] += -Pt.X() * Pt.Z();
						InertiaTensor[2][1] += -Pt.Y() * Pt.Z();

						++Sampled;
					}
				}
			}
		}
		return InertiaTensor / static_cast<float>(Sampled);
	}
	[[nodiscard]] static Mat3 CalcInertiaTensor(const AABB& Aabb, const std::vector<Vec3>& HullVerts, const std::vector<TriInds>& HullInds) {
		return CalcInertiaTensor(Aabb, HullVerts, HullInds, CalcCenterOfMass(Aabb, HullVerts, HullInds));
	}
}
