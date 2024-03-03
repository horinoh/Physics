#pragma once

#include <algorithm>
#include <format>
#include <vector>

#include "Physics.h"
#include "RigidBody.h"
#include "Collision.h"
#include "PhysicsMath.h"

namespace Convex
{
	//!< 頂点を「なるべく」包含するような四面体を作成
	static void BuildTetrahedron(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds > & Indices)
	{
		//!< 特定の軸 (ここではX) に一番遠い点
		std::array<Math::Vec3, 4> P = { *Collision::Distance::Farthest(Pts, Math::Vec3::AxisX()) };
		//< 前出の逆向きの軸軸に一番遠い点
		P[1] = *Collision::Distance::Farthest(Pts, -P[0]);
		//!< 前出の 2 点からなる線分に一番遠い点
		P[2] = *Collision::Distance::Farthest(Pts, P[0], P[1]);
		//!< 前出の 3 点からなる三角形に一番遠い点
		P[3] = *Collision::Distance::Farthest(Pts, P[0], P[1], P[2]);

		//!< CCW になるように調整
		if (Collision::Distance::IsFront(P[0], P[1], P[2], P[3])) {
			std::swap(P[0], P[1]);
		}

		//!< 四面体の頂点
		Vertices.emplace_back(P[0]);
		Vertices.emplace_back(P[1]);
		Vertices.emplace_back(P[2]);
		Vertices.emplace_back(P[3]);

		//!< 四面体のインデックス
		Indices.emplace_back(Collision::TriInds({ 0, 1, 2 }));
		Indices.emplace_back(Collision::TriInds({ 0, 2, 3 }));
		Indices.emplace_back(Collision::TriInds({ 2, 1, 3 }));
		Indices.emplace_back(Collision::TriInds({ 1, 0, 3 }));
	}

	//!< 指定の点が凸包の内部点かどうか
	[[nodiscard]] static bool IsInternal(const Math::Vec3& Pt, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices)
	{
		//!< 全ての三角形に対し、負の側にあれば内部点
		return std::ranges::all_of(Indices, [&](const auto rhs) {
			return !Collision::Distance::IsFront(Pt, Vertices[rhs[0]], Vertices[rhs[1]], Vertices[rhs[2]]);
		});
	}
	//!< 凸包の内部点を削除
	static void RemoveInternal(const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, std::vector<Math::Vec3>& Pts)
	{
		//!< 内部点を除外
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				return IsInternal(Pt, Vertices, Indices);
			});
			Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}

		//!< 既存と同一とみなせる点は除外
		{
			const auto Range = std::ranges::remove_if(Pts, [&](const auto& Pt) {
				//!< 同一とみなせる点
				return std::ranges::any_of(Vertices, [&](const auto rhs) {
					return rhs.NearlyEqual(Pt);
				});
			});
			Pts.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
		}
	}
	static void CollectUniqueEdges(std::vector<Collision::TriInds>::const_iterator Begin, std::vector<Collision::TriInds>::const_iterator End, std::vector<Collision::EdgeIndsCount>& EdgeCounts)
	{
		std::for_each(Begin, End, [&](const auto& i) {
			const std::array Edges = {
				Collision::EdgeInds({ i[0], i[1] }),
				Collision::EdgeInds({ i[1], i[2] }),
				Collision::EdgeInds({ i[2], i[0] }),
			};
			for (auto& j : Edges) {
				//!< 既出の辺かどうかを調べる (真逆も既出として扱う)
				const auto It = std::ranges::find_if(EdgeCounts, [&](const auto& k) {
					return (k.first[0] == j[0] && k.first[1] == j[1]) || (k.first[0] == j[1] && k.first[1] == j[0]);
				});
				if (std::cend(EdgeCounts) == It) {
					//!< 新規の辺なのでカウンタゼロとして追加
					EdgeCounts.emplace_back(Collision::EdgeIndsCount({j, 0}));
				}
				else {
					//!< (追加済みの辺が) 既出の辺となったらカウンタをインクリメントして情報を更新しておく
					++It->second;
				}
			}
		});

		//!< ユニークでない辺 (カウンタが 0 より大きい) を削除
		const auto Range = std::ranges::remove_if(EdgeCounts, [](const auto& i) { return i.second > 0; });
		EdgeCounts.erase(std::ranges::cbegin(Range), std::ranges::cend(EdgeCounts));
	}
	static void CollectUniqueEdges(const std::vector<Collision::TriInds>& Indices, std::vector<Collision::EdgeIndsCount>& EdgeCounts) { CollectUniqueEdges(std::ranges::cbegin(Indices), std::ranges::cend(Indices), EdgeCounts); }

#if 0
	static void BuildConvexHull(const std::vector<Vec3>& Pts, std::vector<Vec3>& Vertices, std::vector<TriInds>& Indices);
#else
	//!< ハイポリを食わせるとかなり時間がかかる上に結局ハイポリの凸包ができるだけなのでコリジョンとして現実的ではない、ローポリを食わせること
	//!< 検証済
	static void BuildConvexHull(const std::vector<Math::Vec3>& Pts, std::vector<Math::Vec3>& Vertices, std::vector<Collision::TriInds>& Indices)
	{
		//LOG(data(std::format("Building convex hull...\n")));

		//!< まずは「なるべく」包含するような四面体を作成
		BuildTetrahedron(Pts, Vertices, Indices);

		//!< 内部点の除外 -> 外部点が残る
		auto External = Pts;
		RemoveInternal(Vertices, Indices, External);

		//!< 外部点が無くなるまで繰り返す
		while (!std::empty(External)) {
			//LOG(data(std::format("Rest vertices = {}\n", size(External))));

			//!< 最遠点を見つける
			const auto ExFarIt = Collision::Distance::Farthest(External, External[0]);

			std::vector<Collision::EdgeIndsCount> EdgeCounts;
			{
				//!< 最遠点を向いていない三角形 (A) と、向いている三角形 (B) に分割
				//!< partition は以下のように返す
				//!<	A ラムダ式が true	: [begin(Indices), begin(Range)]
				//!<	B ラムダ式が false	: [begin(Range), end(Range)]
				const auto Range = std::ranges::partition(Indices, [&](const auto& i) {
					return !Collision::Distance::IsFront(*ExFarIt, Vertices[i[0]], Vertices[i[1]], Vertices[i[2]]);
				});

				//!< A, B の境界となるような辺を収集する (B の中から他の三角形と辺を共有しないユニークな辺のみを収集すれば良い)
				CollectUniqueEdges(std::ranges::cbegin(Range), std::ranges::cend(Range), EdgeCounts);

				//!< (辺は収集済) ここまで来たら B は削除してよい  
				Indices.erase(std::ranges::cbegin(Range), std::ranges::cend(Range));
			}

			//!< 凸包の更新
			{
				//!< 最遠点を頂点として追加する
				Vertices.emplace_back(*ExFarIt);

				//!< 最遠点のインデックス
				const auto FarIndex = static_cast<uint32_t>(std::size(Vertices) - 1);
				//!< 最遠点とユニーク辺からなる三角形群を追加
				std::ranges::transform(EdgeCounts, std::back_inserter(Indices), [&](const auto& i) {
					return Collision::TriInds({ i.first[0], i.first[1], FarIndex });
				});
			}

			//!< 外部点の更新
			{
				//!< ここまで済んだら最遠点は削除してよい
				External.erase(ExFarIt);

				//!< 更新した凸包に対して内部点を削除する
				RemoveInternal(Vertices, Indices, External);
			}
		}
	}
#endif

	//!< #TODO 要検証
	[[nodiscard]] static Math::Vec3 CalcCenterOfMass(const Collision::AABB & Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices) {
		//!< 各軸にサンプリングする個数
		constexpr auto SampleCount = 100;

		auto CenterOfMass = Math::Vec3::Zero();
		auto Sampled = 0;
		const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					//!< AABB 内のサンプル点
					const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z);
					if (IsInternal(Pt, Vertices, Indices)) {
						//!< 内部点なら収集
						CenterOfMass += Pt;
						++Sampled;
					}
				}
			}
		}
		return CenterOfMass / static_cast<float>(Sampled);
	}
	//!< #TODO 要検証
	[[nodiscard]] static Math::Mat3 CalcInertiaTensor(const Collision::AABB& Aabb, const std::vector<Math::Vec3>& Vertices, const std::vector<Collision::TriInds>& Indices, const Math::Vec3& CenterOfMass) {
		//!< 各軸にサンプリングする個数
		constexpr auto SampleCount = 100;

		auto InertiaTensor = Math::Mat3::Zero();
		auto Sampled = 0;
		const auto Delta = Aabb.GetExtent() / static_cast<float>(SampleCount);
		for (auto x = 0; x < SampleCount; ++x) {
			for (auto y = 0; y < SampleCount; ++y) {
				for (auto z = 0; z < SampleCount; ++z) {
					//!< AABB 内のサンプル点 (重心からの相対)
					const auto Pt = Aabb.Min + Math::Vec3(Delta.X() * x, Delta.Y() * y, Delta.Z() * z) - CenterOfMass;
					if (IsInternal(Pt, Vertices, Indices)) {
						//!< 内部点なら収集する
#if 0
						InertiaTensor[0][0] += Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z();
						InertiaTensor[1][1] += Pt.Z() * Pt.Z() + Pt.X() * Pt.X();
						InertiaTensor[2][2] += Pt.X() * Pt.X() + Pt.Y() * Pt.Y();

						InertiaTensor[0][1] += -Pt.X() * Pt.Y();
						InertiaTensor[0][2] += -Pt.X() * Pt.Z();
						InertiaTensor[1][2] += -Pt.Y() * Pt.Z();

						InertiaTensor[1][0] += -Pt.X() * Pt.Y();
						InertiaTensor[2][0] += -Pt.X() * Pt.Z();
						InertiaTensor[2][1] += -Pt.Y() * Pt.Z();
#else
						InertiaTensor += {
							{ Pt.Y() * Pt.Y() + Pt.Z() * Pt.Z(), -Pt.X() * Pt.Y(), -Pt.X() * Pt.Z() },
							{ -Pt.X() * Pt.Y(), Pt.Z() * Pt.Z() + Pt.X() * Pt.X(), -Pt.Y() * Pt.Z() },
							{ -Pt.X() * Pt.Z(), -Pt.Y() * Pt.Z(), Pt.X() * Pt.X() + Pt.Y() * Pt.Y() },
						};
#endif
						++Sampled;
					}
				}
			}
		}
		return InertiaTensor / static_cast<float>(Sampled);
	}
	//[[nodiscard]] static Mat3 CalcInertiaTensor(const AABB& Aabb, const std::vector<Vec3>& Vertices, const std::vector<TriInds>& Indices) {
	//	return CalcInertiaTensor(Aabb, Vertices, Indices, CalcCenterOfMass(Aabb, Vertices, Indices));
	//}
}
