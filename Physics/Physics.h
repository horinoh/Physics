#pragma once

template<typename T> static [[nodiscard]] size_t IndexOf(const std::vector<T>& Vector, const T& rhs) { 
	return static_cast<size_t>(&rhs - &*std::begin(Vector));
}

using TriInds = std::array<uint32_t, 3>;
using EdgeInds = std::array<uint32_t, 2>;
using EdgeIndsCount = std::pair<EdgeInds, uint32_t>;

#include "Scene.h"
