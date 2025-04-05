#pragma once

#include <vector>

template<typename T> static [[nodiscard]] size_t IndexOf(const std::vector<T>& Vector, const T& rhs) {
	return static_cast<size_t>(&rhs - &*std::cbegin(Vector));
}