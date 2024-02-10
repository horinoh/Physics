#pragma once

template<typename T> static [[nodiscard]] size_t IndexOf(const std::vector<T>& Vector, const T& rhs) { 
	return static_cast<size_t>(&rhs - &*std::begin(Vector));
}

#include "Scene.h"
