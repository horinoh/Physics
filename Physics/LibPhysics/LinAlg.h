#pragma once

#include <iostream>
#include <numbers>

namespace LinAlg 
{
	using Comp2 = std::array<float, 2>;
	using Comp3 = std::array<float, 3>;
	using Comp4 = std::array<float, 4>;

	static constexpr float ToRadian(const float Degree) { return Degree * std::numbers::pi_v<float> / 180.0f; }
	static constexpr float ToDegree(const float Radian) { return Radian * 180.0f / std::numbers::pi_v<float>; }
}

#include "Vec.h"
#include "Mat.h"
#include "Quat.h"

static [[nodiscard]] LinAlg::Vec2 operator*(const float lhs, const LinAlg::Vec2& rhs) { return rhs * lhs; }
static [[nodiscard]] LinAlg::Vec3 operator*(const float lhs, const LinAlg::Vec3& rhs) { return rhs * lhs; }
static [[nodiscard]] LinAlg::Vec4 operator*(const float lhs, const LinAlg::Vec4& rhs) { return rhs * lhs; }

static [[nodiscard]] LinAlg::Mat2 operator*(const float lhs, const LinAlg::Mat2& rhs) { return rhs * lhs; }
static [[nodiscard]] LinAlg::Mat3 operator*(const float lhs, const LinAlg::Mat3& rhs) { return rhs * lhs; }
static [[nodiscard]] LinAlg::Mat4 operator*(const float lhs, const LinAlg::Mat4& rhs) { return rhs * lhs; }

static [[nodiscard]] LinAlg::Quat operator*(const float lhs, const LinAlg::Quat& rhs) { return rhs * lhs; }

#ifdef _DEBUG
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Vec2& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << std::endl; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Vec3& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << rhs.Z() << ", " << std::endl; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Vec4& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << rhs.Z() << ", " << rhs.W() << ", " << std::endl; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Mat2& rhs) { lhs << rhs[0] << rhs[1]; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Mat3& rhs) { lhs << rhs[0] << rhs[1] << rhs[2];  return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Mat4& rhs) { lhs << rhs[0] << rhs[1] << rhs[2] << rhs[3]; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const LinAlg::Quat& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << rhs.Z() << ", " << rhs.W() << ", " << std::endl; return lhs; }
#endif