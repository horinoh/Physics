#pragma once

#include <iostream>
#include <numbers>

#define TO_RADIAN(x) ((x) * std::numbers::pi_v<float> / 180.0f)
#define TO_DEGREE(x) ((x) * 180.0f / std::numbers::pi_v<float>)

namespace Math 
{
	using Component2 = std::array<float, 2>;
	using Component3 = std::array<float, 3>;
	using Component4 = std::array<float, 4>;
}

#include "Vec.h"
#include "Mat.h"
#include "Quat.h"

static [[nodiscard]] Math::Vec2 operator*(const float lhs, const Math::Vec2& rhs) { return rhs * lhs; }
static [[nodiscard]] Math::Vec3 operator*(const float lhs, const Math::Vec3& rhs) { return rhs * lhs; }
static [[nodiscard]] Math::Vec4 operator*(const float lhs, const Math::Vec4& rhs) { return rhs * lhs; }

static [[nodiscard]] Math::Mat2 operator*(const float lhs, const Math::Mat2& rhs) { return rhs * lhs; }
static [[nodiscard]] Math::Mat3 operator*(const float lhs, const Math::Mat3& rhs) { return rhs * lhs; }
static [[nodiscard]] Math::Mat4 operator*(const float lhs, const Math::Mat4& rhs) { return rhs * lhs; }

static [[nodiscard]] Math::Quat operator*(const float lhs, const Math::Quat& rhs) { return rhs * lhs; }

#ifdef _DEBUG
static std::ostream& operator<<(std::ostream& lhs, const Math::Vec2& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << std::endl; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const Math::Vec3& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << rhs.Z() << ", " << std::endl; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const Math::Vec4& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << rhs.Z() << ", " << rhs.W() << ", " << std::endl; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const Math::Mat2& rhs) { lhs << rhs[0] << rhs[1]; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const Math::Mat3& rhs) { lhs << rhs[0] << rhs[1] << rhs[2];  return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const Math::Mat4& rhs) { lhs << rhs[0] << rhs[1] << rhs[2] << rhs[3]; return lhs; }
static std::ostream& operator<<(std::ostream& lhs, const Math::Quat& rhs) { lhs << rhs.X() << ", " << rhs.Y() << ", " << rhs.Z() << ", " << rhs.W() << ", " << std::endl; return lhs; }
#endif