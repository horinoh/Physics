#pragma once

namespace LinAlg 
{
	class Quat 
	{
	public:
		Quat() {}
		Quat(const float x, const float y, const float z, const float w) : Data({x, y, z, w}) {}
		Quat(const Vec3& rhs) : Data({ rhs.X(), rhs.Y(), rhs.Z(), 0.0f }) {}
		Quat(const Vec3& Axis, const float Radian) {
			const auto HalfRadian = 0.5f * Radian;
			const auto Ax = Axis.Normalize() * sinf(HalfRadian);
			Data = { Ax.X(),Ax.Y(),Ax.Z(), cosf(HalfRadian) };
		}
		Quat(View4 rhs) : View(rhs) {}

	private:
		static const Quat _Identity;
	public:
		inline static const Quat& Identity() { return _Identity; }

		inline static Mat4 ToLeftMat4(const Quat& rhs) {
			return {
				{ rhs.W(), -rhs.Z(), rhs.Y(), rhs.X() },
				{ rhs.Z(), rhs.W(), -rhs.X(), rhs.Y() },
				{ -rhs.Y(), rhs.X(), rhs.W(), rhs.Z() },
				{ -rhs.X(), -rhs.Y(), -rhs.Z(), rhs.W() },
			};
		}
		inline static Mat4 ToRightMat4(const Quat& rhs) {
			return {
				{ rhs.W(), rhs.Z(), -rhs.Y(), rhs.X() },
				{ -rhs.Z(), rhs.W(), rhs.X(), rhs.Y() },
				{ rhs.Y(), -rhs.X(), rhs.W(), rhs.Z() },
				{ -rhs.X(), -rhs.Y(), -rhs.Z(), rhs.W() },
			};
		}

		inline bool NearlyEqual(const Quat& rhs, const float Epsilon = (std::numeric_limits<float>::epsilon)()) const {
			return std::ranges::equal(Data, rhs.Data,
				[&](const float l, const float r) { 
					return std::abs(l - r) < Epsilon; 
				});
		}

		inline bool operator==(const Quat& rhs) const { 
			return std::ranges::equal(Data, rhs.Data);
		}
		inline bool operator!=(const Quat& rhs) const { return !(*this == rhs); }
		inline Quat operator*(const float rhs) const {
#ifdef USE_STD_LINALG
			return Quat(std::linalg::scale(rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Quat(Data[0] * rhs, Data[1] * rhs, Data[2] * rhs, Data[3] * rhs);
#else
			Quat r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}
		inline Quat operator*(const Quat& rhs) const {
			return Quat(X() * rhs.W() + W() * rhs.X() + Y() * rhs.Z() - Z() * rhs.Y(),
				Y() * rhs.W() + W() * rhs.Y() + Z() * rhs.X() - X() * rhs.Z(),
				Z() * rhs.W() + W() * rhs.Z() + X() * rhs.Y() - Y() * rhs.X(),
				W() * rhs.W() - X() * rhs.X() - Y() * rhs.Y() - Z() * rhs.Z());
		}
		inline Quat operator/(const float rhs) const { 
#ifdef USE_STD_LINALG
			return Quat(std::linalg::scale(1.0f / rhs, View));
#else
#ifdef USE_OWN_IMPL
			return Quat(Data[0] / rhs, Data[1] / rhs, Data[2] / rhs, Data[3] / rhs);
#else
			Quat r;
			std::ranges::transform(Data, std::begin(r.Data), std::bind(std::divides(), std::placeholders::_1, rhs));
			return r;
#endif
#endif
		}

		inline float X() const { return Data[0]; }
		inline float Y() const { return Data[1]; }
		inline float Z() const { return Data[2]; }
		inline float W() const { return Data[3]; }
		inline Vec3 Imag() const { return { X(), Y(), Z() }; }
		inline float Real() const { return W(); }
		inline Vec3 Axis() const { return Imag(); }
		inline float Radian() const { return std::acos(std::clamp(Real(), 0.0f, 1.0f)) * 2.0f; }
		inline float operator[](const int i) const { return Data[i]; }
		inline operator const float* () const { return std::data(Data); }
		inline operator Vec3() const { return { X(), Y(), Z() }; }
		inline operator Mat3() const {
			return {
				Rotate(Vec3::AxisX()),
				Rotate(Vec3::AxisY()),
				Rotate(Vec3::AxisZ()),
			};
		}
		inline Vec3 ToVec3() const { return static_cast<Vec3>(*this); }
		inline Mat3 ToMat3() const { return static_cast<Mat3>(*this); }
	
		inline Mat4 ToLeftMat4() const { return ToLeftMat4(*this); }
		inline Mat4 ToRightMat4() const { return ToRightMat4(*this); }

		inline float Dot(const Quat& rhs) const {
#ifdef USE_STD_LINALG
			return std::linalg::dot(View, rhs.View)
#else
			return std::inner_product(std::cbegin(Data), std::cend(Data), std::cbegin(rhs.Data), 0.0f);
#endif
		}
		inline float LengthSq() const { 
#ifdef USE_STD_LINALG
			return std::linalg::vector_sum_of_squares(View);
#else
			return Dot(*this); 
#endif
		}
		inline float Length() const {
#ifdef USE_STD_LINALG
			return std::linalg::vector_two_norm(View);
#else
			return std::sqrtf(LengthSq());
#endif
		}
		inline Quat Normalize() const {
#ifdef USE_STD_LINALG
			return Quat(std::linalg::scale(1.0f / std::linalg::vector_two_norm(View), View));
#else
			const auto Sq = LengthSq();
			if (Sq > (std::numeric_limits<float>::epsilon)()) {
				return *this / std::sqrtf(Sq);
			}
#ifdef _DEBUG
			//__debugbreak();
#endif
			return Quat(*this);
#endif
		}
		inline Quat Conjugate() const { return Quat(-X(), -Y(), -Z(), Real()); }
		inline Quat Inverse() const { return Conjugate() / LengthSq(); }
		inline Vec3 Rotate(const Vec3& rhs) const {
			return *this * Quat(rhs) * Inverse();
		}
		inline Mat3 Rotate(const Mat3& rhs) const {
			return {
				Rotate(rhs[0]),
				Rotate(rhs[1]),
				Rotate(rhs[2])
			};
		}

		inline Quat& operator=(const Quat& rhs) { 
#ifdef USE_STD_LINALG
			std::linalg::copy(rhs.View, View);
#else
			std::ranges::copy(rhs.Data, std::begin(Data));
#endif
			return *this;
		}
		inline const Quat& operator*=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] *= rhs;
			Data[1] *= rhs;
			Data[2] *= rhs;
			Data[3] *= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::multiplies(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline const Quat& operator/=(const float rhs) {
#ifdef USE_STD_LINALG
			std::linalg::scale(1.0f / rhs, View);
#else
#ifdef USE_OWN_IMPL
			Data[0] /= rhs;
			Data[1] /= rhs;
			Data[2] /= rhs;
			Data[3] /= rhs;
#else
			std::ranges::transform(Data, std::begin(Data), std::bind(std::divides(), std::placeholders::_1, rhs));
#endif
#endif
			return *this; 
		}
		inline float& operator[](const int i) { return Data[i]; }
		inline operator float* () { return std::data(Data); }
		inline operator const Float4& () const { return Data; }
		inline operator Float4& () { return Data; }

		inline Quat& ToIdentity() { return (*this = Identity()); }
		
		inline std::string ToString() const { return std::format("({:1.4f}, {:1.4f}, {:1.4f}, {:1.4f})\n", X(), Y(), Z(), W()); }

	private:
		Float4 Data = { 0.0f, 0.0f, 0.0f, 1.0f };
		View4 View{ std::data(Data) };
	};
}



