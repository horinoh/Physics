#pragma once

#include <fstream>
#include <filesystem>

#ifdef _DEBUG
#define LOG(x) OutputDebugStringA((x))
#else
#define LOG(x)
#endif

const auto ASSET_PATH = std::filesystem::path("..") / "Asset";
const auto GLTF_PATH = std::filesystem::path("..") / std::filesystem::path("..") / "glTF-Sample-Models" / "2.0";

#define TO_RADIAN(x) ((x) * std::numbers::pi_v<float> / 180.0f)
#define TO_DEGREEE(x) ((x) * 180.0f / std::numbers::pi_v<float>)

class Win 
{
public:
	enum class CONTROL_FLAG : uint8_t {
		Pause,
		Step,
		Count,
	};
	bool IsPause() const { return ControlFlag.test(static_cast<size_t>(CONTROL_FLAG::Pause)); }
	void Pause() { ControlFlag.set(static_cast<size_t>(CONTROL_FLAG::Pause)); }
	void UnPause() { ControlFlag.reset(static_cast<size_t>(CONTROL_FLAG::Pause)); }
	void TogglePause() { IsPause() ? UnPause() : Pause(); }

	bool IsStep() const { return ControlFlag.test(static_cast<size_t>(CONTROL_FLAG::Step)); }
	void Step() { ControlFlag.set(static_cast<size_t>(CONTROL_FLAG::Step)); }
	void UnStep() { ControlFlag.reset(static_cast<size_t>(CONTROL_FLAG::Step)); }
	bool ProcessStep() {
		if (IsStep()) {
			UnStep();
			return true;
		}
		return false;
	}
	bool IsUpdate() { return !IsPause() || ProcessStep(); }

	virtual void OnKeyDown(HWND hWnd, [[maybe_unused]] HINSTANCE hInstance, const WPARAM Param) {
		switch (Param) {
		case VK_ESCAPE:
			SendMessage(hWnd, WM_DESTROY, 0, 0);
			break;
		case VK_PAUSE:
			TogglePause();
			break;
		case VK_RETURN:
		case VK_SPACE:
			Step();
			break;
		default:
			break;
		}
	}

protected:
	std::bitset<static_cast<size_t>(CONTROL_FLAG::Count)> ControlFlag;
};

static constexpr size_t RoundUpMask(const size_t Size, const size_t Mask) { return (Size + Mask) & ~Mask; }
static constexpr size_t RoundUp(const size_t Size, const size_t Align) { return RoundUpMask(Size, Align - 1); }
static constexpr size_t RoundUp256(const size_t Size) { return RoundUpMask(Size, 0xff); }

static bool IsDDS(const std::filesystem::path& Path) {
	std::ifstream In(data(Path.string()), std::ios::in | std::ios::binary);
	if (!In.fail()) {
		std::array<uint32_t, 2> Header = { 0, 0 };
		In.read(reinterpret_cast<char*>(data(Header)), sizeof(Header));
		In.close();
		return 0x20534444 == Header[0] && 124 == Header[1];
	}
	return false;
}
static bool IsKTX(const std::filesystem::path& Path) {
	std::ifstream In(data(Path.string()), std::ios::in | std::ios::binary);
	if (!In.fail()) {
		std::array<uint32_t, 4> Header = { 0, 0, 0, 0 };
		In.read(reinterpret_cast<char*>(data(Header)), sizeof(Header));
		In.close();
		return 0x58544bab == Header[0] && 0xbb313120 == Header[1] && 0x0a1a0a0d == Header[2] && 0x04030201 == Header[3];
	}
	return false;
}

template<typename T> static constexpr size_t TotalSizeOf(const std::vector<T>& rhs) { return sizeof(T) * size(rhs); }
template<typename T, size_t U> static constexpr size_t TotalSizeOf(const std::array<T, U>& rhs) { return sizeof(rhs); }

template<typename T> const T GetMin(const T& lhs, const T& rhs);
template<typename T> const T GetMax(const T& lhs, const T& rhs);
template<typename T> void AdjustScale(std::vector<T>& Vertices, const float Scale);
