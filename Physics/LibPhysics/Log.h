#pragma once

#include <chrono>
#include <source_location>

#ifndef LOG
#ifdef _DEBUG
#include <Windows.h>
#include <format>
#define LOG(x) OutputDebugStringA((x))
#else
#define LOG(x)
#endif
#endif

#ifndef PERFORMANCE_COUNTER
#ifdef _DEBUG
#define PERFORMANCE_COUNTER(x) PerformanceCounter __PC(x)
#define PERFORMANCE_COUNTER_FUNC() PERFORMANCE_COUNTER(std::source_location::current().function_name())
#else
#define PERFORMANCE_COUNTER(x)
#define PERFORMANCE_COUNTER_FUNC()
#endif
#endif

class PerformanceCounter
{
public:
	PerformanceCounter(std::string_view Lbl = "") : Start(std::chrono::system_clock::now()), Label(Lbl) {}
	~PerformanceCounter() {
		const auto End = std::chrono::system_clock::now();
		const auto MilliSec = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(End - Start).count());
		LOG(std::data(std::format("{} : {} msec\n", Label, MilliSec)));
	}
private:
	std::chrono::system_clock::time_point Start;
	std::string Label;
};