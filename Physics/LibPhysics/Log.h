#pragma once

#ifndef LOG
#ifdef _DEBUG
#include <Windows.h>
#include <format>
#define LOG(x) OutputDebugStringA((x))
#else
#define LOG(x)
#endif
#endif
