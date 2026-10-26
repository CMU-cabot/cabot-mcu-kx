#pragma once
#include <cstdio>
// Assertions remain enabled by the build script, including release builds.
#define RUN_TEST(function) do { function(); std::puts("  PASS: " #function); } while (false)
