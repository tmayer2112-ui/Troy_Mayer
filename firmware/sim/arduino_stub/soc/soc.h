#pragma once
#include <cstdint>
inline uint32_t stub_reg_read(uintptr_t) { return 0; }
#define REG_READ(r) stub_reg_read(r)
