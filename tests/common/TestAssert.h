#pragma once

#include <iostream>

#define TEST_ASSERT(CONDITION, MESSAGE)                                           \
  do {                                                                            \
    if (!(CONDITION)) {                                                           \
      std::cerr << "[TEST ASSERTION FAILED] " << (MESSAGE) << std::endl;         \
      return 1;                                                                   \
    }                                                                             \
  } while (0)

