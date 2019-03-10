//
// Created by Todd Stellanova on 2019-03-09.
//

#include "rust_lib.h"


#include <stdio.h>
#include <uORB/uORB.h>
#include <px4_log.h>


extern "C" {
  uint32_t remote_add(uint32_t  lhs, uint32_t rhs);
  uint32_t happy_testo(uint32_t  a, uint32_t b);
}

RustLib::RustLib()
{

}

void RustLib::tricks() {
  uint32_t result = happy_testo(100, 23);
  PX4_WARN("tricks %d\n", result);
}
