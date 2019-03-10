//
// Created by Todd Stellanova on 2019-03-09.
//

#include "rust_lib.h"


#include <stdio.h>
#include <uORB/uORB.h>
#include <px4_log.h>
#include <uORB/topics/sensor_combined.h>

extern "C" {
  uint32_t remote_add(uint32_t  lhs, uint32_t rhs);
  uint32_t happy_testo(uint32_t  a, uint32_t b);
  uint64_t get_sub_metadata() ;

}

RustLib::RustLib()
{

}

void RustLib::tricks() {
  uint32_t result = happy_testo(100, 23);
  PX4_WARN("tricks: %u", result);

//  void* result = (void*)get_sub_metadata();
//  void* comparable = (void*)ORB_ID(sensor_combined);
//  int64_t  diff = (int64_t)result - (int64_t)comparable;
//  bool same = (diff == 0);
//  PX4_WARN("tricks same? %d diff: %lld %p %p\n", same, diff, result, comparable);
}
