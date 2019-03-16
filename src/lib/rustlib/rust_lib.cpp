//
// Created by Todd Stellanova on 2019-03-09.
//

#include "rust_lib.h"


#include <stdio.h>
#include <uORB/uORB.h>
#include <px4_log.h>
#include <uORB/topics/sensor_combined.h>

extern "C" {
  uint64_t get_sub_metadata() ;
  uint32_t ephemeral_happy();
  void* new_happy_instance() ;
  uint32_t check_inst_subs(void* happy);
}

RustLib::RustLib()
{
  _boxed_rust_struct = new_happy_instance();
}



void RustLib::tricks() {
  uint32_t result = check_inst_subs(this->_boxed_rust_struct);
  PX4_WARN("tricks: %u", result);

//  void* result = (void*)get_sub_metadata();
//  void* comparable = (void*)ORB_ID(sensor_combined);
//  int64_t  diff = (int64_t)result - (int64_t)comparable;
//  bool same = (diff == 0);
//  PX4_WARN("tricks same? %d diff: %lld %p %p\n", same, diff, result, comparable);
}
