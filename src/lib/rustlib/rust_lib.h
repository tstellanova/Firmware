//
// Created by Todd Stellanova on 2019-03-09.
//

#ifndef PX4_RUSTLIB_H
#define PX4_RUSTLIB_H

#pragma once

class RustLib {
public:
  RustLib();

  void tricks();

private:
  void* _boxed_rust_struct;

};


#endif //PX4_RUSTLIB_H
