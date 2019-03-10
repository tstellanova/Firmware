############################################################################
#
#   Copyright (c) 2019 Todd Stellanova. All rights reserved.


px4_add_library(rustlib
		rust_lib.cpp
)

target_link_libraries(rustlib PUBLIC
#		debug "${CMAKE_CURRENT_SOURCE_DIR}/common-rust/target/debug/librust_example.a"
		optimized "${CMAKE_CURRENT_SOURCE_DIR}/common-rust/target/release/librust_example.a"
#		optimized "${CMAKE_CURRENT_SOURCE_DIR}/common-rust/target/armv7-unknown-linux-gnueabihf/release/librust_example.a"
#		optimized "${CMAKE_CURRENT_SOURCE_DIR}/common-rust/target/thumbv7em-none-eabihf/release/librust_example.a"
		)



