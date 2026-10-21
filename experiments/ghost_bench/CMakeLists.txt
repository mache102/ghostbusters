# Topological ghost cost benchmarks (ghostbusters experiments)

add_executable(ghost_bench main.c)

set_target_properties(ghost_bench PROPERTIES
	C_STANDARD 17
)

# Recorded in the output header
string(TOUPPER "${CMAKE_BUILD_TYPE}" GHOST_BENCH_CONFIG)
target_compile_definitions(ghost_bench PRIVATE
	GHOST_BENCH_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
	GHOST_BENCH_C_FLAGS="${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${GHOST_BENCH_CONFIG}}"
)

target_link_libraries(ghost_bench PRIVATE box2d shared)
