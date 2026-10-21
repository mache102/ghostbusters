# Headless ghost collision probe (ghostbusters experiments)

add_executable(ghost_probe main.c)

set_target_properties(ghost_probe PROPERTIES
	C_STANDARD 17
)

target_link_libraries(ghost_probe PRIVATE box2d shared)
