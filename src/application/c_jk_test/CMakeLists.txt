
add_executable(ecp_jk_test
	ecp_t_jk_test.cc
)

add_executable(mp_jk_test
	mp_t_jk_test.cc
)

target_link_libraries(mp_jk_test mp)

target_link_libraries(ecp_jk_test
    ecp_robot
	)

install(TARGETS mp_jk_test DESTINATION bin)
install(TARGETS	ecp_jk_test DESTINATION bin)

