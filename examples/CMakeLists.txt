add_executable(main "main.cpp")
target_link_libraries(main PRIVATE lib)
target_link_libraries(main PRIVATE embree)

add_executable(embree_test "embree_test.cpp")
target_link_libraries(embree_test PRIVATE embree)