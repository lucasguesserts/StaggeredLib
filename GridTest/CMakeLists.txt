project(GridTest)

file(GLOB SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_libraries(${PROJECT_NAME} Grid)
target_link_libraries(${PROJECT_NAME} Boost::filesystem)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
