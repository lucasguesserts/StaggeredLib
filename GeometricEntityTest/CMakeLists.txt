project(GeometricEntityTest)

file(GLOB_RECURSE SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_libraries(${PROJECT_NAME} GeometricEntity)
target_link_libraries(${PROJECT_NAME} Grid)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
