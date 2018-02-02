project(UtilsTest)

file(GLOB SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})
