project(FacetCenterHeatTransferTest)

file(GLOB_RECURSE SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} Boost::filesystem)
target_link_libraries(${PROJECT_NAME} FacetCenterHeatTransfer)
