project(FacetCenterApp)

file(GLOB_RECURSE SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_libraries(${PROJECT_NAME} Utils)
target_link_libraries(${PROJECT_NAME} FacetCenterHeatTransfer)
target_link_libraries(${PROJECT_NAME} SquareCavityHeatTransfer)
target_link_libraries(${PROJECT_NAME} CgnsInterface)