project(PetersFacetCenterHeatTransfer)

file (GLOB_RECURSE source_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} STATIC ${source_files})

target_link_libraries(${PROJECT_NAME} GeometricEntity)
target_link_libraries(${PROJECT_NAME} Stencil)
target_link_libraries(${PROJECT_NAME} Grid)
target_link_libraries(${PROJECT_NAME} LinearSystem)