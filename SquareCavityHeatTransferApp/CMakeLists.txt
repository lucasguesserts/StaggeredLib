project(SquareCavityHeatTransferApp)

add_executable(${PROJECT_NAME} SquareCavityHeatTransferApp.cpp)

target_link_libraries(${PROJECT_NAME} Boost::filesystem)
target_link_libraries(${PROJECT_NAME} SquareCavityHeatTransfer)