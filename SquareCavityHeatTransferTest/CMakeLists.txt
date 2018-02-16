project(SquareCavityHeatTransferTest)

file(GLOB SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_libraries(${PROJECT_NAME} SquareCavityHeatTransfer)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
