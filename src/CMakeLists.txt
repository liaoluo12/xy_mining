
file(GLOB_RECURSE USER_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/app/*.c
        ${CMAKE_CURRENT_LIST_DIR}/app/*.cc
        ${CMAKE_CURRENT_LIST_DIR}/app/*.cpp
        ${CMAKE_CURRENT_LIST_DIR}/app/*.s
        ${CMAKE_CURRENT_LIST_DIR}/app/*.S)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)
target_include_directories(stm32cubemx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/app
)
