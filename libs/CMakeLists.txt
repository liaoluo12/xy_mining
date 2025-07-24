
# Pack all third-party libraries into a single target

set(LIBRM_PLATFORM STM32)
add_subdirectory(librm)

add_library(${PROJECT_NAME}_libs INTERFACE)
target_link_libraries(${PROJECT_NAME}_libs INTERFACE
        rm)