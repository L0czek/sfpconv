set(sfpconv_hal_file ${CMAKE_CURRENT_SOURCE_DIR}/inc/hal.hpp)

function(replace_in_file filepath pattern replacement)
    file(READ "${filepath}" file_content)
    string(REPLACE "${pattern}" "${replacement}" new_content "${file_content}")
    file(WRITE "${filepath}" "${new_content}")
endfunction()

replace_in_file(${sfpconv_hal_file} "stm32f1x" "${chip_family}")

add_library(sfpconv
    src/sfpconv.cc
    src/sfp_conv_io.cc
)

set(SFPCONV_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)

target_include_directories(sfpconv PRIVATE ${SFPCONV_INC_DIR} ${TCA9535_INC_DIR} ${Si5351_INC_DIR})

set(SFPCONV_INC_DIR ${SFPCONV_INC_DIR} PARENT_SCOPE)
