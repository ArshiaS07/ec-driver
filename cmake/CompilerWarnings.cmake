# Warnings for our own targets only (never for dependencies).
function(ec_set_warnings target)
    if(MSVC)
        # /utf-8: read sources as UTF-8, like GCC and Clang do
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
            -Wnon-virtual-dtor -Wold-style-cast -Wcast-align)
    endif()
    if(EC_WARNINGS_AS_ERRORS)
        set_target_properties(${target} PROPERTIES COMPILE_WARNING_AS_ERROR ON)
    endif()
endfunction()
