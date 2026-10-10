# AddressSanitizer + UndefinedBehaviorSanitizer (GCC and Clang).
# Directory-wide on purpose: every target defined after this include (library,
# apps, tests) gets the same flags. Dependencies from vcpkg stay unsanitized.
if(EC_SANITIZE)
    if(MSVC)
        message(FATAL_ERROR "EC_SANITIZE supports GCC and Clang only")
    endif()
    # -fno-sanitize-recover: undefined behaviour aborts the process, so a test
    # that triggers it fails instead of printing a warning and passing.
    add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()
