# Sanitizers for GCC and Clang, chosen with EC_SANITIZE:
#   address: AddressSanitizer + UndefinedBehaviorSanitizer
#   thread:  ThreadSanitizer (it cannot be combined with the others)
# Directory-wide on purpose: every target defined after this include (library,
# apps, tests) gets the same flags. Dependencies from vcpkg stay unsanitized.
if(EC_SANITIZE)
    if(MSVC)
        message(FATAL_ERROR "EC_SANITIZE supports GCC and Clang only")
    endif()
    if(EC_SANITIZE STREQUAL "address")
        # -fno-sanitize-recover: undefined behaviour aborts the process, so a test
        # that triggers it fails instead of printing a warning and passing.
        set(ec_sanitize_flags -fsanitize=address,undefined -fno-sanitize-recover=all)
    elseif(EC_SANITIZE STREQUAL "thread")
        set(ec_sanitize_flags -fsanitize=thread)
    else()
        message(FATAL_ERROR "EC_SANITIZE must be address, thread or empty, not '${EC_SANITIZE}'")
    endif()
    add_compile_options(${ec_sanitize_flags} -fno-omit-frame-pointer)
    add_link_options(${ec_sanitize_flags})
endif()
