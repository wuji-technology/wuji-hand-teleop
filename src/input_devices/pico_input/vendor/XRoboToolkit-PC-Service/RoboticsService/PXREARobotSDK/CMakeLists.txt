cmake_minimum_required(VERSION 3.14)

project(PXREARobotSDK LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_host_system_information(RESULT ISA_NAME QUERY OS_PLATFORM)
message("OS_PLATFORM: ${ISA_NAME}")
if(WIN32)
set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/../SDK/win/64)
endif()

if(UNIX)
    if(ISA_NAME STREQUAL "aarch64")
        set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/../SDK/linux_aarch64/64)
    else()
        set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/../SDK/linux/64)
    endif()
endif()

add_compile_definitions(_SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING)




if(WIN32)

    add_definitions(-DPXREACLIENTSDK_LIBRARY)

    add_library(PXREARobotSDK SHARED
        ../PXREAService/PXREAService.grpc.pb.cc
        ../PXREAService/PXREAService.pb.cc
        ../PXREAService/PXREAService.grpc.pb.h
        ../PXREAService/PXREAService.pb.h
        PXREARobotSDK.cpp
        PXREARobotSDK.h

    )

    target_include_directories(PXREARobotSDK PUBLIC
        ../GrpcSDK/include
        ../PXREAService
    )
endif()

if(UNIX)
    if(ISA_NAME STREQUAL "aarch64")
        add_library(PXREARobotSDK SHARED
            ../PXREAService/linux_aarch64/PXREAService.grpc.pb.cc
            ../PXREAService/linux_aarch64/PXREAService.pb.cc
            ../PXREAService/linux_aarch64/PXREAService.grpc.pb.h
            ../PXREAService/linux_aarch64/PXREAService.pb.h
            PXREARobotSDK.cpp
            PXREARobotSDK.h

        )

        target_include_directories(PXREARobotSDK PUBLIC
            ../Redistributable/linux_aarch64/grpc/include
            ../PXREAService/linux_aarch64

        )

    else()
        add_library(PXREARobotSDK SHARED
            ../PXREAService/linux_x86/PXREAService.grpc.pb.cc
            ../PXREAService/linux_x86/PXREAService.pb.cc
            ../PXREAService/linux_x86/PXREAService.grpc.pb.h
            ../PXREAService/linux_x86/PXREAService.pb.h
            PXREARobotSDK.cpp
            PXREARobotSDK.h

        )

        target_include_directories(PXREARobotSDK PUBLIC
            ../Redistributable/linux/grpc/include
            ../PXREAService/linux_x86
        )

    endif()
endif()


if(WIN32)

    target_compile_definitions(PXREARobotSDK PUBLIC
        WINDOWS_x86
    )

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_link_directories(PXREARobotSDK PUBLIC ${PROJECT_SOURCE_DIR}/../GrpcSDK/lib/debug)

        target_link_libraries(PXREARobotSDK PUBLIC
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
            zlibstaticd
            absl_bad_any_cast_impl
            absl_bad_optional_access
            absl_bad_variant_access
            absl_base
            absl_city
            absl_civil_time
            absl_cord
            absl_debugging_internal
            absl_demangle_internal
            absl_examine_stack
            absl_exponential_biased
            absl_failure_signal_handler
            absl_flags
            absl_flags_commandlineflag
            absl_flags_commandlineflag_internal
            absl_flags_config
            absl_flags_internal
            absl_flags_marshalling
            absl_flags_parse
            absl_flags_private_handle_accessor
            absl_flags_program_name
            absl_flags_reflection
            absl_flags_usage
            absl_flags_usage_internal
            absl_graphcycles_internal
            absl_hash
            absl_hashtablez_sampler
            absl_int128
            absl_leak_check
            absl_leak_check_disable
            absl_log_severity
            absl_malloc_internal
            absl_periodic_sampler
            absl_random_distributions
            absl_random_internal_distribution_test_util
            absl_random_internal_platform
            absl_random_internal_pool_urbg
            absl_random_internal_randen
            absl_random_internal_randen_hwaes
            absl_random_internal_randen_hwaes_impl
            absl_random_internal_randen_slow
            absl_random_internal_seed_material
            absl_random_seed_gen_exception
            absl_random_seed_sequences
            absl_raw_hash_set
            absl_raw_logging_internal
            absl_scoped_set_env
            absl_spinlock_wait
            absl_stacktrace
            absl_status
            absl_statusor
            absl_str_format_internal
            absl_strerror
            absl_strings
            absl_strings_internal
            absl_symbolize
            absl_synchronization
            absl_throw_delegate
            absl_time
            absl_time_zone
            absl_wyhash
            address_sorting
            cares
            crypto
            gpr
            grpc
            grpc_plugin_support
            grpc_unsecure
            grpc++
            grpc++_alts
            grpc++_error_details
            grpc++_reflection
            grpc++_unsecure
            grpcpp_channelz
            libprotobufd
            libprotobuf-lited
            libprotocd
            re2
            ssl
            upb
            zlibstaticd
        )

    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_directories(PXREARobotSDK PUBLIC ${PROJECT_SOURCE_DIR}/../GrpcSDK/lib/release)

        target_link_libraries(PXREARobotSDK PUBLIC
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
            zlibstatic.lib
            absl_bad_any_cast_impl.lib
            absl_bad_optional_access.lib
            absl_bad_variant_access.lib
            absl_base.lib
            absl_city.lib
            absl_civil_time.lib
            absl_cord.lib
            absl_debugging_internal.lib
            absl_demangle_internal.lib
            absl_examine_stack.lib
            absl_exponential_biased.lib
            absl_failure_signal_handler.lib
            absl_flags.lib
            absl_flags_commandlineflag.lib
            absl_flags_commandlineflag_internal.lib
            absl_flags_config.lib
            absl_flags_internal.lib
            absl_flags_marshalling.lib
            absl_flags_parse.lib
            absl_flags_private_handle_accessor.lib
            absl_flags_program_name.lib
            absl_flags_reflection.lib
            absl_flags_usage.lib
            absl_flags_usage_internal.lib
            absl_graphcycles_internal.lib
            absl_hash.lib
            absl_hashtablez_sampler.lib
            absl_int128.lib
            absl_leak_check.lib
            absl_leak_check_disable.lib
            absl_log_severity.lib
            absl_malloc_internal.lib
            absl_periodic_sampler.lib
            absl_random_distributions.lib
            absl_random_internal_distribution_test_util.lib
            absl_random_internal_platform.lib
            absl_random_internal_pool_urbg.lib
            absl_random_internal_randen.lib
            absl_random_internal_randen_hwaes.lib
            absl_random_internal_randen_hwaes_impl.lib
            absl_random_internal_randen_slow.lib
            absl_random_internal_seed_material.lib
            absl_random_seed_gen_exception.lib
            absl_random_seed_sequences.lib
            absl_raw_hash_set.lib
            absl_raw_logging_internal.lib
            absl_scoped_set_env.lib
            absl_spinlock_wait.lib
            absl_stacktrace.lib
            absl_status.lib
            absl_statusor.lib
            absl_str_format_internal.lib
            absl_strerror.lib
            absl_strings.lib
            absl_strings_internal.lib
            absl_symbolize.lib
            absl_synchronization.lib
            absl_throw_delegate.lib
            absl_time.lib
            absl_time_zone.lib
            absl_wyhash.lib
            address_sorting.lib
            cares.lib
            crypto.lib
            gpr.lib
            grpc.lib
            grpc_plugin_support.lib
            grpc_unsecure.lib
            grpc++.lib
            grpc++_alts.lib
            grpc++_error_details.lib
            grpc++_reflection.lib
            grpc++_unsecure.lib
            grpcpp_channelz.lib
            libprotobuf.lib
            libprotobuf-lite.lib
            libprotoc.lib
            re2.lib
            ssl.lib
            upb.lib
            zlibstatic.lib
        )

    endif()


    target_compile_definitions(PXREARobotSDK PRIVATE PXREAROBOTSDK_LIBRARY)

    add_custom_command(TARGET PXREARobotSDK
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/PXREARobotSDK.lib" ${INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/PXREARobotSDK.dll" ${INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_SOURCE_DIR}/PXREARobotSDK.h" ${INSTALL_DIR}
    )
endif()


if(UNIX)
    if(ISA_NAME STREQUAL "aarch64")
        target_compile_definitions(PXREARobotSDK PUBLIC
            LINUX_aarch64
        )
        target_link_directories(PXREARobotSDK PUBLIC ${PROJECT_SOURCE_DIR}/../Redistributable/linux_aarch64/grpc/lib)
    else()
        target_compile_definitions(PXREARobotSDK PUBLIC
            LINUX_x86
        )
        target_link_directories(PXREARobotSDK PUBLIC ${PROJECT_SOURCE_DIR}/../Redistributable/linux/grpc/lib)
    endif()
    set(CMAKE_CXX_STANDARD 17)

    target_compile_definitions(PXREARobotSDK PUBLIC PXREAROBOTSDK_LIBRARY)

    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")


    target_link_libraries(PXREARobotSDK PUBLIC
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
        -Wl,--start-group
        libabsl_bad_any_cast_impl.a
        libabsl_bad_optional_access.a
        libabsl_bad_variant_access.a
        libabsl_base.a
        libabsl_city.a
        libabsl_civil_time.a
        libabsl_cord.a
        libabsl_cord_internal.a
        libabsl_cordz_functions.a
        libabsl_cordz_handle.a
        libabsl_cordz_info.a
        libabsl_cordz_sample_token.a
        libabsl_crc32c.a
        libabsl_crc_cord_state.a
        libabsl_crc_cpu_detect.a
        libabsl_crc_internal.a
        libabsl_debugging_internal.a
        libabsl_demangle_internal.a
        libabsl_die_if_null.a
        libabsl_examine_stack.a
        libabsl_exponential_biased.a
        libabsl_failure_signal_handler.a
        libabsl_flags_commandlineflag.a
        libabsl_flags_commandlineflag_internal.a
        libabsl_flags_config.a
        libabsl_flags_internal.a
        libabsl_flags_marshalling.a
        libabsl_flags_parse.a
        libabsl_flags_private_handle_accessor.a
        libabsl_flags_program_name.a
        libabsl_flags_reflection.a
        libabsl_flags_usage.a
        libabsl_flags_usage_internal.a
        libabsl_graphcycles_internal.a
        libabsl_hash.a
        libabsl_hashtablez_sampler.a
        libabsl_int128.a
        libabsl_kernel_timeout_internal.a
        libabsl_leak_check.a
        libabsl_log_entry.a
        libabsl_log_flags.a
        libabsl_log_globals.a
        libabsl_log_initialize.a
        libabsl_log_internal_check_op.a
        libabsl_log_internal_conditions.a
        libabsl_log_internal_fnmatch.a
        libabsl_log_internal_format.a
        libabsl_log_internal_globals.a
        libabsl_log_internal_log_sink_set.a
        libabsl_log_internal_message.a
        libabsl_log_internal_nullguard.a
        libabsl_log_internal_proto.a
        libabsl_log_severity.a
        libabsl_log_sink.a
        libabsl_low_level_hash.a
        libabsl_malloc_internal.a
        libabsl_periodic_sampler.a
        libabsl_random_distributions.a
        libabsl_random_internal_distribution_test_util.a
        libabsl_random_internal_platform.a
        libabsl_random_internal_pool_urbg.a
        libabsl_random_internal_randen.a
        libabsl_random_internal_randen_hwaes.a
        libabsl_random_internal_randen_hwaes_impl.a
        libabsl_random_internal_randen_slow.a
        libabsl_random_internal_seed_material.a
        libabsl_random_seed_gen_exception.a
        libabsl_random_seed_sequences.a
        libabsl_raw_hash_set.a
        libabsl_raw_logging_internal.a
        libabsl_scoped_set_env.a
        libabsl_spinlock_wait.a
        libabsl_stacktrace.a
        libabsl_status.a
        libabsl_statusor.a
        libabsl_strerror.a
        libabsl_str_format_internal.a
        libabsl_strings.a
        libabsl_strings_internal.a
        libabsl_string_view.a
        libabsl_symbolize.a
        libabsl_synchronization.a
        libabsl_throw_delegate.a
        libabsl_time.a
        libabsl_time_zone.a
        libabsl_vlog_config_internal.a
        libaddress_sorting.a
        libcares.a
        libcrypto.a
        libgpr.a
        libgrpc++.a
        libgrpc.a
        libgrpc++_alts.a
        libgrpc_authorization_provider.a
        libgrpc++_error_details.a
        libgrpc_plugin_support.a
        libgrpcpp_channelz.a
        libgrpc++_reflection.a
        libgrpc++_unsecure.a
        libgrpc_unsecure.a
        libprotobuf.a
        libprotobuf-lite.a
        libprotoc.a
        libre2.a
        libssl.a
        libupb.a
        libutf8_range.a
        libutf8_validity.a
        libz.a
        -Wl,--end-group
        rt
        pthread
        dl
    )

    add_custom_command(TARGET PXREARobotSDK
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/libPXREARobotSDK.so" ${INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_SOURCE_DIR}/PXREARobotSDK.h" ${PROJECT_SOURCE_DIR}/../SDK/include/
    )
endif()
