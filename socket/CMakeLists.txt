aux_source_directory(./ TCP_LIB_DIR)
# add_library(tcp_interface SHARED ${cpp_files})
add_library(tcp_interface STATIC ${TCP_LIB_DIR})
target_link_libraries(tcp_interface pthread)
