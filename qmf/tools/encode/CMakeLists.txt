# used indirectly by module service (from libservice repo)

define_module(BINARY qmf-encode
  DEPENDS qmf service
  )

set(qmf-encode_SOURCES
  qmf-encode.cpp
  )

add_executable(qmf-encode ${qmf-encode_SOURCES} )
target_link_libraries(qmf-encode ${MODULE_LIBRARIES})
buildsys_binary(qmf-encode)
