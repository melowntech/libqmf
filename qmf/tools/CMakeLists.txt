# used indirectly by module service (from libservice repo)

define_module(BINARY qmf-convert
  DEPENDS qmf service
  )

set(qmf-convert_SOURCES
  qmf-convert.cpp
  )

add_executable(qmf-convert ${qmf-convert_SOURCES} )
target_link_libraries(qmf-convert ${MODULE_LIBRARIES})
target_compile_definitions(qmf-convert PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(qmf-convert)
