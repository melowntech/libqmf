# bump version here
set(qmf_VERSION 0.0)

define_module(LIBRARY qmf=${qmf_VERSION}
  DEPENDS
  BuildSystem>=1.13
  miniball geo geometry Boost_THREAD Boost_SYSTEM)

set(qmf_SOURCES
  qmf.hpp qmf.cpp
  )

add_library(qmf STATIC ${qmf_SOURCES})
buildsys_library(qmf)

target_link_libraries(qmf ${MODULE_LIBRARIES})
target_compile_definitions(qmf PRIVATE ${MODULE_DEFINITIONS})

if(MODULE_service_FOUND)
  add_subdirectory(tools EXCLUDE_FORM_ALL)
endif()
