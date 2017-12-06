cmake_minimum_required(VERSION 2.8)
project (ShrinkWrap)

add_definitions(-D_UNICODE -DUNICODE)

find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )

# find_package(GMP REQUIRED)
# include_directories( ${GMP_INCLUDE_DIR} )

# find_package(MPFR REQUIRED)
# include_directories( ${MPFR_INCLUDE_DIR} )

add_definitions( -DBOOST_ALL_NO_LIB )
set( Boost_USE_STATIC_LIBS   ON )
find_package(Boost 1.64.0 COMPONENTS program_options system thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})

add_executable(ShrinkWrap
  ShrinkWrap.cpp
)
#target_link_libraries( ShrinkWrap ${GMP_LIBRARIES} )
#target_link_libraries( ShrinkWrap ${MPFR_LIBRARIES} )
target_link_libraries( ShrinkWrap ${Boost_LIBRARIES} )