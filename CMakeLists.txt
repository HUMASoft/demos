project(demos)

cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(main SRC_LIST)
aux_source_directory(tools SRC_LIST)


add_subdirectory(lib)
#revisar la forma de incluir kinematics
INCLUDE_DIRECTORIES(kinematics)
add_library(kinematics kinematics/Kinematics.cpp)
set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} kinematics)




#find_package(libplot)

INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})

foreach( testsourcefile ${SRC_LIST} )
    # I used a simple string replace, to cut off .cpp.
    string( REGEX REPLACE "[A-z]+[/]" "" testname ${testsourcefile} )
    string( REPLACE ".cpp" "" testname ${testname} )

    add_executable( ${testname} ${testsourcefile} )
    message ("Including executable: " ${testname})
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${SUBDIR_LINK_NAMES} )
endforeach( testsourcefile ${APP_SOURCES} )


#link with libs
#target_link_libraries( ${PROJECT_NAME} fcontrol ${SUBDIR_LINK_NAMES})


##functions
function(echo_all_cmake_variable_values)
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction()


echo_all_cmake_variable_values()
