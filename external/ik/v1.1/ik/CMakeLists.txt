project ("ik"
  VERSION 1.1.0
  LANGUAGES C ${CXX_LANGUAGE}
)

set (IK_PRECISION "double" CACHE STRING "Type to use for real numbers")

string (REPLACE " " "_" IK_PRECISION_CAPS_AND_NO_SPACES ${IK_PRECISION})
string (TOUPPER ${IK_PRECISION_CAPS_AND_NO_SPACES} IK_PRECISION_CAPS_AND_NO_SPACES)

configure_file ("templates/config.h.in"
                "include/public/ik/config.h")

###############################################################################
# All source file definitions
###############################################################################

set (IK_HEADERS
    "include/private/ik/backtrace.h"
    "include/private/ik/chain.h"
    "include/private/ik/memory.h"
    "include/public/ik/bstv.h"
    "include/public/ik/build_info.h"
    "include/public/ik/constraint.h"
    "include/public/ik/effector.h"
    "include/public/ik/ik.h"
    "include/public/ik/log.h"
    "include/public/ik/node.h"
    "include/public/ik/pstdint.h"
    "include/public/ik/quat.h"
    "include/public/ik/retcodes.h"
    "include/public/ik/solver.h"
    "include/public/ik/tests.h"
    "include/public/ik/transform.h"
    "include/public/ik/util.h"
    "include/public/ik/vec3.h"
    "include/public/ik/vector.h"
    "templates/config.h.in")
set (IK_SOURCES
    "src/bstv.c"
    "src/chain.c"
    "src/ik.c"
    "src/log_static.c"
    "src/memory.c"
    "src/quat_static.c"
    "src/retcodes.c"
    "src/solver_static.c"
    "src/transform_chains.c"
    "src/transform_tree.c"
    "src/util.c"
    "src/vec3_static.c"
    "src/vector.c"
    "src/solver/base/constraint_base.c"
    "src/solver/base/effector_base.c"
    "src/solver/base/node_base.c"
    "src/solver/base/solver_base.c"
    "src/solver/FABRIK/node_FABRIK.c"
    "src/solver/FABRIK/solver_FABRIK.c"
    "src/solver/MSS/solver_MSS.c"
    "src/solver/ONE_BONE/solver_ONE_BONE.c"
    "src/solver/TWO_BONE/solver_TWO_BONE.c"
    "${CMAKE_CURRENT_BINARY_DIR}/src/build_info.c"
    "src/platform/linux/backtrace_linux.c"
    "templates/build_info.c.in")
set (IK_VTABLES
    "include/vtables/build_info_static.v"
    "include/vtables/constraint_base.v"
    "include/vtables/effector_base.v"
    "include/vtables/log_static.v"
    "include/vtables/node_base.v"
    "include/vtables/node_FABRIK.v"
    "include/vtables/quat_static.v"
    "include/vtables/solver_base.v"
    "include/vtables/solver_FABRIK.v"
    "include/vtables/solver_MSS.v"
    "include/vtables/solver_ONE_BONE.v"
    "include/vtables/solver_static.v"
    "include/vtables/solver_TWO_BONE.v"
    "include/vtables/tests_static.v"
    "include/vtables/vec3_static.v")

# IK preprocessor script
set (Python_ADDITIONAL_VERSIONS 3)
find_package (PythonInterp REQUIRED)
set (IK_VTABLES_H "")
set (PYTHON_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/ik_gen_vtable.py)
macro (add_vtable_file VTABLE_FILE)
    get_filename_component (H_FILE_BASENAME ${VTABLE_FILE} NAME)
    string (REPLACE ".v" ".h" H_FILE_BASENAME ${H_FILE_BASENAME})
    set (H_FILE ${CMAKE_CURRENT_BINARY_DIR}/include/private/ik/${H_FILE_BASENAME})
    add_custom_command (
        OUTPUT ${H_FILE}
        DEPENDS ${VTABLE_FILE} ${PYTHON_SCRIPT}
        COMMAND ${PYTHON_EXECUTABLE} ${PYTHON_SCRIPT} ${VTABLE_FILE} ${H_FILE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating vtable ${H_FILE}" VERBATIM)
    set (IK_VTABLES_H ${IK_VTABLES_H} ${H_FILE})
endmacro ()
foreach (VTABLE_FILE ${IK_VTABLES})
    add_vtable_file (${VTABLE_FILE})
endforeach ()

###############################################################################
# Main library
###############################################################################

add_library (ik
  STATIC
  ${IK_HEADERS}
  ${IK_SOURCES}
  ${IK_VTABLES}
  ${IK_VTABLES_H}
)
set_target_properties(ik
  PROPERTIES
  C_STANDARD 11
)
target_include_directories (ik
  PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include/private
  ${CMAKE_CURRENT_SOURCE_DIR}/include/private
)
target_include_directories (ik
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/include/public
  ${CMAKE_CURRENT_SOURCE_DIR}/include/public
)
target_compile_definitions(ik
  PRIVATE
  IK_BUILDING
)
target_compile_options (ik
  PRIVATE
  -W -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-unused-parameter -Wno-conversion -Wno-implicit-fallthrough
  -pedantic -pedantic-errors -fno-strict-aliasing -ffast-math
)
