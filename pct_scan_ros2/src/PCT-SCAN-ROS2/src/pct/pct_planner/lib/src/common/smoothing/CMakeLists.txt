# Modified in 2026 for OSQP 1.0 relocatable package discovery.
find_package(osqp REQUIRED CONFIG)

# OSQP 1.0 exports ${prefix}/include/osqp, while this project includes
# headers as <osqp/osqp.h>.  Add the install prefix's include directory and
# derive it from osqp_DIR so the workspace remains relocatable.
get_filename_component(OSQP_INSTALL_PREFIX "${osqp_DIR}/../../.." ABSOLUTE)

add_library(common_smoothing SHARED
    affine_constraint.cc
    discrete_points_math.cc
    osqp_spline1d_solver.cc
    osqp_spline2d_solver.cc
    polynomialxd.cc
    spline1d_constraint.cc
    spline1d_kernel_helper.cc
    spline1d_kernel.cc
    spline1d_seg.cc
    spline1d.cc
    spline2d_constraint.cc
    spline2d_kernel.cc
    spline2d_seg.cc
    spline2d.cc
    solver/osqp/osqp_interface.cc
    utils/math.cc
)
target_include_directories(common_smoothing PUBLIC
  "${OSQP_INSTALL_PREFIX}/include"
)
target_link_libraries(common_smoothing PUBLIC
  osqp::osqpstatic
)
