AM_CPPFLAGS = -I$(abs_top_srcdir)/include -I$(abs_top_builddir)/include \
        -DDRAWSTUFF_TEXTURE_PATH="\"$(abs_top_srcdir)/drawstuff/textures\""
LIBS = @GL_LIBS@ @XLIBRARIES@ @LIBSTDCXX@
GENERALLIBS = $(top_builddir)/drawstuff/src/libdrawstuff.la
SINGLELIBS= $(GENERALLIBS) $(top_builddir)/ode/src/libode_single.la
DOUBLELIBS= $(GENERALLIBS) $(top_builddir)/ode/src/libode_double.la

noinst_HEADERS = basket_geom.h texturepath.h world_geom3.h

noinst_PROGRAMS=demo_collision_single \
		demo_slider_single \
		demo_feedback_single \
		demo_crash_single \
		demo_space_single \
		demo_I_single \
		demo_friction_single \
		demo_space_stress_single \
		demo_boxstack_single \
		demo_hinge_single \
		demo_step_single \
		demo_buggy_single \
		demo_joints_single \
		demo_motor_single \
		demo_piston_single \
		demo_chain1_single \
		demo_chain2_single \
                demo_cylvssphere_single \
		demo_ode_single \
		demo_plane2d_single \
		demo_heightfield_single \
		demo_convex_cd_single \
		demo_jointPR_single \
		demo_collision_double \
		demo_slider_double \
		demo_feedback_double \
		demo_crash_double \
		demo_space_double \
		demo_I_double \
		demo_friction_double \
		demo_space_stress_double \
		demo_boxstack_double \
		demo_hinge_double \
		demo_step_double \
		demo_buggy_double \
		demo_joints_double \
		demo_motor_double \
		demo_piston_double \
		demo_chain1_double \
		demo_chain2_double \
		demo_cylvssphere_double \
		demo_ode_double \
		demo_plane2d_double \
		demo_heightfield_double \
		demo_convex_cd_double \
		demo_jointPR_double
if TRIMESH
noinst_PROGRAMS+= demo_trimesh_single \
		  demo_moving_trimesh_single \
		  demo_basket_single \
		  demo_cyl_single \
		  demo_trimesh_double \
		  demo_moving_trimesh_double \
		  demo_basket_double \
		  demo_cyl_double
endif

# SINGLE PRECISION

demo_collision_single_SOURCES= demo_collision.cpp
demo_slider_single_SOURCES= demo_slider.cpp 
demo_feedback_single_SOURCES= demo_feedback.cpp 
demo_crash_single_SOURCES= demo_crash.cpp
demo_space_single_SOURCES= demo_space.cpp
demo_I_single_SOURCES= demo_I.cpp
demo_friction_single_SOURCES= demo_friction.cpp
demo_space_stress_single_SOURCES= demo_space_stress.cpp
demo_boxstack_single_SOURCES= demo_boxstack.cpp
demo_hinge_single_SOURCES= demo_hinge.cpp
demo_step_single_SOURCES= demo_step.cpp
demo_buggy_single_SOURCES= demo_buggy.cpp
demo_cyl_single_SOURCES= demo_cyl.cpp world_geom3.h
demo_cylvssphere_single_SOURCES= demo_cylvssphere.cpp
demo_joints_single_SOURCES= demo_joints.cpp
demo_jointPR_single_SOURCES= demo_jointPR.cpp
demo_motor_single_SOURCES= demo_motor.cpp
demo_piston_single_SOURCES = demo_piston.cpp
demo_chain1_single_SOURCES= demo_chain1.c
demo_chain2_single_SOURCES= demo_chain2.cpp
demo_ode_single_SOURCES= demo_ode.cpp
demo_plane2d_single_SOURCES= demo_plane2d.cpp
demo_heightfield_single_SOURCES= demo_heightfield.cpp
demo_convex_cd_single_SOURCES= demo_convex_cd.cpp
# DOUBLE PRECISION
demo_collision_double_SOURCES=demo_collision.cpp
demo_slider_double_SOURCES= demo_slider.cpp 
demo_feedback_double_SOURCES= demo_feedback.cpp 
demo_crash_double_SOURCES= demo_crash.cpp
demo_space_double_SOURCES= demo_space.cpp
demo_I_double_SOURCES= demo_I.cpp
demo_friction_double_SOURCES= demo_friction.cpp
demo_space_stress_double_SOURCES= demo_space_stress.cpp
demo_boxstack_double_SOURCES= demo_boxstack.cpp
demo_hinge_double_SOURCES= demo_hinge.cpp
demo_step_double_SOURCES= demo_step.cpp
demo_buggy_double_SOURCES= demo_buggy.cpp
demo_cyl_double_SOURCES= demo_cyl.cpp world_geom3.h
demo_cylvssphere_double_SOURCES=demo_cylvssphere.cpp
demo_joints_double_SOURCES= demo_joints.cpp
demo_jointPR_double_SOURCES= demo_jointPR.cpp
demo_motor_double_SOURCES= demo_motor.cpp
demo_piston_double_SOURCES = demo_piston.cpp
demo_chain1_double_SOURCES= demo_chain1.c
demo_chain2_double_SOURCES= demo_chain2.cpp
demo_ode_double_SOURCES= demo_ode.cpp
demo_plane2d_double_SOURCES= demo_plane2d.cpp
demo_heightfield_double_SOURCES= demo_heightfield.cpp
demo_convex_cd_double_SOURCES= demo_convex_cd.cpp

demo_collision_single_DEPENDENCIES=$(SINGLELIBS)
demo_slider_single_DEPENDENCIES=$(SINGLELIBS)
demo_feedback_single_DEPENDENCIES=$(SINGLELIBS)
demo_crash_single_DEPENDENCIES=$(SINGLELIBS)
demo_space_single_DEPENDENCIES=$(SINGLELIBS)
demo_I_single_DEPENDENCIES=$(SINGLELIBS)
demo_friction_single_DEPENDENCIES=$(SINGLELIBS)
demo_space_stress_single_DEPENDENCIES=$(SINGLELIBS)
demo_boxstack_single_DEPENDENCIES=$(SINGLELIBS)
demo_hinge_single_DEPENDENCIES=$(SINGLELIBS)
demo_step_single_DEPENDENCIES=$(SINGLELIBS)
demo_buggy_single_DEPENDENCIES=$(SINGLELIBS)
demo_cyl_single_DEPENDENCIES=$(SINGLELIBS)
demo_cylvssphere_single_DEPENDENCIES=$(SINGLELIBS)
demo_joints_single_DEPENDENCIES=$(SINGLELIBS)
demo_jointPR_single_DEPENDENCIES=$(SINGLELIBS)
demo_motor_single_DEPENDENCIES=$(SINGLELIBS)
demo_piston_single_DEPENDENCIES=$(SINGLELIBS)
demo_chain1_single_DEPENDENCIES=$(SINGLELIBS)
demo_chain2_single_DEPENDENCIES=$(SINGLELIBS)
demo_ode_single_DEPENDENCIES=$(SINGLELIBS)
demo_plane2d_single_DEPENDENCIES=$(SINGLELIBS)
demo_heightfield_single_DEPENDENCIES=$(SINGLELIBS)
demo_convex_cd_single_DEPENDENCIES=$(SINGLELIBS)

demo_collision_single_LDADD=$(SINGLELIBS)
demo_slider_single_LDADD=$(SINGLELIBS)
demo_feedback_single_LDADD=$(SINGLELIBS)
demo_crash_single_LDADD=$(SINGLELIBS)
demo_space_single_LDADD=$(SINGLELIBS)
demo_I_single_LDADD=$(SINGLELIBS)
demo_friction_single_LDADD=$(SINGLELIBS)
demo_space_stress_single_LDADD=$(SINGLELIBS)
demo_boxstack_single_LDADD=$(SINGLELIBS)
demo_hinge_single_LDADD=$(SINGLELIBS)
demo_step_single_LDADD=$(SINGLELIBS)
demo_buggy_single_LDADD=$(SINGLELIBS)
demo_cyl_single_LDADD=$(SINGLELIBS)
demo_cylvssphere_single_LDADD=$(SINGLELIBS)
demo_joints_single_LDADD=$(SINGLELIBS)
demo_jointPR_single_LDADD=$(SINGLELIBS)
demo_motor_single_LDADD=$(SINGLELIBS)
demo_piston_single_LDADD=$(SINGLELIBS)
demo_chain1_single_LDADD=$(SINGLELIBS)
demo_chain2_single_LDADD=$(SINGLELIBS)
demo_ode_single_LDADD=$(SINGLELIBS)
demo_plane2d_single_LDADD=$(SINGLELIBS)
demo_heightfield_single_LDADD=$(SINGLELIBS)
demo_convex_cd_single_LDADD=$(SINGLELIBS)

demo_collision_double_DEPENDENCIES=$(DOUBLELIBS)
demo_slider_double_DEPENDENCIES=$(DOUBLELIBS)
demo_feedback_double_DEPENDENCIES=$(DOUBLELIBS)
demo_crash_double_DEPENDENCIES=$(DOUBLELIBS)
demo_space_double_DEPENDENCIES=$(DOUBLELIBS)
demo_I_double_DEPENDENCIES=$(DOUBLELIBS)
demo_friction_double_DEPENDENCIES=$(DOUBLELIBS)
demo_space_stress_double_DEPENDENCIES=$(DOUBLELIBS)
demo_boxstack_double_DEPENDENCIES=$(DOUBLELIBS)
demo_hinge_double_DEPENDENCIES=$(DOUBLELIBS)
demo_step_double_DEPENDENCIES=$(DOUBLELIBS)
demo_buggy_double_DEPENDENCIES=$(DOUBLELIBS)
demo_cyl_double_DEPENDENCIES=$(DOUBLELIBS)
demo_cylvssphere_double_DEPENDENCIES=$(DOUBLELIBS)
demo_joints_double_DEPENDENCIES=$(DOUBLELIBS)
demo_jointPR_double_DEPENDENCIES=$(DOUBLELIBS)
demo_motor_double_DEPENDENCIES=$(DOUBLELIBS)
demo_piston_double_DEPENDENCIES=$(DOUBLELIBS)
demo_chain1_double_DEPENDENCIES=$(DOUBLELIBS)
demo_chain2_double_DEPENDENCIES=$(DOUBLELIBS)
demo_ode_double_DEPENDENCIES=$(DOUBLELIBS)
demo_plane2d_double_DEPENDENCIES=$(DOUBLELIBS)
demo_heightfield_double_DEPENDENCIES=$(DOUBLELIBS)
demo_convex_cd_double_DEPENDENCIES=$(DOUBLELIBS)

demo_collision_double_LDADD=$(DOUBLELIBS)
demo_slider_double_LDADD=$(DOUBLELIBS)
demo_feedback_double_LDADD=$(DOUBLELIBS)
demo_crash_double_LDADD=$(DOUBLELIBS)
demo_space_double_LDADD=$(DOUBLELIBS)
demo_I_double_LDADD=$(DOUBLELIBS)
demo_friction_double_LDADD=$(DOUBLELIBS)
demo_space_stress_double_LDADD=$(DOUBLELIBS)
demo_boxstack_double_LDADD=$(DOUBLELIBS)
demo_hinge_double_LDADD=$(DOUBLELIBS)
demo_step_double_LDADD=$(DOUBLELIBS)
demo_buggy_double_LDADD=$(DOUBLELIBS)
demo_cyl_double_LDADD=$(DOUBLELIBS)
demo_cylvssphere_double_LDADD=$(DOUBLELIBS)
demo_joints_double_LDADD=$(DOUBLELIBS)
demo_jointPR_double_LDADD=$(DOUBLELIBS)
demo_motor_double_LDADD=$(DOUBLELIBS)
demo_piston_double_LDADD=$(DOUBLELIBS)
demo_chain1_double_LDADD=$(DOUBLELIBS)
demo_chain2_double_LDADD=$(DOUBLELIBS)
demo_ode_double_LDADD=$(DOUBLELIBS)
demo_plane2d_double_LDADD=$(DOUBLELIBS)
demo_heightfield_double_LDADD=$(DOUBLELIBS)
demo_convex_cd_double_LDADD=$(DOUBLELIBS)

demo_collision_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_slider_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_feedback_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_crash_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_space_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_I_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_friction_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_space_stress_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_boxstack_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_hinge_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_step_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_buggy_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_cyl_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_cylvssphere_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_joints_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_jointPR_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_motor_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_piston_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_chain1_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_chain2_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_ode_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_plane2d_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_heightfield_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_convex_cd_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)

if TRIMESH
demo_trimesh_single_SOURCES= 			demo_trimesh.cpp
demo_moving_trimesh_single_SOURCES= 		demo_moving_trimesh.cpp
demo_basket_single_SOURCES= 			demo_basket.cpp
demo_trimesh_single_DEPENDENCIES=$(SINGLELIBS)
demo_moving_trimesh_single_DEPENDENCIES=$(SINGLELIBS)
demo_basket_single_DEPENDENCIES=$(SINGLELIBS)
demo_trimesh_single_LDADD=$(SINGLELIBS)
demo_moving_trimesh_single_LDADD=$(SINGLELIBS)
demo_basket_single_LDADD=$(SINGLELIBS)
demo_trimesh_double_SOURCES= 			demo_trimesh.cpp
demo_moving_trimesh_double_SOURCES= 		demo_moving_trimesh.cpp
demo_basket_double_SOURCES= 			demo_basket.cpp
demo_trimesh_double_DEPENDENCIES=$(DOUBLELIBS)
demo_moving_trimesh_double_DEPENDENCIES=$(DOUBLELIBS)
demo_basket_double_DEPENDENCIES=$(DOUBLELIBS)
demo_trimesh_double_LDADD=$(DOUBLELIBS)
demo_moving_trimesh_double_LDADD=$(DOUBLELIBS)
demo_basket_double_LDADD=$(DOUBLELIBS)
demo_trimesh_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_moving_trimesh_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
demo_basket_double_CPPFLAGS=-DdDOUBLE $(AM_CPPFLAGS)
endif


if WIN32
resources.o: $(top_srcdir)/drawstuff/src/resources.rc $(top_srcdir)/drawstuff/src/resource.h
	@WINDRES@ $(top_srcdir)/drawstuff/src/resources.rc -o resources.o
GENERALLIBS += resources.o
endif


