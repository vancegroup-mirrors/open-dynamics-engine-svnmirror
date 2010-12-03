AM_CPPFLAGS = -I$(top_srcdir)/include

noinst_LTLIBRARIES = libjoints.la

libjoints_la_SOURCES =  joints.h \
                        joint.h joint.cpp \
                        joint_internal.h \
                        ball.h ball.cpp \
                        hinge.h hinge.cpp \
                        slider.h slider.cpp \
                        contact.h contact.cpp \
                        universal.h universal.cpp \
                        hinge2.h hinge2.cpp \
                        fixed.h fixed.cpp \
                        null.h null.cpp \
                        amotor.h amotor.cpp \
                        lmotor.h lmotor.cpp \
                        plane2d.h plane2d.cpp \
                        pu.h pu.cpp \
                        pr.h pr.cpp \
                        piston.h piston.cpp

#################################
#   Export more files for external solver use
#################################
libode_la_includedir = $(includedir)/src/joints
libode_la_include_HEADERS = 	\
				universal.h \
				slider.h \
				pu.h \
				pr.h \
				plane2d.h \
				piston.h \
				null.h \
				lmotor.h \
				joints.h \
				joint_internal.h \
				joint.h \
				hinge.h \
				hinge2.h \
				fixed.h \
				contact.h \
				ball.h \
				amotor.h

