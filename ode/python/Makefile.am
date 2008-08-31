
BUILT_SOURCES = $(builddir)/swig_wrap.cpp
SWIG_SOURCES = ode.i types.i mass.i world.i body.i joints.i \
                geom.i geoms.i spaces.i


pkgpython_PYTHON = $(builddir)/ode.py
pkgpyexec_LTLIBRARIES = _ode.la

_ode_la_SOURCES = $(BUILT_SOURCES) $(SWIG_SOURCES)
_ode_la_CPPFLAGS = $(SWIG_PYTHON_CPPFLAGS) -I$(top_srcdir)/include
_ode_la_LDFLAGS = -module
_ode_la_LIBADD = ../src/libode.la

$(BUILT_SOURCES) : $(SWIG_SOURCES)
	$(SWIG) $(SWIG_PYTHON_OPT) -modern -I$(top_srcdir)/include -o $@ $<

