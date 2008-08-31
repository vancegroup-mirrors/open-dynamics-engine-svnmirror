/* The main file */
%module ode

%{
/* Here go headers and declarations */
#include <ode/ode.h>

%}

%inline %{
#ifndef dDOUBLE
typedef float dReal;
#else
typedef double dReal;
#endif
%}

%init %{
    dInitODE();
%}

%inline %{
const char* getConfiguration()
{
    return dGetConfiguration();
}

bool checkConfiguration(const char*s)
{
    return dCheckConfiguration(s) != 0;
}
%}

%apply int * OUTPUT { int *outA, int *outB };
%apply dReal * OUTPUT { dReal *outA, dReal *outB };
%apply dReal * OUTPUT { dReal *outA, dReal *outB, dReal *outC, dReal *outD };


%include "ode/enums.h"


%include "types.i"

%include "mass.i"

%include "world.i"

%include "body.i"

%include "joints.i"

%include "geom.i"

%include "spaces.i"

%include "geoms.i"

