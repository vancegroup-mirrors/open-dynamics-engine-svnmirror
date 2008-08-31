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


%include "types.i"

%include "mass.i"

%include "world.i"

%include "body.i"

%include "joints.i"

