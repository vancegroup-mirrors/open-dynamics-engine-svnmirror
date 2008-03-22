#ifndef ODE_ERROR_PRIVATE
#define ODE_ERROR_PRIVATE

void dErrorInit(void);

#ifdef dNOERRORCHECK
#define dCheck(x, e)
#define dCheckRet(x, e, r)
#elif defined(dFATALERRORS)
#include <assert.h>
#define dCheck(x, e)            assert(x)
#define dCheckRet(x, e, r)      assert(x)
#elif defined(dEXCEPTIONS)
#define dCheck(x, e)                                   \
        if (!(x)) {                                    \
                throw (e);                             \
        } else {                                       \
                dSetLastError(dErrNoError);            \
        }
#define dCheckRet(x, e, r) dCheck(x, e)
#else
#define dCheck(x, e)                            \
        if (!(x)) {                             \
                dSetLastError(e);               \
                return;                         \
        } else {                                \
                dSetLastError(dErrNoError);     \
        }
#define dCheckRet(x, e, r) \
        if (!(x)) {     \
                dSetLastError(e);               \
                return r;                       \
        } else {                                \
                dSetLastError(dErrNoError);     \
        }
#endif


#define dCheckArg(x) dCheck(x, dErrInvalidArg)
#define dCheckArgRet(x, r) dCheckRet(x, dErrInvalidArg, r)

#define dCheckWorld(w) dCheck(w, dErrInvalidWorld)
#define dCheckWorldRet(w, r) dCheck(w, dErrInvalidWorld, r)

#define dCheckBody(b) dCheck(b, dErrInvalidBody)
#define dCheckBodyRet(b, r) dCheck(b, dErrInvalidBody, r)




#endif

// Local Variables:
// mode:c++
// End:

