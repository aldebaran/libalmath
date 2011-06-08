


#ifndef _ALMATH_API_HPP_
#define _ALMATH_API_HPP_

#include <qi/api.hpp>

// almath_EXPORTS controls which symbols are exported when libqi
// is compiled as a SHARED lib.
// DO NOT USE OUTSIDE LIBQI
#ifdef almath_EXPORTS
# define ALMATH_API QI_EXPORT_API
#else
# define ALMATH_API QI_IMPORT_API
#endif

#endif

