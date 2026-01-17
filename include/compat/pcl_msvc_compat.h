/*
 * PCL Compatibility fixes for MSVC 2022+
 * Include this BEFORE any PCL headers
 */

#ifndef PCL_MSVC_COMPAT_H
#define PCL_MSVC_COMPAT_H

#if defined(_MSC_VER) && _MSC_VER >= 1940

// For MSVC 19.40+ (VS2022 17.10+), the _read and _write functions
// are no longer exposed in <ostream>/<istream>. PCL's eigen.hpp
// uses these through a macro expansion issue.
// 
// This is actually a PCL bug where it checks for _MSC_VER and tries
// to use MSVC-specific stream functions. We need to ensure PCL
// uses the standard write()/read() functions.

// Disable PCL's MSVC-specific code paths
#ifndef PCL_FORCE_STANDARD_OSTREAM
#define PCL_FORCE_STANDARD_OSTREAM 1
#endif

#endif // _MSC_VER >= 1940

// Suppress C4819 warnings for Eigen files with non-ASCII characters
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4819)  // file contains characters that cannot be represented
#pragma warning(disable: 4267)  // conversion from 'size_t' to 'type'
#pragma warning(disable: 4244)  // conversion, possible loss of data
#endif

#endif // PCL_MSVC_COMPAT_H
