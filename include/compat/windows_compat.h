#ifndef WINDOWS_COMPAT_H
#define WINDOWS_COMPAT_H

/*
 * Windows/MSVC Compatibility Layer for FAST-LIO
 * Provides POSIX-like functions that are missing on Windows/MSVC
 * This allows compilation with MSVC while using POSIX-style code
 */

#if defined(_WIN32) || defined(_MSC_VER) || defined(__MINGW32__) || defined(__MINGW64__)

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <io.h>
#include <direct.h>
#include <process.h>

// Windows API for sleep functions
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// ==========================================
// Type definitions for POSIX compatibility
// ==========================================
#ifndef uint
typedef unsigned int uint;
#endif

#ifndef uchar
typedef unsigned char uchar;
#endif

#ifndef ushort
typedef unsigned short ushort;
#endif

#ifndef ulong
typedef unsigned long ulong;
#endif

// ==========================================
// unistd.h replacements for MSVC
// ==========================================
#ifdef _MSC_VER

// Process ID
#ifndef getpid
#define getpid _getpid
#endif

// File access modes
#ifndef F_OK
#define F_OK 0
#endif
#ifndef X_OK
#define X_OK 1
#endif
#ifndef W_OK
#define W_OK 2
#endif
#ifndef R_OK
#define R_OK 4
#endif

// access function
#ifndef access
#define access _access
#endif

// getcwd function
#ifndef getcwd
#define getcwd _getcwd
#endif

// chdir function
#ifndef chdir
#define chdir _chdir
#endif

// rmdir function
#ifndef rmdir
#define rmdir _rmdir
#endif

// unlink function
#ifndef unlink
#define unlink _unlink
#endif

// NOTE: Do NOT define close macro here!
// It would conflict with C++ method names like DataReader::close()
// Use _close() directly where needed for file descriptors

// NOTE: Do NOT define read/write macros here!
// They would conflict with std::istream::read() and std::ostream::write()
// which are used by PCL and other libraries.
// If you need POSIX read/write for file descriptors, use _read/_write directly.

// isatty function
#ifndef isatty
#define isatty _isatty
#endif

// fileno function
#ifndef fileno
#define fileno _fileno
#endif

// Sleep functions
// usleep: microseconds -> Sleep uses milliseconds
static inline void usleep(unsigned int us) {
    if (us >= 1000) {
        Sleep(us / 1000);
    } else if (us > 0) {
        Sleep(1); // Minimum 1ms
    }
}

// sleep: seconds
static inline unsigned int sleep(unsigned int seconds) {
    Sleep(seconds * 1000);
    return 0;
}

#endif // _MSC_VER

// ==========================================
// err.h replacements (BSD functions)
// ==========================================

// warn() - print warning message with errno
#ifndef warn
static inline void warn(const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[WARN] ");
    if (fmt != NULL) {
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
    if (errno != 0) {
        fprintf(stderr, ": %s", strerror(errno));
    }
    fprintf(stderr, "\n");
    fflush(stderr);
}
#endif

// warnx() - print warning message without errno
#ifndef warnx
static inline void warnx(const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[WARN] ");
    if (fmt != NULL) {
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
    fprintf(stderr, "\n");
    fflush(stderr);
}
#endif

// err() - print error message with errno and exit
#ifndef err
static inline void err(int eval, const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[ERROR] ");
    if (fmt != NULL) {
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
    if (errno != 0) {
        fprintf(stderr, ": %s", strerror(errno));
    }
    fprintf(stderr, "\n");
    fflush(stderr);
    exit(eval);
}
#endif

// errx() - print error message without errno and exit
#ifndef errx
static inline void errx(const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[ERROR] ");
    if (fmt != NULL) {
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
    fprintf(stderr, "\n");
    fflush(stderr);
    exit(1);
}
#endif

// ==========================================
// Signal handling compatibility
// ==========================================
#ifndef SIGPIPE
#define SIGPIPE 13  // Broken pipe (not used on Windows, but define it)
#endif

#ifndef SIGHUP
#define SIGHUP 1    // Hangup
#endif

#ifndef SIGQUIT
#define SIGQUIT 3   // Quit
#endif

#ifndef SIGKILL
#define SIGKILL 9   // Kill (cannot be caught or ignored)
#endif

#ifndef SIGALRM
#define SIGALRM 14  // Alarm clock
#endif

// ==========================================
// Other POSIX compatibility
// ==========================================

// ssize_t for MSVC
#ifdef _MSC_VER
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

// M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// Disable some MSVC warnings that are common with POSIX code
#ifdef _MSC_VER
#pragma warning(disable: 4996)  // 'function': was declared deprecated
#pragma warning(disable: 4244)  // conversion, possible loss of data
#pragma warning(disable: 4267)  // conversion from 'size_t' to 'type'
#pragma warning(disable: 4305)  // truncation from 'double' to 'float'
#endif

#endif // _WIN32

#endif // WINDOWS_COMPAT_H
