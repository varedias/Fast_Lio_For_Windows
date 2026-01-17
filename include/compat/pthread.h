/*
 * pthread.h compatibility for Windows/MSVC
 * Maps POSIX threads API to Windows threads
 */

#ifndef PTHREAD_COMPAT_H
#define PTHREAD_COMPAT_H

#if defined(_WIN32) || defined(_MSC_VER)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <process.h>

// ==========================================
// Thread Types
// ==========================================
typedef HANDLE pthread_t;
typedef CRITICAL_SECTION pthread_mutex_t;
typedef CONDITION_VARIABLE pthread_cond_t;

// Attributes (simplified - not fully implemented)
typedef struct {
    int detachstate;
} pthread_attr_t;

typedef struct {
    int dummy;
} pthread_mutexattr_t;

typedef struct {
    int dummy;
} pthread_condattr_t;

// ==========================================
// Constants
// ==========================================
#define PTHREAD_CREATE_JOINABLE 0
#define PTHREAD_CREATE_DETACHED 1
#define PTHREAD_MUTEX_INITIALIZER {0}

// ==========================================
// Thread Functions
// ==========================================

// Thread creation wrapper structure
typedef struct {
    void *(*start_routine)(void *);
    void *arg;
} pthread_start_info;

static unsigned __stdcall pthread_start_wrapper(void *arg) {
    pthread_start_info *info = (pthread_start_info *)arg;
    void *(*start_routine)(void *) = info->start_routine;
    void *thread_arg = info->arg;
    free(info);
    start_routine(thread_arg);
    return 0;
}

static inline int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                                  void *(*start_routine)(void *), void *arg) {
    pthread_start_info *info = (pthread_start_info *)malloc(sizeof(pthread_start_info));
    if (!info) return -1;
    info->start_routine = start_routine;
    info->arg = arg;
    
    *thread = (HANDLE)_beginthreadex(NULL, 0, pthread_start_wrapper, info, 0, NULL);
    if (*thread == NULL) {
        free(info);
        return -1;
    }
    return 0;
}

static inline int pthread_join(pthread_t thread, void **retval) {
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    if (retval) *retval = NULL;
    return 0;
}

static inline int pthread_detach(pthread_t thread) {
    CloseHandle(thread);
    return 0;
}

static inline pthread_t pthread_self(void) {
    return GetCurrentThread();
}

// ==========================================
// Mutex Functions
// ==========================================

static inline int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr) {
    (void)attr;
    InitializeCriticalSection(mutex);
    return 0;
}

static inline int pthread_mutex_destroy(pthread_mutex_t *mutex) {
    DeleteCriticalSection(mutex);
    return 0;
}

static inline int pthread_mutex_lock(pthread_mutex_t *mutex) {
    EnterCriticalSection(mutex);
    return 0;
}

static inline int pthread_mutex_trylock(pthread_mutex_t *mutex) {
    return TryEnterCriticalSection(mutex) ? 0 : EBUSY;
}

static inline int pthread_mutex_unlock(pthread_mutex_t *mutex) {
    LeaveCriticalSection(mutex);
    return 0;
}

// ==========================================
// Condition Variable Functions
// ==========================================

static inline int pthread_cond_init(pthread_cond_t *cond, const pthread_condattr_t *attr) {
    (void)attr;
    InitializeConditionVariable(cond);
    return 0;
}

static inline int pthread_cond_destroy(pthread_cond_t *cond) {
    (void)cond;
    // Windows condition variables don't need explicit destruction
    return 0;
}

static inline int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex) {
    SleepConditionVariableCS(cond, mutex, INFINITE);
    return 0;
}

static inline int pthread_cond_signal(pthread_cond_t *cond) {
    WakeConditionVariable(cond);
    return 0;
}

static inline int pthread_cond_broadcast(pthread_cond_t *cond) {
    WakeAllConditionVariable(cond);
    return 0;
}

// ==========================================
// Attribute Functions (stubs)
// ==========================================

static inline int pthread_attr_init(pthread_attr_t *attr) {
    attr->detachstate = PTHREAD_CREATE_JOINABLE;
    return 0;
}

static inline int pthread_attr_destroy(pthread_attr_t *attr) {
    (void)attr;
    return 0;
}

static inline int pthread_attr_setdetachstate(pthread_attr_t *attr, int detachstate) {
    attr->detachstate = detachstate;
    return 0;
}

#endif // _WIN32 || _MSC_VER

#endif // PTHREAD_COMPAT_H
