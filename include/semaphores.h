#ifndef SEMAPHORES_H
#define SEMAPHORES_H

#ifdef __APPLE__
#include <dispatch/dispatch.h>
#else
#include <semaphore.h>
#endif

#ifdef __APPLE__
typedef dispatch_semaphore_t semaphore;
#else
typedef sem_t semaphore;
#endif

static inline void semaphore_init(semaphore *sem, unsigned int value) {
  #ifdef __APPLE__
  *sem = dispatch_semaphore_create(value);
#else
  sem_init(sem, 0, value);
#endif
}

static inline void semaphore_post(semaphore *sem) {
#ifdef __APPLE__
  dispatch_semaphore_signal(*sem);
#else
  sem_post(sem);
#endif
}

static inline void semaphore_wait(semaphore *sem) {
#ifdef __APPLE__
  dispatch_semaphore_wait(*sem, DISPATCH_TIME_FOREVER);
#else
  sem_wait(sem);
#endif
}

static inline void semaphore_destroy(semaphore *sem) {
#ifdef __APPLE__
#else
  sem_destroy(sem);
#endif
}

#endif
