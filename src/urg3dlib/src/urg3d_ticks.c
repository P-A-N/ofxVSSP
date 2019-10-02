#include "urg3d_ticks.h"
#include "urg3d_detect_os.h"
#include <time.h>

#ifdef URG3D_MAC_OS
#include <mach/clock.h>
#include <mach/mach.h>
#endif

void gettime(struct timespec *ts)
{
  #ifdef URG3D_MAC_OS // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;

  #else
  #ifndef URG3D_WINDOWS_OS
  clock_gettime(CLOCK_REALTIME, ts);
  #endif
  #endif
}


long urg3d_ticks_ms(void)
{
    static int is_initialized = 0;
#if defined(URG3D_WINDOWS_OS)
    clock_t current_clock;
#else
    static struct timespec first_spec;
    struct timespec current_spec;
#endif
    long time_ms;

#if defined(URG3D_WINDOWS_OS)
    if (is_initialized == 0) {
        is_initialized = 1;
    }
    current_clock = clock();
    time_ms = current_clock / (CLOCKS_PER_SEC / 1000);
#else
    if (is_initialized == 0) {
        gettime(&first_spec);
        is_initialized = 1;
    }
    gettime(&current_spec);
    time_ms =
        (current_spec.tv_sec - first_spec.tv_sec) * 1000
        + (current_spec.tv_nsec - first_spec.tv_nsec) / 1000000;
#endif
    return time_ms;
}
