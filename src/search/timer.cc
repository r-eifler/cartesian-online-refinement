#include "timer.h"

#include <ostream>
#include <unistd.h>

#ifndef _WIN32
    #include <sys/times.h>
#else
    #include <windows.h>
#endif

using namespace std;

Timer::Timer() {
    last_start_clock = current_clock();
    collected_time = 0;
    stopped = false;
}

Timer::~Timer() {
}

double Timer::current_clock() const {
#ifndef _WIN32
    struct tms the_tms;
    times(&the_tms);
    clock_t clocks = the_tms.tms_utime + the_tms.tms_stime;
    return double(clocks) / sysconf(_SC_CLK_TCK);
#else
    //http://nadeausoftware.com/articles/2012/03/c_c_tip_how_measure_cpu_time_benchmarking
    FILETIME createTime;
    FILETIME exitTime;
    FILETIME kernelTime;
    FILETIME userTime;
    if ( GetProcessTimes( GetCurrentProcess( ),
        &createTime, &exitTime, &kernelTime, &userTime ) != -1 )
    {
        SYSTEMTIME userSystemTime;
        if ( FileTimeToSystemTime( &userTime, &userSystemTime ) != -1 )
            return (double)userSystemTime.wHour * 3600.0 +
                (double)userSystemTime.wMinute * 60.0 +
                (double)userSystemTime.wSecond +
                (double)userSystemTime.wMilliseconds / 1000.0;
    }
    return -1;
#endif
}

double Timer::stop() {
    collected_time = (*this)();
    stopped = true;
    return collected_time;
}

double Timer::operator()() const {
    if (stopped)
        return collected_time;
    else
        return collected_time + current_clock() - last_start_clock;
}

void Timer::resume() {
    if (stopped) {
        stopped = false;
        last_start_clock = current_clock();
    }
}

double Timer::reset() {
    double result = (*this)();
    collected_time = 0;
    last_start_clock = current_clock();
    return result;
}

ostream &operator<<(ostream &os, const Timer &timer) {
    double value = timer();
    if (value < 0 && value > -1e-10)
        value = 0.0;  // We sometimes get inaccuracies from god knows where.
    if (value < 1e-10)
        value = 0.0;  // Don't care about such small values.
    os << value << "s";
    return os;
}
