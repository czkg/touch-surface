#ifndef _UTIL_H
#define _UTIL_H

#include <chrono>

namespace TouchDetectionUtils {

    typedef std::chrono::time_point<std::chrono::system_clock> Time;
    typedef std::chrono::milliseconds Duration;

    inline Time now() { return std::chrono::system_clock::now(); }
    inline int betweenMs(const Time start, const Time end) {
        return std::chrono::duration_cast<Duration>(end - start).count();
    }
    inline int untilNowMs(const Time start) {
        return betweenMs(start, now());
    }

} /* TouchDetectionUtils */

#endif /* end of include guard: _UTIL_H */
