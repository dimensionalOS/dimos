#include "commons.h"

void PoseWithTime::setTime(int32_t _sec, uint32_t _nsec)
{
    sec = _sec;
    nsec = _nsec;
    second = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
}
