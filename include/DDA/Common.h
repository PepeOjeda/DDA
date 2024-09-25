#pragma once 
#include <functional>
#include <iostream>

inline int sign(float x)
{
    if (x > 0)
        return 1;
    if (x < 0)
        return -1;
    return 0;
}

inline void resetColor()
{
    fprintf(stderr, "\033[0m");
}

inline void Error()
{
    fprintf(stderr, "\033[1;31m");
    fprintf(stderr, "[ERROR] ");
    resetColor();
}

inline void Warn()
{
    fprintf(stderr, "\033[1;33m");
    fprintf(stderr, "[WARN] ");
    resetColor();
}