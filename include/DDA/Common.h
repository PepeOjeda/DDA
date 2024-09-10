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
    printf("\033[0m");
}

inline void Error()
{
    printf("\033[1;31m");
    printf("[ERROR] ");
    resetColor();
}

inline void Warn()
{
    printf("\033[1;33m");
    printf("[WARN] ");
    resetColor();
}