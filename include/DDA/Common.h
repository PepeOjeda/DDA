#pragma once
#include <cstdio>

namespace DDA
{
    inline bool loggingEnabled = true;
    namespace Internal
    {
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

        inline void Error(const char* msg)
        {
            if (loggingEnabled)
                return;
            fprintf(stderr, "\033[1;31m");
            fprintf(stderr, "[ERROR] ");
            resetColor();
            fprintf(stderr, "%s", msg);
        }

        inline void Warn(const char* msg)
        {
            if (loggingEnabled)
                return;
            fprintf(stderr, "\033[1;33m");
            fprintf(stderr, "[WARN] ");
            resetColor();
            fprintf(stderr, "%s", msg);
        }
    } // namespace Internal
} // namespace DDA