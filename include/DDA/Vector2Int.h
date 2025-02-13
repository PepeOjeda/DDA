#pragma once
#include "Vector2.h"

namespace DDA
{
    struct Vector2Int
    {
        int x, y;
        Vector2Int()
        {
            x = y = 0;
        }
        Vector2Int(int i, int j)
        {
            x = i;
            y = j;
        }
        Vector2Int(const Vector2Int& other)
        {
            x = other.x;
            y = other.y;
        }

        explicit Vector2Int(const Vector2& point) : x(point.x), y(point.y)
        {}

        bool operator==(const Vector2Int& other) const
        {
            return other.x == x && other.y == y;
        }

        bool operator!=(const Vector2Int& other) const
        {
            return other.x != x || other.y != y;
        }

        inline Vector2Int operator+(const Vector2Int& other) const
        {
            return Vector2Int(x + other.x, y + other.y);
        }

        inline Vector2Int operator-(const Vector2Int& other) const
        {
            return Vector2Int(x - other.x, y - other.y);
        }

        inline Vector2Int operator/(const float& other) const
        {
            return Vector2Int(x / other, y / other);
        }

        inline float norm() const
        {
            return std::sqrt(x * x + y * y);
        }

        explicit operator Vector2() const
        {
            return Vector2(x, y);
        }
    };

    inline Vector2Int operator*(const Vector2Int& p, const float& f)
    {
        return Vector2Int(p.x * f, p.y * f);
    }
    inline Vector2Int operator*(const float& f, const Vector2Int& p)
    {
        return p * f;
    }
} // namespace DDA


template<>
struct std::hash<DDA::Vector2Int>
{
    size_t operator()(const DDA::Vector2Int& vec) const
    {
        return (size_t) vec.x + ((size_t)(vec.y) << 32);
    }
};