#pragma once
#include "Vector3.h"

namespace DDA
{
    struct Vector3Int
    {
        int x, y, z;
        Vector3Int()
        {
            x = y = z = 0;
        }
        Vector3Int(int i, int j, int k)
        {
            x = i;
            y = j;
            z = k;
        }
        Vector3Int(const Vector3Int& other)
        {
            x = other.x;
            y = other.y;
            z = other.z;
        }

        explicit Vector3Int(const Vector3& point) : x(point.x), y(point.y), z(point.z)
        {}

        bool operator==(const Vector3Int& other) const
        {
            return other.x == x && other.y == y && other.z == z;
        }

        bool operator!=(const Vector3Int& other) const
        {
            return other.x != x || other.y != y || other.z != z;
        }

        inline Vector3Int operator+(const Vector3Int& other) const
        {
            return Vector3Int(x + other.x, y + other.y, z + other.z);
        }

        inline Vector3Int operator-(const Vector3Int& other) const
        {
            return Vector3Int(x - other.x, y - other.y, z - other.z);
        }

        inline float norm() const
        {
            return std::sqrt(x * x + y * y);
        }

        operator Vector3() const
        {
            return Vector3(x, y, z);
        }
    };

    inline Vector3Int operator*(const Vector3Int& p, const float& f)
    {
        return Vector3Int(p.x * f, p.y * f, p.z * f);
    }
    inline Vector3Int operator*(const float& f, const Vector3Int& p)
    {
        return p * f;
    }
} // namespace DDA


template<>
struct std::hash<DDA::Vector3Int>
{
    size_t operator()(const DDA::Vector3Int& vec) const
    {
        return *(size_t*) &vec.x + *(size_t*)(&vec.y) + (*(size_t*)(&vec.z) << 32);
    }
};