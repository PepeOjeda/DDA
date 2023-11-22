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

        struct Vec3IntCompare
        {
            bool operator()(const Vector3Int& one, const Vector3Int& other) const
            {
                return one == other;
            }
        };
        struct Vec3IntHash
        {
            size_t operator()(const Vector3Int& one) const
            {
                return one.x * 7 + one.y * 4397;
            }
        };

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