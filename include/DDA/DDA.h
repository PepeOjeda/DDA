#include "Vector2Int.h"
#include "Vector3Int.h"

#include <vector>
#include <functional>
#include <iostream>

static int sign(float x)
{
    if (x > 0)
        return 1;
    if (x < 0)
        return -1;
    return 0;
}

static void resetColor()
{
    printf("\033[0m");
}

static void Error()
{
    printf("\033[1;31m");
    printf("[ERROR] ");
    resetColor();
}

static void Warn()
{
    printf("\033[1;33m");
    printf("[WARN] ");
    resetColor();
}

namespace DDA::_2D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::vector<std::pair<Vector2Int, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<Vector2Int, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    template <typename T> struct Map
    {
        std::vector<std::vector<T>> cells;
        Vector2 origin;
        float resolution;
    };

    // returns true if a blocked cell was hit. The outline of the map is considered blocked.
    template <typename T>
    RayCastInfo castRay(
        const Vector2& start, Vector2 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const Vector2&)>& positionPredicate = [](const Vector2& v) { return true; })
    {
        if (direction.norm() == 0)
        {
            Warn();
            printf("Ray of length 0\n");
            return {false, 0};
        }

        Vector2 currentPosition = start;
        Vector2Int currentCell = static_cast<Vector2Int>((currentPosition - map.origin) / map.resolution);
        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray outside the environment!\n");
            return {false, 0};
        }

        direction.normalize();
        int stepX = sign(direction.x);
        int stepY = sign(direction.y);

        float currentDistance = 0;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }

            if ((stepX != 0 && tX > 0 && tX < tY) || (stepY == 0 || tY <= 0))
            {
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else
            {
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            currentCell = static_cast<Vector2Int>((currentPosition - map.origin) / map.resolution);

            if ((currentPosition - start).norm() > maxDistance)
                return {false, maxDistance};
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
                return {true, currentDistance};
        }
    }

    // returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(
        const Vector2& start, Vector2 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const Vector2&)>& positionPredicate = [](const Vector2& v) { return true; })
    {
        if (direction.norm() == 0)
        {
            Warn();
            printf("Ray of length 0\n");

            return RayMarchInfo();
        }

        Vector2 currentPosition = start;
        Vector2Int currentCell = static_cast<Vector2Int>((currentPosition - map.origin) / map.resolution);

        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray origin in invalid position: (%f, %f)\n", start.x, start.y);

            return RayMarchInfo();
        }

        direction = direction / direction.norm();
        int stepX = sign(direction.x);
        int stepY = sign(direction.y);

        float currentDistance = 0;
        std::vector<std::pair<Vector2Int, float>> lengthsMap;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }

            if ((stepX != 0 && tX > 0 && tX < tY) || (stepY == 0 || tY <= 0))
            {
                lengthsMap.emplace_back(currentCell, tX);
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else
            {
                lengthsMap.emplace_back(currentCell, tY);
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            currentCell = static_cast<Vector2Int>((currentPosition - map.origin) / map.resolution);

            if ((currentPosition - start).norm() > maxDistance)
                return RayMarchInfo();
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_2D

namespace DDA::_3D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::vector<std::pair<Vector3Int, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<Vector3Int, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    template <typename T> struct Map
    {
        std::vector<std::vector<std::vector<T>>> cells;
        Vector3 origin;
        float resolution;
    };

    // returns true if a blocked cell was hit. The outline of the map is considered blocked.
    template <typename T>
    RayCastInfo castRay(
        const Vector3& start, Vector3 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const Vector3&)>& positionPredicate = [](const Vector3& v) { return true; })
    {
        if (direction.norm() == 0)
        {
            Warn();
            printf("Ray of length 0\n");
            return {false, 0};
        }

        Vector3 currentPosition = start;
        Vector3Int currentCell = static_cast<Vector3Int>((currentPosition - map.origin) / map.resolution);
        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() || !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) ||
            !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray origin in invalid position: (%f, %f, %f)\n", start.x, start.y, start.z);

            return {false, 0};
        }

        direction = direction / direction.norm();
        int stepX = sign(direction.x);
        int stepY = sign(direction.y);
        int stepZ = sign(direction.z);

        float currentDistance = 0;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;
            float zCoordNext = (stepZ > 0 ? currentCell.z + 1 : currentCell.z) * map.resolution + map.origin.z;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }
            float tZ = (zCoordNext - currentPosition.z) / direction.z;
            if (tZ <= 0)
            {
                zCoordNext += stepZ * map.resolution;
                tZ = (zCoordNext - currentPosition.z) / direction.z;
            }

            if ((stepX != 0 && tX > 0) && (tX < tY || stepY == 0 || tY <= 0) && (tX < tZ || stepZ == 0 || tZ <= 0))
            {
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else if ((stepY != 0 && tY > 0) && (tY < tZ || stepZ == 0 || tZ <= 0))
            {
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            else
            {
                currentPosition += direction * tZ;
                currentDistance += tZ;
            }

            currentCell = static_cast<Vector3Int>((currentPosition - map.origin) / map.resolution);

            if (currentDistance > maxDistance)
                return {false, maxDistance};
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) || !positionPredicate(currentPosition))
                return {true, currentDistance};
        }
    }

    // returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(
        const Vector3& start, Vector3 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const Vector3&)>& positionPredicate = [](const Vector3& v) { return true; })
    {
        if (direction.norm() == 0)
        {
            Warn();
            printf("Ray of length 0\n");

            return RayMarchInfo();
        }

        Vector3 currentPosition = start;
        Vector3Int currentCell = static_cast<Vector3Int>((currentPosition - map.origin) / map.resolution);

        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() || !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) ||
            !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray outside the environment!\n");

            return RayMarchInfo();
        }

        direction = direction / direction.norm();
        int stepX = sign(direction.x);
        int stepY = sign(direction.y);
        int stepZ = sign(direction.z);

        float currentDistance = 0;
        std::vector<std::pair<Vector3Int, float>> lengthsMap;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;
            float zCoordNext = (stepZ > 0 ? currentCell.z + 1 : currentCell.z) * map.resolution + map.origin.z;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }
            float tZ = (zCoordNext - currentPosition.z) / direction.z;
            if (tZ <= 0)
            {
                zCoordNext += stepZ * map.resolution;
                tZ = (zCoordNext - currentPosition.z) / direction.z;
            }

            if (stepX != 0 && (tX < tY || stepY == 0) && (tX < tZ || stepZ == 0))
            {
                if (tX > 0)
                    lengthsMap.emplace_back(currentCell, tX);
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else if (stepY != 0 && (tY < tZ || stepZ == 0))
            {
                if (tY > 0)
                    lengthsMap.emplace_back(currentCell, tY);
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            else
            {
                if (tZ > 0)
                    lengthsMap.emplace_back(currentCell, tZ);
                currentPosition += direction * tZ;
                currentDistance += tZ;
            }

            currentCell = static_cast<Vector3Int>((currentPosition - map.origin) / map.resolution);

            if (currentDistance > maxDistance)
                return RayMarchInfo();
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_3D
