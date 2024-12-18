#pragma once 
#include "Map.h"

namespace DDA::_3D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
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
            fprintf(stderr, "Ray of length 0\n");
            return {false, 0};
        }

        float invResolution = 1. / map.resolution;
        Vector3 currentPosition = start;
        Vector3Int currentCell = static_cast<Vector3Int>((currentPosition - map.origin) * invResolution);
        if (currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 || currentCell.y >= map.dimensions.y ||
            currentCell.z < 0 || currentCell.z >= map.dimensions.z)
        {
            Error();
            fprintf(stderr, "Ray origin in invalid position: (%f, %f, %f)\n", start.x, start.y, start.z);

            return {false, 0};
        }
            
        if(!mapPredicate(map.at(currentCell.x, currentCell.y, currentCell.z)) || !positionPredicate(currentPosition))
        {
            Error();
            fprintf(stderr, "Ray starts inside an obstacle!\n");
            return {false, 0};
        }

        direction = direction / direction.norm();
        Vector3 invDirection(1. / direction.x, 1. / direction.y, 1. / direction.z);

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
            float tX = (xCoordNext - currentPosition.x) * invDirection.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) * invDirection.x;
            }
            float tY = (yCoordNext - currentPosition.y) * invDirection.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) * invDirection.y;
            }
            float tZ = (zCoordNext - currentPosition.z) * invDirection.z;
            if (tZ <= 0)
            {
                zCoordNext += stepZ * map.resolution;
                tZ = (zCoordNext - currentPosition.z) * invDirection.z;
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

            currentCell = static_cast<Vector3Int>((currentPosition - map.origin) * invResolution);

            if (currentDistance > maxDistance || currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 ||
                currentCell.y >= map.dimensions.y || currentCell.z < 0 || currentCell.z >= map.dimensions.z)
                return {false, maxDistance};
            else if (!mapPredicate(map.at(currentCell.x, currentCell.y, currentCell.z)) || !positionPredicate(currentPosition))
                return {true, currentDistance};
        }
    }
}