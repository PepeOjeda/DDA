#pragma once
#include "Map.h"
#include <functional>

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
        const glm::vec3& start, glm::vec3 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const glm::vec3&)>& positionPredicate = [](const glm::vec3& v) { return true; })
    {
        if (glm::length(direction) == 0)
        {
            Internal::Warn("Ray of length 0\n");
            return {false, 0};
        }

        float invResolution = 1. / map.resolution;
        glm::vec3 currentPosition = start;
        glm::ivec3 currentCell = static_cast<glm::ivec3>((currentPosition - map.origin) * invResolution);
        if (currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 || currentCell.y >= map.dimensions.y || currentCell.z < 0 ||
            currentCell.z >= map.dimensions.z)
        {
            Internal::Error("Ray outside the environment!\n");

            return {false, 0};
        }

        if (!mapPredicate(map.at(currentCell.x, currentCell.y, currentCell.z)) || !positionPredicate(currentPosition))
        {
            Internal::Error("Ray starts inside an obstacle!\n");
            return {false, 0};
        }

        direction = glm::normalize(direction);
        glm::vec3 invDirection(1. / direction.x, 1. / direction.y, 1. / direction.z);

        int stepX = Internal::sign(direction.x);
        int stepY = Internal::sign(direction.y);
        int stepZ = Internal::sign(direction.z);

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

            currentCell = static_cast<glm::ivec3>((currentPosition - map.origin) * invResolution);

            if (currentDistance > maxDistance || currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 ||
                currentCell.y >= map.dimensions.y || currentCell.z < 0 || currentCell.z >= map.dimensions.z)
                return {false, maxDistance};
            else if (!mapPredicate(map.at(currentCell.x, currentCell.y, currentCell.z)) || !positionPredicate(currentPosition))
                return {true, currentDistance};
        }
    }
} // namespace DDA::_3D