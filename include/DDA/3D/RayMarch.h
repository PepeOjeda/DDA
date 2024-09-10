#pragma once
#include "Map.h"

namespace DDA::_3D
{

    struct RayMarchInfo
    {
        std::vector<std::pair<Vector3Int, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<Vector3Int, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };


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

        if (currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 || currentCell.y >= map.dimensions.y ||
            currentCell.z < 0 || currentCell.z >= map.dimensions.z)
        {
            Error();
            printf("Ray origin in invalid position: (%f, %f, %f)\n", start.x, start.y, start.z);

            return RayMarchInfo();
        }
            
        if(!mapPredicate(map.at(currentCell.x, currentCell.y, currentCell.z)) || !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray starts inside an obstacle!\n");
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

            if (currentDistance > maxDistance || currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 ||
                currentCell.y >= map.dimensions.y || currentCell.z < 0 || currentCell.z >= map.dimensions.z)
                return RayMarchInfo();
            else if (!mapPredicate(map.at(currentCell.x, currentCell.y, currentCell.z)) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_3D
