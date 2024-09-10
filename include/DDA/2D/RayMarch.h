#pragma once
#include "Map.h"

namespace DDA::_2D
{
    struct RayMarchInfo
    {
        std::vector<std::pair<Vector2Int, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<Vector2Int, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

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
        Vector2Int currentCell = Vector2Int((currentPosition - map.origin) / map.resolution);

        if (currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 || currentCell.y >= map.dimensions.y)
        {
            Error();
            printf("Ray outside the environment!\n");
            return RayMarchInfo();
        }

        if (!mapPredicate(map.at(currentCell.x, currentCell.y)) || !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray starts inside an obstacle!\n");
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

            currentCell = Vector2Int((currentPosition - map.origin) / map.resolution);

            if (currentDistance > maxDistance || currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 ||
                currentCell.y >= map.dimensions.y)
                return RayMarchInfo();
            else if (!mapPredicate(map.at(currentCell.x, currentCell.y)) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_2D