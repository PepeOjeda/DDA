#pragma once
#include "Map.h"

namespace DDA::_2D
{
    struct RayMarchInfo
    {
        std::vector<std::pair<glm::ivec2, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<glm::ivec2, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    // returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(
        const glm::vec2& start, glm::vec2 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const glm::vec2&)>& positionPredicate = [](const glm::vec2& v) { return true; })
    {
        if (glm::length(direction) == 0)
        {
            Internal::Warn("Ray of length 0\n");

            return RayMarchInfo();
        }

        float invResolution = 1. / map.resolution;
        glm::vec2 currentPosition = start;
        glm::ivec2 currentCell = glm::ivec2((currentPosition - map.origin) * invResolution);

        if (currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 || currentCell.y >= map.dimensions.y)
        {
            Internal::Error("Ray outside the environment!\n");
            return RayMarchInfo();
        }

        if (!mapPredicate(map.at(currentCell.x, currentCell.y)) || !positionPredicate(currentPosition))
        {
            Internal::Error("Ray starts inside an obstacle!\n");
            return RayMarchInfo();
        }

        direction = glm::normalize(direction);
        glm::vec2 invDirection(1. / direction.x, 1. / direction.y);

        int stepX = Internal::sign(direction.x);
        int stepY = Internal::sign(direction.y);

        float currentDistance = 0;
        std::vector<std::pair<glm::ivec2, float>> lengthsMap;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;

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

            currentCell = glm::ivec2((currentPosition - map.origin) * invResolution);

            if (currentDistance > maxDistance || currentCell.x < 0 || currentCell.x >= map.dimensions.x || currentCell.y < 0 ||
                currentCell.y >= map.dimensions.y)
                return RayMarchInfo();
            else if (!mapPredicate(map.at(currentCell.x, currentCell.y)) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_2D