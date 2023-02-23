#include <vector>
#include <glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/hash.hpp"

#include <unordered_map>

namespace DDA
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::unordered_map<glm::ivec2, float> lengthInCell;
        RayMarchInfo(){}
        RayMarchInfo(std::unordered_map<glm::ivec2, float> inputMap): lengthInCell(std::move(inputMap)){}
    };

    //returns true if a blocked cell was hit. The outline of the map is considered blocked.
    RayCastInfo castRay(const glm::vec2& start, glm::vec2 direction, const float maxDistance, const std::vector<std::vector<bool>>& map, const glm::vec2& mapOrigin, const float mapResolution);
    //returns how far through each cell the ray has traveled. Useful for volumetric calculations
    RayMarchInfo marchRay(const glm::vec2& start, glm::vec2 direction, const float maxDistance, const std::vector<std::vector<bool>>& map, const glm::vec2& mapOrigin, const float mapResolution);

}