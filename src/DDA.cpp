#include <DDA.h>
#include <iostream>

static void resetColor () {
  printf("\033[0m");
}

static void Error () {
  printf("\033[1;31m");
  printf("[ERROR] ");
  resetColor();
}

static void Warn() {
  printf("\033[1;33m");
  printf("[WARN] ");
  resetColor();
}



namespace DDA{

RayCastInfo castRay(const glm::vec2& start, glm::vec2 direction, const float maxDistance, const std::vector<std::vector<bool>>& map, const glm::vec2& mapOrigin, const float mapResolution)
{
    if(glm::length(direction) == 0)
    {
        Warn();
        printf("Ray of length 0\n");
        return {false,0};
    }
    
    glm::ivec2 currentCell = (start-mapOrigin)/mapResolution;
    if(!map[currentCell.x][currentCell.y])
    {
        Error();
        printf("Ray outside the environment!\n"); 
        return {false,0};
    }
    glm::vec2 currentPosition = start;

    direction = direction/glm::length(direction);
    int stepX = glm::sign(direction.x);
    int stepY = glm::sign(direction.y);
    
    float currentDistance = 0;
    while(true)
    {
        float xCoordNext = (stepX>0? currentCell.x+1 : currentCell.x) * mapResolution + mapOrigin.x;
        float yCoordNext = (stepY>0? currentCell.y+1 : currentCell.y) * mapResolution + mapOrigin.y;
        float tMaxX = (xCoordNext-currentPosition.x)/direction.x;
        float tMaxY = (yCoordNext-currentPosition.y)/direction.y;
        if((stepX!=0 && tMaxX<tMaxY)|| stepY==0)
        {
            currentPosition += direction * tMaxX;
            currentCell += glm::ivec2{stepX, 0};
            currentDistance += tMaxX;
        }
        else
        {
            currentPosition += direction * tMaxY;
            currentCell += glm::ivec2{0, stepY};
            currentDistance += tMaxY;
        }

        if(glm::length(currentPosition-start)>maxDistance)
            return {false, maxDistance};
        else if (currentCell.x<0 || currentCell.x>=map.size()
                || currentCell.y<0 || currentCell.y>=map[0].size()
                || !map[currentCell.x][currentCell.y])
            return {true, currentDistance};
    }
}

RayMarchInfo marchRay(const glm::vec2& start, glm::vec2 direction, const float maxDistance, const std::vector<std::vector<bool>>& map, const glm::vec2& mapOrigin, const float mapResolution)
{
    if(glm::length(direction) == 0)
    {
        Warn();
        printf("Ray of length 0\n");
        return RayMarchInfo();
    }
    
    glm::ivec2 currentCell = (start-mapOrigin)/mapResolution;
    
    if(!map[currentCell.x][currentCell.y])
    {
        Error();
        printf("Ray outside the environment!\n");   
        return RayMarchInfo();
    }

    glm::vec2 currentPosition = start;

    direction = direction/glm::length(direction);
    int stepX = glm::sign(direction.x);
    int stepY = glm::sign(direction.y);
    
    float currentDistance = 0;
    std::unordered_map<glm::ivec2, float> lengthsMap; 
    while(true)
    {
        float xCoordNext = (stepX>0? currentCell.x+1 : currentCell.x) * mapResolution + mapOrigin.x;
        float yCoordNext = (stepY>0? currentCell.y+1 : currentCell.y) * mapResolution + mapOrigin.y;
        float tMaxX = (xCoordNext-currentPosition.x)/direction.x;
        float tMaxY = (yCoordNext-currentPosition.y)/direction.y;
        if((stepX!=0 && tMaxX<tMaxY)|| stepY==0)
        {
            if(tMaxX>0)
                lengthsMap.insert({currentCell, tMaxX});
            currentPosition += direction * tMaxX;
            currentCell += glm::ivec2{stepX, 0};
            currentDistance += tMaxX;
        }
        else
        {
            if(tMaxY>0)
                lengthsMap.insert({currentCell, tMaxY});
            currentPosition += direction * tMaxY;
            currentCell += glm::ivec2{0, stepY};
            currentDistance += tMaxY;
        }

        if(glm::length(currentPosition-start)>maxDistance)
            return RayMarchInfo();
        else if (currentCell.x<0 || currentCell.x>=map.size()
                || currentCell.y<0 || currentCell.y>=map[0].size()
                || !map[currentCell.x][currentCell.y])
            return {lengthsMap};
    }
}

}

