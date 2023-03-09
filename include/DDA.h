#include <vector>
#include "glm/glm.hpp"
#include "glm/common.hpp"
#include <functional>

#include <unordered_map>

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
    
namespace DDA::_2D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::vector<std::pair<glm::ivec2, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0) {}
        RayMarchInfo(std::vector<std::pair<glm::ivec2, float>> inputMap, float length)
            : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    //returns true if a blocked cell was hit. The outline of the map is considered blocked.    
    template<typename T>
    RayCastInfo castRay(const glm::vec2& start, glm::vec2 direction, const float maxDistance, const std::vector<std::vector<T>>& map, std::function<bool(T)> predicate, const glm::vec2& mapOrigin, const float mapResolution)
    {
        if(glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");
            return {false,0};
        }
        
        glm::ivec2 currentCell = (start-mapOrigin)/mapResolution;
        if( currentCell.x<0 ||currentCell.x>=map.size()||
            currentCell.y<0 ||currentCell.y>=map[0].size()||
            !predicate(map[currentCell.x][currentCell.y]) )
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

            //how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext-currentPosition.x)/direction.x;
            if(tX<=0)
            {
                xCoordNext += stepX*mapResolution;
                tX = (xCoordNext-currentPosition.x)/direction.x;
            }
            float tY = (yCoordNext-currentPosition.y)/direction.y;
            if(tY<=0)
            {
                yCoordNext += stepY*mapResolution;
                tY = (yCoordNext-currentPosition.y)/direction.y;
            }

            if((stepX!=0 && tX>0 && tX<tY) || (stepY==0 || tY <= 0))
            {
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else
            {
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            currentCell = glm::floor( (currentPosition-mapOrigin)/mapResolution );


            if(glm::length(currentPosition-start)>maxDistance)
                return {false, maxDistance};
            else if (currentCell.x<0 || currentCell.x>=map.size()
                    || currentCell.y<0 || currentCell.y>=map[0].size()
                    || !predicate( map[currentCell.x][currentCell.y] ) )
                return {true, currentDistance};
        }
    }
    
    //returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(const glm::vec2& start, glm::vec2 direction, const float maxDistance, const std::vector<std::vector<T>>& map,std::function<bool(T)> predicate, const glm::vec2& mapOrigin, const float mapResolution)
    {
        if(glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");

            return RayMarchInfo();
        }
        
        glm::ivec2 currentCell = (start-mapOrigin)/mapResolution;
        
        if(currentCell.x<0 ||currentCell.x>=map.size()||
            currentCell.y<0 ||currentCell.y>=map[0].size()||
            !predicate( map[currentCell.x][currentCell.y] ) )
        {
            Error();
            printf("Ray origin in invalid position: (%f, %f)\n", start.x, start.y);   

            return RayMarchInfo();
        }

        glm::vec2 currentPosition = start;

        direction = direction/glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);
        
        float currentDistance = 0;
        std::vector<std::pair<glm::ivec2, float>> lengthsMap; 
        while(true)
        {
            float xCoordNext = (stepX>0? currentCell.x+1 : currentCell.x) * mapResolution + mapOrigin.x;
            float yCoordNext = (stepY>0? currentCell.y+1 : currentCell.y) * mapResolution + mapOrigin.y;

            //how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext-currentPosition.x)/direction.x;
            if(tX<=0)
            {
                xCoordNext += stepX*mapResolution;
                tX = (xCoordNext-currentPosition.x)/direction.x;
            }
            float tY = (yCoordNext-currentPosition.y)/direction.y;
            if(tY<=0)
            {
                yCoordNext += stepY*mapResolution;
                tY = (yCoordNext-currentPosition.y)/direction.y;
            }



            if((stepX!=0 && tX>0 && tX<tY) || (stepY==0 || tY <= 0))
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
            currentCell = glm::floor( (currentPosition-mapOrigin)/mapResolution );

            if(glm::length(currentPosition-start)>maxDistance)
                return RayMarchInfo();
            else if (currentCell.x<0 || currentCell.x>=map.size()
                    || currentCell.y<0 || currentCell.y>=map[0].size()
                    || !predicate( map[currentCell.x][currentCell.y] ) )
                return {lengthsMap, currentDistance};
        }
    }
}


namespace DDA::_3D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::vector<std::pair<glm::ivec3, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0){}
        RayMarchInfo(std::vector<std::pair<glm::ivec3, float>> inputMap, float length): 
            lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    //returns true if a blocked cell was hit. The outline of the map is considered blocked.    
    template<typename T>
    RayCastInfo castRay(const glm::vec3& start, glm::vec3 direction, const float maxDistance, const std::vector<std::vector<std::vector<T>>>& map, std::function<bool(T)> predicate, const glm::vec3& mapOrigin, const float mapResolution)
    {
        if(glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");
            return {false,0};
        }
        
        glm::ivec3 currentCell = (start-mapOrigin)/mapResolution;
        if( currentCell.x<0 ||currentCell.x>=map.size()||
            currentCell.y<0 ||currentCell.y>=map[0].size()||
            currentCell.z<0 ||currentCell.z>=map[0][0].size()||
            !predicate(map[currentCell.x][currentCell.y][currentCell.z]) )
        {
            Error();            
            printf("Ray origin in invalid position: (%f, %f, %f)\n", start.x, start.y, start.z);   

            return {false,0};
        }
        glm::vec3 currentPosition = start;

        direction = direction/glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);
        int stepZ = glm::sign(direction.z);
        
        float currentDistance = 0;
        while(true)
        {
            float xCoordNext = (stepX>0? currentCell.x+1 : currentCell.x) * mapResolution + mapOrigin.x;
            float yCoordNext = (stepY>0? currentCell.y+1 : currentCell.y) * mapResolution + mapOrigin.y;
            float zCoordNext = (stepZ>0? currentCell.z+1 : currentCell.z) * mapResolution + mapOrigin.z;

            //how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext-currentPosition.x)/direction.x;
            if(tX<=0)
            {
                xCoordNext += stepX*mapResolution;
                tX = (xCoordNext-currentPosition.x)/direction.x;
            }
            float tY = (yCoordNext-currentPosition.y)/direction.y;
            if(tY<=0)
            {
                yCoordNext += stepY*mapResolution;
                tY = (yCoordNext-currentPosition.y)/direction.y;
            }
            float tZ = (zCoordNext-currentPosition.z)/direction.z;
            if(tZ<=0)
            {
                zCoordNext += stepZ*mapResolution;
                tZ = (zCoordNext-currentPosition.z)/direction.z;
            }



            if((stepX!=0 && tX>0) && (tX<tY || stepY==0 || tY <= 0) && (tX<tZ || stepZ==0 || tZ <= 0))
            {
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else if ((stepY!=0 && tY>0) && (tY<tZ || stepZ==0 || tZ <= 0))
            {
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            else
            {
                currentPosition += direction * tZ;
                currentDistance += tZ;
            }

            currentCell = glm::floor( (currentPosition-mapOrigin)/mapResolution );
        

            if(currentDistance>maxDistance)
                return {false, maxDistance};
            else if (currentCell.x<0 || currentCell.x>=map.size()
                    || currentCell.y<0 || currentCell.y>=map[0].size()
                    || currentCell.z<0 || currentCell.z>=map[0][0].size()
                    || !predicate( map[currentCell.x][currentCell.y][currentCell.z] ) )
                return {true, currentDistance};
        }
    }
    
    //returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(const glm::vec3& start, glm::vec3 direction, const float maxDistance, const std::vector<std::vector<std::vector<T>>>& map,std::function<bool(T)> predicate, const glm::vec3& mapOrigin, const float mapResolution)
    {
        if(glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");

            return RayMarchInfo();
        }
        
        glm::ivec3 currentCell = (start-mapOrigin)/mapResolution;
        
        if( currentCell.x<0 ||currentCell.x>=map.size()||
            currentCell.y<0 ||currentCell.y>=map[0].size()||
            currentCell.z<0 ||currentCell.z>=map[0][0].size()||
            !predicate(map[currentCell.x][currentCell.y][currentCell.z]) )
        {
            Error();
            printf("Ray outside the environment!\n");   

            return RayMarchInfo();
        }

        glm::vec3 currentPosition = start;

        direction = direction/glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);
        int stepZ = glm::sign(direction.z);
        
        float currentDistance = 0;
        std::vector<std::pair<glm::ivec3, float>> lengthsMap; 
        while(true)
        {
            float xCoordNext = (stepX>0? currentCell.x+1 : currentCell.x) * mapResolution + mapOrigin.x;
            float yCoordNext = (stepY>0? currentCell.y+1 : currentCell.y) * mapResolution + mapOrigin.y;
            float zCoordNext = (stepZ>0? currentCell.z+1 : currentCell.z) * mapResolution + mapOrigin.z;

            //how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext-currentPosition.x)/direction.x;
            if(tX<=0)
            {
                xCoordNext += stepX*mapResolution;
                tX = (xCoordNext-currentPosition.x)/direction.x;
            }
            float tY = (yCoordNext-currentPosition.y)/direction.y;
            if(tY<=0)
            {
                yCoordNext += stepY*mapResolution;
                tY = (yCoordNext-currentPosition.y)/direction.y;
            }
            float tZ = (zCoordNext-currentPosition.z)/direction.z;
            if(tZ<=0)
            {
                zCoordNext += stepZ*mapResolution;
                tZ = (zCoordNext-currentPosition.z)/direction.z;
            }
            
            if(stepX!=0 && (tX<tY|| stepY==0) && (tX<tZ||stepZ==0) )
            {
                if(tX>0)
                    lengthsMap.emplace_back(currentCell, tX);
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else if (stepY != 0 && (tY<tZ || stepZ==0))
            {
                if(tY>0)
                    lengthsMap.emplace_back(currentCell, tY);
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            else
            {
                if(tZ>0)
                    lengthsMap.emplace_back(currentCell, tZ);
                currentPosition += direction * tZ;
                currentDistance += tZ;
            }

            currentCell = glm::floor( (currentPosition-mapOrigin)/mapResolution );
            
            if(currentDistance>maxDistance)
                return RayMarchInfo();
            else if (currentCell.x<0 || currentCell.x>=map.size()
                    || currentCell.y<0 || currentCell.y>=map[0].size()
                    || currentCell.z<0 || currentCell.z>=map[0][0].size()
                    || !predicate( map[currentCell.x][currentCell.y][currentCell.z] ) )
                return {lengthsMap, currentDistance};
        }
    }
    
}
