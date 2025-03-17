#pragma once

class AABB
{
    public:
        Vector2D min;
        Vector2D max;

        AABB () 
        {
            this->min = Vector2D(0, 0);
            this->max = Vector2D(0, 0);
        }

        AABB (Vector2D min, Vector2D max)
        {
            this->min = min;
            this->max = max;
        }

        AABB (float minX, float minY, float maxX, float maxY)
        {
            this->min = Vector2D(minX, minY);
            this->max = Vector2D(maxX, maxY);
        }
};