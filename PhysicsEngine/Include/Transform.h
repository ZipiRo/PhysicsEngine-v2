#pragma once

class Transform
{
    public:
        Vector2D position;
        float sin;
        float cos;

        Transform() 
        {
            this->position = Vector2D(0, 0);
            this->cos = 1.0f;
            this->sin = 0.0f;
        }

        Transform(Vector2D position, float angle)
        {
            this->position = position;
            this->sin = sinf(angle * DEG_TO_RAD);
            this->cos = cosf(angle * DEG_TO_RAD);
        }

        Transform(float x, float y, float angle)
        {
            this->position = Vector2D(x, y);
            
            this->sin = sinf(angle * DEG_TO_RAD);
            this->cos = cosf(angle * DEG_TO_RAD);
        }
};