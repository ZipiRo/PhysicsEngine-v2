#pragma once

bool NearlyEqual(float a, float b)
{
    return std::abs(b - a) < VerySmallAmount;
} 

namespace VectorMath
{
    Vector2D sfmlVector2DtoVector2D(sf::Vector2f vector) 
    {
        return Vector2D(vector.x, vector.y);
    }
    
    sf::Vector2f Vector2DtosfmlVector2D(Vector2D vector)
    {
        return sf::Vector2f(vector.x, vector.y);
    }
    
    bool NAN_Values(Vector2D vector)
    {
        return (std::isnan(vector.x) || std::isnan(vector.y));
    }
    
    Vector2D VectorTransformZ(Vector2D vector, Transform&transform)
    {
        return Vector2D(transform.cos * vector.x - transform.sin * vector.y + transform.position.x, 
                        transform.sin * vector.x + transform.cos * vector.y + transform.position.y);
    }
    
    float DistanceSquared(Vector2D a, Vector2D b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
    
        return dx * dx + dy * dy;
    }
    
    float LengthSquared(Vector2D vector)
    {
        return vector.x * vector.x + vector.y * vector.y;
    }
    
    bool NearlyEqualVectors(Vector2D a, Vector2D b)
    {
        return DistanceSquared(a, b) < VerySmallAmount * VerySmallAmount;
    } 
}