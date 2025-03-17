#pragma once

class Vector2D
{
public:
    float x, y;

    Vector2D();
    Vector2D(float x, float y);
    
    Vector2D &operator=(const Vector2D vector);
    bool operator!() const;
    bool operator==(const Vector2D& vector);
    bool operator!=(const Vector2D& vector);
    
    Vector2D operator-() const;
    Vector2D operator+(const Vector2D vector) const;
    Vector2D operator-(const Vector2D vector) const;
    Vector2D operator*(float scalar) const;
    Vector2D operator/(float scalar) const;
    Vector2D &operator+=(const Vector2D& vector);
    Vector2D &operator-=(const Vector2D& vector);
    Vector2D &operator*=(float scalar);
    Vector2D &operator/=(float scalar);

    static float Length(const Vector2D &vector);
    static Vector2D Normalize(Vector2D vector);
    static float Distance(const Vector2D& a, const Vector2D& b);
    static float DotProduct(const Vector2D& a, const Vector2D& b);
    static float CrossProduct(const Vector2D& a, const Vector2D& b);

    const static Vector2D ZERO;
    const static Vector2D UP;
    const static Vector2D DOWN;
    const static Vector2D RIGHT;
    const static Vector2D LEFT;
};

const Vector2D Vector2D::ZERO = Vector2D(0, 0);
const Vector2D Vector2D::UP = Vector2D(0, -1);
const Vector2D Vector2D::DOWN = Vector2D(0, 1);
const Vector2D Vector2D::RIGHT = Vector2D(1, 0);
const Vector2D Vector2D::LEFT = Vector2D(-1, 0);

Vector2D::Vector2D() 
{
    this->x = 0;
    this->y = 0;
}

Vector2D::Vector2D(float x, float y) 
{
    this->x = x;
    this->y = y;
}

float Vector2D::Length(const Vector2D &vector)
{
    return sqrt(vector.x * vector.x + vector.y * vector.y);
}

float Vector2D::Distance(const Vector2D& a, const Vector2D& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;

    return sqrt(dx * dx + dy * dy);
}

Vector2D Vector2D::Normalize(Vector2D vector)
{
    float length = Vector2D::Length(vector);
    vector.x /= length;
    vector.y /= length;
    
    return vector;
}

float Vector2D::DotProduct(const Vector2D& a, const Vector2D& b) 
{
    return a.x * b.x + a.y * b.y;
}

float Vector2D::CrossProduct(const Vector2D& a, const Vector2D& b)
{
    return a.x * b.y - a.y * b.x;
}

Vector2D& Vector2D::operator=(const Vector2D vector) 
{
    if (this != &vector) 
    {
        this->x = vector.x;
        this->y = vector.y;
    }
    return *this;
}

bool Vector2D::operator==(const Vector2D& vector)
{
    return (this->x == vector.x && this->y == vector.y);
}

bool Vector2D::operator!=(const Vector2D& vector)
{
    return (this->x != vector.x && this->y != vector.y);
}

bool Vector2D::operator!() const
{
    return (!this->x && !this->y);
}

Vector2D Vector2D::operator-() const {
    return Vector2D(-this->x, -this->y);
}

Vector2D Vector2D::operator+(const Vector2D vector) const 
{
    return Vector2D(this->x + vector.x, this->y + vector.y);
}

Vector2D Vector2D::operator-(const Vector2D vector) const 
{
    return Vector2D(this->x - vector.x, this->y - vector.y);
}

Vector2D operator*(float scalar, const Vector2D& vector) 
{
    return Vector2D(scalar * vector.x, scalar * vector.y);
}

Vector2D operator/(float scalar, const Vector2D& vector) 
{
    return Vector2D(scalar / vector.x, scalar / vector.y);
}

Vector2D Vector2D::operator*(float scalar) const 
{
    return Vector2D(this->x * scalar, this->y * scalar);
}

Vector2D Vector2D::operator/(float scalar) const 
{
    return Vector2D(this->x / scalar, this->y / scalar);
}

Vector2D& Vector2D::operator+=(const Vector2D& vector) 
{
    this->x += vector.x;
    this->y += vector.y;
    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& vector) 
{
    this->x -= vector.x;
    this->y -= vector.y;
    return *this;
}

Vector2D& Vector2D::operator*=(float scalar) 
{
    this->x *= scalar;
    this->y *= scalar;
    return *this;
}

Vector2D& Vector2D::operator/=(float scalar) 
{
    this->x /= scalar;
    this->y /= scalar;
    return *this;
}

Vector2D multipyVector2(const Vector2D a, const Vector2D vector) 
{
    return Vector2D(a.x * vector.x, a.y * vector.y);
}

Vector2D divideVector2(const Vector2D a, const Vector2D vector) 
{
    return Vector2D(a.x / vector.x, a.y / vector.y);
}