#pragma once

class Body
{
    public:
        enum { PolygonShape, CircleShape};
        
        Vector2D position;
        Vector2D linearVelocity;
        float angle;
        float angularVelocity;

        Vector2D force;

        float width;
        float height;
        float radius;

        float inertia;
        float invInertia;

        float surface;
        float density;
        float mass;
        float invMass;
        float restitution;
        float area;

        float staticFriction;
        float dinamicFriction;

        bool isStatic;

        bool UPDATE_VERTICES;
        bool UPDATE_AABB;

        sf::Color fillColor;
        sf::Color outlineColor;
        sf::Shape* shape;

        int shapeType;

        Body() {};
        
        virtual void Draw(sf::RenderWindow* window) = 0;
        virtual std::vector<Vector2D> GetTransformedVertices() = 0;
        virtual AABB GetAABB() = 0;

        void Move(Vector2D amount) {
            this->position += amount;
            this->UPDATE_VERTICES = true;
            this->UPDATE_AABB = true;
        }

        void MoveTo(Vector2D position) {
            this->position = position;
            this->UPDATE_VERTICES = true;
            this->UPDATE_AABB = true;
        }

        void Rotate(float amount) {
            this->angle += amount;
            this->UPDATE_VERTICES = true;
            this->UPDATE_AABB = true;
        }

        void RotateTo(float angle) {
            this->angle = angle;
            this->UPDATE_VERTICES = true;
            this->UPDATE_AABB = true;
        }

        void AddForce(Vector2D amount) {
            this->force += amount;
        }

        void SetFillColor(sf::Color color) {
            this->fillColor = color;
            this->shape->setFillColor(this->fillColor);
        }

        void SetOutlineColor(sf::Color color) {
            this->outlineColor = color;
            this->shape->setOutlineColor(this->outlineColor);
        }

        void Step(float deltaTime, int totalIterations) {
            if(this->isStatic) return;
            
            deltaTime /= float(totalIterations);

            Vector2D acceleration = this->force / this->mass;

            this->linearVelocity += acceleration * deltaTime;
            this->position += this->linearVelocity * deltaTime;
            this->angle += this->angularVelocity * RAD_TO_DEG * deltaTime;
            
            this->force = Vector2D(0, 0);

            this->UPDATE_VERTICES = true;
            this->UPDATE_AABB = true;
        }
};