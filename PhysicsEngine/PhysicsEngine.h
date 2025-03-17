#include "PlainEngine.h"
#include <iostream>
#include <list>
#include <vector>
#include <cmath>

const double PI = 3.141592635389f;
const float infinity = 3.4028234e38f;
const float DEG_TO_RAD = PI / 180.0f;
const float RAD_TO_DEG = 180.0f / PI;
float VerySmallAmount = 0.00001f;
const float pixelsPerMeter = 10.0f;
const float gravityAcceleration = 9.81;

#include "Include/Vector2D.h"
#include "Include/Transform.h"
#include "Include/VectorMath.h"
#include "Include/AABB.h"
#include "Include/Body.h"
#include "Include/Manifold.h"
#include "Include/Circle.h"
#include "Include/Rectangle.h"
#include "Include/Collisions.h"
#include "Include/World.h"

class APP : public PlainEngine
{
private:
    World world;

    sf::Font font;  
    sf::Text frameRateUI;
    sf::Text bodyCountUI;
    double frameTimer = 1;

    bool lPressed = false;
    int currentBody = 1;

    Vector2D screenSize;
    sf::Vector2i mouseLocalPos;
    sf::Vector2f mouseWorldPos;

    void Start() override
    {
        srand(time(0));
        if(!font.loadFromFile("Resources/Fonts/arial.ttf"));
        GetCamera().zoom(0.2);
        screenSize = VectorMath::sfmlVector2DtoVector2D(sf::Vector2f(GetScreen()->getSize()));

        frameRateUI = sf::Text("", font);
        frameRateUI.setCharacterSize(15);
        frameRateUI.move(10, 10);
        
        bodyCountUI = sf::Text("Bodyes: 0", font);
        bodyCountUI.setCharacterSize(15);
        bodyCountUI.move(10, 30);

        Body *Ground = new Rectangle(200.0f, 3.0f, Vector2D(0, 0), 1.0f, 0.5f, sf::Color::White, sf::Color::Transparent, true);
        world.AddBody(Ground);
    }

    void Events(sf::Event &event) override
    {
        if(event.type == sf::Event::EventType::MouseWheelScrolled)
        {
            if(event.mouseWheelScroll.delta < 0)
            {
                GetCamera().zoom(1.2);
            }
            else if(event.mouseWheelScroll.delta > 0)
            {
                GetCamera().zoom(0.9);
            }
        }
    }

    void Update(double deltaTime) override
    {   
        mouseLocalPos = sf::Mouse::getPosition(*GetScreen());
        mouseWorldPos = GetScreen()->mapPixelToCoords(mouseLocalPos);
        frameTimer += deltaTime;
        if(frameTimer >= 1)
        {
            frameRateUI.setString(std::to_string(int(1.0f / deltaTime)) + " FPS");
            frameTimer = 0;
        }
        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            GetCamera().move(2, 0);
        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            GetCamera().move(-2, 0);
            
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            GetCamera().move(0, -2);
        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            GetCamera().move(0, 2);

        if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
                if(!lPressed)
                {
                    CreateBody();
                    bodyCountUI.setString("Bodyes: " + std::to_string(world.BodyCount()));
                    lPressed = true;
                }
            }
            else {
                if(lPressed) lPressed = false;
            }

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num1))
            {
                currentBody = 1;
                std::cout << "CREATING RECTANGLES" << '\n';
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num2))
            {
                currentBody = 2;
                std::cout << "CREATING CIRCLES" << '\n';
            }

            world.Step(fixedDeltaTime, 10);

            for(int i = 0; i < world.BodyCount(); i++)
            {
                Body *body = world.GetBody(i);
                AABB aabb = body->GetAABB();

                if(aabb.max.y > 150)
                {
                    world.RemoveBody(body);
                }
            }
    }

    void Draw(sf::RenderWindow *&Screen) override
    {
        for(int i = 0; i < world.BodyCount(); i++)
        {
            Body *body = world.GetBody(i);
            body->Draw(Screen);
        }
    }

    void UIDraw(sf::RenderWindow *&Screen) override
    {
        Screen->draw(frameRateUI);
        Screen->draw(bodyCountUI);
    }

    void CreateBody()
        {   
            Body *body;
            int r = 0 + rand() % (255 - 0 + 1);
            int g = 0 + rand() % (255 - 0 + 1);
            int b = 0 + rand() % (255 - 0 + 1);

            if(currentBody == 1)
            {
                float width = 3 + rand() % (5 - 3 + 1);
                float height = 3 + rand() % (5 - 3 + 1);

                body = new Rectangle(width, height, VectorMath::sfmlVector2DtoVector2D(mouseWorldPos), 1.0f, 1.0f, sf::Color(r, g, b), sf::Color::Transparent, false);
                world.AddBody(body);
            }
            else if(currentBody == 2)
            {
                float radius = 2 + rand() % (3 - 2 + 1);

                body = new Circle(radius, VectorMath::sfmlVector2DtoVector2D(mouseWorldPos), 10.0f, 1.0f, sf::Color(r, g, b), sf::Color::Transparent, false);
                world.AddBody(body);
            }
        }

public:
    APP()
    {
        BackgroundColor = sf::Color::Black;
        TimeScale = 1;
        fixedDeltaTime = 1 / 120;
        MAX_FPS = 120;
        Initialize(1280, 720, "DEMO");
    }
};