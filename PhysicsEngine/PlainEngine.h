#pragma once
#include <SFML/Graphics.hpp>
#include <chrono>

class PlainEngine
{
protected:
    int AntiAliasing = 0;
    double MAX_FPS = 60;
    double TimeScale = 1;
    double fixedDeltaTime;

    sf::Color BackgroundColor = sf::Color::White;

    void Initialize(int width, int height, std::string title, int WindowStyle = sf::Style::Default, int AntiAliasing = 0)
    {
        sf::ContextSettings WindowSettings; WindowSettings.antialiasingLevel = AntiAliasing;
        Screen = new sf::RenderWindow(sf::VideoMode(width, height), title, WindowStyle, WindowSettings);
        Camera = sf::View(sf::Vector2f(0, 0), sf::Vector2f(width, height));
        Canvas = sf::View(sf::Vector2f(width / 2, height / 2), sf::Vector2f(width, height));

        startTime = std::chrono::system_clock::now();
        fixedDeltaTime = 1.0 / 60;
    }

    virtual void Start() {}
    virtual void Update(double deltaTime) = 0;
    virtual void Draw(sf::RenderWindow *&screen) = 0;
    virtual void UIDraw(sf::RenderWindow *&screen) {}
    virtual void Events(sf::Event &event) {}
    virtual void Quit() {}

    sf::RenderWindow *GetScreen()
    { return Screen; }

    sf::View &GetCamera()
    { return Camera; }

private:
    sf::RenderWindow *Screen;
    sf::Event Event;
    sf::View Camera;
    sf::View Canvas;

    std::chrono::system_clock::time_point startTime;
    std::chrono::duration<double> deltaTimeDuration;
    double deltaTime;

public:
    void Run()
    {
        Start();

        while (Screen->isOpen())
        {
            Screen->setView(Camera);
         
            while (Screen->pollEvent(Event))
            {
                if (Event.type == sf::Event::Closed)
                    Screen->close();

                Events(Event);
            }   

            deltaTimeDuration = std::chrono::system_clock::now() - startTime;
            deltaTime = deltaTimeDuration.count() * TimeScale;

            if(deltaTime >= 1.0 / MAX_FPS)
            {
                startTime = std::chrono::system_clock::now();

                Update(deltaTime);
            }

            Screen->clear(BackgroundColor);

            // WORLD VIEW CAMERA
            Draw(this->Screen);
            
            //CANVAS UI VIEW
            Screen->setView(Canvas);
            UIDraw(this->Screen);
            Screen->display();   
        }

        Quit();

        delete Screen;
    }
};
