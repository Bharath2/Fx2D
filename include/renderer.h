#pragma once 

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "imgui-SFML.h"

#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>
#include <memory>

#include "physics.h" 


class Renderer {
private:
    unsigned int m_display_w, m_display_h;
    // Helper methods for drawing UI parts and the scene.
    void drawUI(FxScene &scene);
    void drawScene(FxScene &scene);
    // sf window and background varaibles
    std::unique_ptr<sf::RenderWindow> m_window;
    std::unique_ptr<sf::Sprite> m_backgroundSprite;
    std::unique_ptr<sf::Texture> m_backgroundTexture;
    sf::Color m_backgroundColor {50, 50, 50, 255};

public:
    // Constructor, Destructor
    Renderer(unsigned int display_w, unsigned int display_h);
    ~Renderer();

    void run(FxScene &scene);   // The main entry point to start the application loop
    void set_background(const ImVec4 &color);
    void set_background(const std::string &filepath);

};
