#pragma once 

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "imgui-SFML.h"

#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>
#include <memory>

#include "Core.h" 
#include "MathTypes.h"


class Renderer {
private:
    FxScene &scene;
    unsigned int m_scale;
    unsigned int m_display_w, m_display_h;
    // sf window and background varaibles
    std::unique_ptr<sf::RenderWindow> m_window;
    std::unique_ptr<sf::Sprite> m_backgroundSprite;
    std::unique_ptr<sf::Texture> m_backgroundTexture;
    sf::Color m_backgroundColor {50, 50, 50, 255};
    // sets if the scene.step is called or not
    bool m_play = true;
    // Helper methods for drawing UI parts and the scene.
    void init();
    void draw_ui(float curr_rt_factor);
    void draw_scene();

public:
    // Constructor, Destructor
    Renderer(FxScene &scene); // scene, scale is set to 100 pixels/unit
    Renderer(FxScene &scene, unsigned int scale); // scene and scale pixels/unit
    ~Renderer();
    // The main entry point to start the application loop
    void run();   
    void set_background(const ImVec4 &color);
    void set_background(const std::string &filepath);
};
