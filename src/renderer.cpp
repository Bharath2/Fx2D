#include "renderer.h" 
#include "utils.h"

#include <iostream>
#include <execution>

Renderer::Renderer(unsigned int display_w, unsigned int display_h)
    : m_display_w(display_w), m_display_h(display_h) {
    // Create an SFML window exactly as in your sample.
    m_window = std::make_unique<sf::RenderWindow>(sf::VideoMode(m_display_w, m_display_h), "Fx2D");
    m_window->setFramerateLimit(60);

    // Initialize ImGui-SFML with the window.
    if (!ImGui::SFML::Init(*m_window)) {
        throw std::runtime_error("Failed to initialize ImGui-SFML");
    }
}

Renderer::~Renderer() {
    ImGui::SFML::Shutdown();
}

void Renderer::run(FxScene &scene) {
    sf::Clock deltaClock;

    // Main loop.
    while (m_window->isOpen()) {
        sf::Event event;
        while (m_window->pollEvent(event)) {
            ImGui::SFML::ProcessEvent(*m_window, event);
            if (event.type == sf::Event::Closed) {
                m_window->close();
            }
        }
        float deltaTime = deltaClock.restart().asSeconds();

        // Update the physics of scene.
        scene.step(deltaTime);

        // Update the ImGui frame.
        ImGui::SFML::Update(*m_window, sf::seconds(deltaTime));
        m_window->clear(m_backgroundColor);
        if(m_backgroundSprite != nullptr){
            m_window->draw(*m_backgroundSprite);
        }

        // Draw the simulation scene.
        drawScene(scene);
        drawUI(scene);

        // Render ImGui draw data.
        ImGui::SFML::Render(*m_window);

        // Display the frame.
        m_window->display();
    }
}

void Renderer::set_background(const ImVec4 &color){
    // set color and delete the background texture
    m_backgroundColor = sf::Color(color);
    m_backgroundSprite.reset();
    m_backgroundTexture.reset();
}

void Renderer::set_background(const std::string &filepath) {
    // Make sure your pointers are allocated.
    if (!m_backgroundSprite)
        m_backgroundSprite = std::make_unique<sf::Sprite>();
    if (!m_backgroundTexture)
        m_backgroundTexture = std::make_unique<sf::Texture>();

    // Get the window size for scaling 
    sf::Vector2u windowSize = m_window->getSize();
    // Attempt to load the texture from file.
    if (!m_backgroundTexture->loadFromFile(filepath)) {
        std::cerr << "Failed to load background texture from file: " << filepath << std::endl;
        // Fallback: create an image filled with a default color (dark gray: (50,50,50))
        sf::Image defaultImage;
        defaultImage.create(windowSize.x, windowSize.y, sf::Color(50, 50, 50));
        m_backgroundTexture->loadFromImage(defaultImage);
    } 
    else {
        // If loaded successfully, set the sprite scale so that it fills the window.
        sf::Vector2u textureSize = m_backgroundTexture->getSize();
        m_backgroundSprite->setScale(
            static_cast<float>(windowSize.x) / textureSize.x,
            static_cast<float>(windowSize.y) / textureSize.y
        );
    }
    m_backgroundSprite->setTexture(*m_backgroundTexture, true);
}


void Renderer::drawScene(FxScene &scene) {
    // First create shapes from the entities
    std::vector<std::unique_ptr<sf::Shape>> shapes;
    scene.transform_entities(std::execution::seq, [](auto entity) -> std::unique_ptr<sf::Shape> {
        //edit with shpae
        auto visual = entity->visual();
        auto shape = std::make_unique<sf::CircleShape>(100.f);
        shape->setPosition(entity->pos.x(), entity->pos.y());
        shape->setFillColor(visual->fillColor());
        shape->setOutlineThickness(5.f);
        shape->setOutlineColor(visual->outlineColor());
        return shape;
    }, shapes);

    // Draw these shapes on the window
    for (const auto& shape : shapes) {
        m_window->draw(*shape);
    }
}

void Renderer::drawUI(FxScene &scene) {

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 8));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 10));

    ImGui::Begin("Simulation Controls");
    ImGui::SetWindowFocus("Simulation Controls");

    // sliders for controlling gravity
    float gravity_x = scene.gravity.x();
    float gravity_y = scene.gravity.y();
    ImGui::Text("Gravity_X:");
    ImGui::SliderFloat("##Gravity_X", &(gravity_x), -20.0f, 20.0f, "%.1f");
    ImGui::Text("Gravity_Y:");
    ImGui::SliderFloat("##Gravity_Y", &(gravity_y), -20.0f, 20.0f, "%.1f");
    scene.gravity.set_x(gravity_x);
    scene.gravity.set_y(gravity_y);

    ImGui::Dummy(ImVec2(0, 10));
    ImGui::Separator();

    if (ImGui::Button("Reset Simulation", ImVec2(150, 0))) {
        scene.reset();
    }

    ImGui::Dummy(ImVec2(0, 10));
    ImGui::Separator();

    float frame_rate = ImGui::GetIO().Framerate;
    ImGui::Text("Application Performance:\n %.3f ms/frame\n (%.1f FPS)",
                1000.0f / frame_rate, frame_rate);

    ImGui::End();
    ImGui::PopStyleVar(2);
}

