#include "Core.h"
#include "Renderer.h"
#include <cstdio>
#include <string>

#include <iostream>
#include <execution>


Renderer::Renderer(FxScene &scene): scene(scene), m_scale(100) { init(); }
Renderer::Renderer(FxScene &scene, unsigned int scale): scene(scene), m_scale(scale) { init(); }
 
void Renderer::init(){
    m_display_w = m_scale*scene.size.x();
    m_display_h = m_scale*scene.size.y();

    // Create an SFML window exactly as in your sample.
    m_window = std::make_unique<sf::RenderWindow>(sf::VideoMode(m_display_w, m_display_h), "Fx2D");
    m_window->setFramerateLimit(120);

    // Initialize ImGui-SFML with the window.
    if (!ImGui::SFML::Init(*m_window)) {
        throw std::runtime_error("Failed to initialize ImGui-SFML");
    }
}


Renderer::~Renderer() { ImGui::SFML::Shutdown(); }

void Renderer::run() {
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
        float curr_rt_factor = 0.0f;
        if(m_play) curr_rt_factor = scene.step(deltaTime);
        // Update the ImGui frame.
        ImGui::SFML::Update(*m_window, sf::seconds(deltaTime));
        m_window->clear(m_backgroundColor);
        if(m_backgroundSprite != nullptr){
            m_window->draw(*m_backgroundSprite);
        }
        // Draw the simulation scene.
        draw_scene();
        draw_ui(curr_rt_factor);
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

void Renderer::draw_scene() {
    // First create shapes from the entities
    std::vector<std::unique_ptr<sf::Shape>> shapes;
    scene.transform_entities(std::execution::par, [&](auto entity) -> std::unique_ptr<sf::Shape> {
        //edit with shpae
        auto visual = entity->visual_geometry();
        if (!visual) { return nullptr; } // skip if no visual geometry

        FxVec3f offset_pose = visual->offset_pose();
        FxVec3f current_pose = entity->pose + offset_pose; // current pose in world coordinates
        // convert FxVisualShape to sf::Shape 
        std::unique_ptr<sf::Shape> shape; 
        const std::string& shape_type = visual->shape_type();
        if (shape_type == "Circle") {
            const float& radius = m_scale * visual->radius();
            shape = std::make_unique<sf::CircleShape>(radius);
            shape->setOrigin(radius, radius);
        } else {
            const FxVec2fArray& vertices = visual->vertices();
            auto polygon = std::make_unique<sf::ConvexShape>(vertices.size());
            // To compute the centroid (average of transformed vertices)
            sf::Vector2f centroid(0.f, 0.f);
            for (size_t i = 0; i < vertices.size(); ++i) {
                float transformed_x = m_scale * vertices[i].x();
                float transformed_y = m_display_h - m_scale * vertices[i].y(); 
                polygon->setPoint(i, sf::Vector2f(transformed_x, transformed_y));
                centroid.x += transformed_x;
                centroid.y += transformed_y;
            }
            // Average the accumulated sum.
            centroid.x /= vertices.size();
            centroid.y /= vertices.size();
            polygon->setOrigin(centroid);
            shape = std::move(polygon);
        } 
        // move to object position and set rotation, color, texture.
        auto fillTexture = visual->fillTexture();
        if(fillTexture){
            shape->setTexture(fillTexture.get(), false);
        } else {
            shape->setFillColor(visual->fillColor());
        }
        shape->setOutlineThickness(visual->outlineThickness());
        shape->setOutlineColor(visual->outlineColor());
        shape->setPosition(m_scale*current_pose.x(), (m_display_h - m_scale*current_pose.y()));
        shape->setRotation(current_pose.theta());
        return shape;
    }, shapes);

    // Draw these shapes on the window
    for (const auto& shape : shapes) {
        m_window->draw(*shape);
    }
}

void Renderer::draw_ui(float curr_rt_factor) {
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 8));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 10));
    ImGui::Begin("Simulation Controls");
    ImGui::SetWindowFocus("Simulation Controls");
    char buf[32];
    // grab the current gravity so we can edit it locally
    static float gravity_x = scene.gravity.x();
    static float gravity_y = scene.gravity.y();
    // ---- GRAVITY X ----
    ImGui::Text("Gravity X:");
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputFloat("##GravityX", &gravity_x, 0.0f, 0.0f, "%.1f")) {
        // clamp into range
        gravity_x = std::clamp(gravity_x, -20.0f, 20.0f);
        scene.gravity.set_x(gravity_x);
    }
    // ---- GRAVITY Y ----
    ImGui::Text("Gravity Y:");
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputFloat("##GravityY", &gravity_y, 0.0f, 0.0f, "%.1f")) {
        gravity_y = std::clamp(gravity_y, -20.0f, 20.0f);
        scene.gravity.set_y(gravity_y);
    }
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::Separator();

    ImGui::Text("Real Time Factor:");
    static float rt_factor = 1.0f;
    std::snprintf(buf, sizeof(buf), " : %.3f", curr_rt_factor);
    ImGui::SetNextItemWidth(100.0f);   
    if (ImGui::InputFloat(buf, &rt_factor, 0.0f, 0.0f, "%.3f")) {
        // Clamp the rt_factor so it never goes below 0.001.
        if (rt_factor < 0.001f)
            rt_factor = 0.001f;
        scene.set_real_time_factor(rt_factor);
    }
    ImGui::Dummy(ImVec2(0, 5));
    ImGui::Separator();
    // ←–– Pause / Play toggle button
    const char* label = m_play ? "Pause" : "Play";
    if (ImGui::Button(label, ImVec2(150,0))) {
        m_play = !m_play;
    }
    if (ImGui::Button("Reset Simulation", ImVec2(150, 0))) {
        scene.reset();
    }
    ImGui::Dummy(ImVec2(0, 1));
    ImGui::Separator();

    float frame_rate = ImGui::GetIO().Framerate;
    ImGui::Text("Performance:\n %.3f ms/frame\n (%.1f FPS)",
                1000.0f / frame_rate, frame_rate);
    ImGui::End();
    ImGui::PopStyleVar(2);
}

