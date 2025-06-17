#include "renderer.h"
#include "physics.h"


// fix green object does not move error
// edit drawScene to change with shape
int main(int, char**){
    Renderer renderer(1200, 800);
    renderer.set_background("./assets/bg.png");

    FxScene scene({12.0f, 8.0f});
    
    auto body1 = std::make_shared<FxEntity>("body1");
    scene.add_entity(body1);

    auto body2 = std::make_shared<FxEntity>("body2");
    body2->set_init_pose({600, 400, 0});

    FxVisualShape bshape;
    bshape.set_fillColor(sf::Color::Green);
    bshape.set_outlineColor(sf::Color::Black);
    body2->set_visual(bshape);

    std::cout<<body2->gravity_scale<<std::endl;

    scene.add_entity(body2);

    auto bod3 = scene.get_entity("body2");
    std::cout<<bod3->pos<<std::endl;
    bod3->pos += FxVec2f {10, 10};
    std::cout<<bod3->pos<<std::endl;

    std::cout<<body2->pos<<std::endl;
    // std::cout<<bod3->pos<<std::endl;
    // body2->visual.def = sf::CircleShape(100.f)

    //     sf::CircleShape shape(100.f);
    // shape.setFillColor(sf::Color::Green);
    // shape.setPosition(600.f, 450.f); // Draw the circle at (600,450)
    // shape.setOutlineThickness(5.f);
    // // Set the border (outline) color (e.g., black).
    // shape.setOutlineColor(sf::Color::Black);
    // m_window->draw(shape);

    // auto bd = scene.get_entity("body1");
   
    renderer.run(scene);

    return 0;
}
