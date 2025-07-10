#include "Core.h"
#include "Renderer.h"
#include "YamlUtils.h"

#include <iostream>
#include <string>


// rewrite FxShape with inbuilt add_offset, get_bounds, and rotate. 
// and also write sat_collision using vectorized ops
// think about collison callback and step callback, provide as virtual override methods or as attributes



// custom function is called every time step
// void Fx_sceneloop(FxScene& scene, float dt){


// }


int main(int, char**){
    // FxVec2fArray a {{1.0f, 2.0f}, {1.2f, 2.3f}, {2.1f, 3.2f}};
    // std::cout<< a << std::endl;
    // std::cout<< -a*a << std::endl;
    // std::cout<< a + 1 << std::endl;
    // std::cout<< a + FxVec2f{1.0f, 2.0f} << std::endl;
    // std::cout<< 1/a << std::endl;



    // FxScene scene({12.0f, 8.0f});
    
    // auto body1 = std::make_shared<FxEntity>("body1");
    // scene.add_entity(body1);
    // body1->set_collision_geometry(0.5f);
    // body1->set_init_velocity({1, 6, 0});
    // // body1->visual_geometry()->set_fillTexture("./assets/ball.PNG");

    // auto body2 = std::make_shared<FxEntity>("body2");
    // body2->set_init_pose({6, 4, 0});

    // FxVisualShape bshape;
    // body2->set_collision_geometry(0.5f);
    // bshape.set_fillColor(sf::Color::Green);
    // bshape.set_outlineColor(sf::Color::Black);
    // body2->set_visual_geometry(bshape);


    // auto body3 = std::make_shared<FxEntity>("body3");
    // body3->set_init_pose({6, 2, 0});
    // scene.add_entity(body3);
    // FxShape cshape(FxVec2f{0.5f, 0.5f});
    // FxShape cshape2(FxVec2f{0.5f, 0.5f});
    // body3->set_visual_geometry(cshape);
    // body3->set_collision_geometry(cshape2);
    // // body3->set_init_velocity({1, 6, 0});


    // std::cout<<body2->gravity_scale<<std::endl;

    // scene.add_entity(body2);
    // // scene.set_step_callback(Fx_sceneloop);

    // auto bod3 = scene.get_entity("body2");
    auto scene = FxYAML::buildScene("./Scene.yml");

    Renderer renderer(scene);
    // renderer.set_background("./assets/bg.png");
    renderer.run();

    return 0;
}
