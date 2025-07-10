#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "MathTypes.h"

// Parse a YAML sequence of length N into a FxArray<float>.
template<int N>
FxArray<float> parseArray(const YAML::Node &node) {
    FxArray<float> arr(N);
    for (int i = 0; i < N; ++i)
        arr[i] = node[i].as<float>();
    return arr;
}

FxYAML::ConstructScene(const string& filename) {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(filename);
    if (!config) {
        throw std::runtime_error("Failed to load YAML file: " + filename);
    }

    // Read scene properties
    auto scene_size = config["scene_size"];
    if(!scene_size) {
        throw std::runtime_error("Scene size not defined in YAML file.");
    }
    auto scene_size_array = parseArray<2>(scene_size);
    FxScene scene({scene_size_array[0], scene_size_array[1]});

    // Physics configuration
    auto scene_physics = config["physics"];
    if (scene_physics) {
        //.....
    }

    // Read entities
    auto entities = config["entities"];
    if (!entities) {
        throw std::runtime_error("No entities defined in YAML file.");
    } else {
        for( const auto& entity_node : entities) {
           // .....
        }
    }

    // Return the constructed scene
    return scene;
}


int main() {
    // Load YAML file
    YAML::Node config = YAML::LoadFile("Scene.yml");
    if (!config) {
        std::cerr << "Failed to load YAML file." << std::endl;
        return 1;
    }

    // Read scene properties
    auto scene_size = config["scene_size"];
    if(!scene_size) {
        throw std::runtime_error("Scene size not defined in YAML file.");
    }
    auto scene_size_array = parseArray<2>(scene_size)
    FxScene scene({scene_size_array[0], scene_size_array[1]});

    // Physics configuration
    auto scene_physics = config["physics"];
    if (scene_physics) {
        auto max_time_step = scene_physics["max_time_step"];
        if (max_time_step) {    
            scene.set_max_time_step(max_time_step.as<float>());
        }
        auto real_time_factor = scene_physics["real_time_factor"];
        if (real_time_factor) {
            scene.set_real_time_factor(real_time_factor.as<float>());
        }
        auto gravity = scene_physics["gravity"];
        if (gravity) {
            auto gravity_array = parseArray<2>(gravity);
            scene.set_gravity({gravity_array[0], gravity_array[1]});
        }
    }

    // Read entities
    auto entities = config["entities"];
    if (!entities) {
        std::cerr << "No entities defined in YAML file." << std::endl;
    } else {
        for( const auto& entity_node : entities) {
            std::string entity_name = entity_node.first.as<std::string>();
            auto entity_data = entity_node.second;

            // Create a new FxEntity
            auto entity = std::make_shared<FxEntity>(entity_name);
            scene.add_entity(entity);

            // Read initial pose
            auto init_pose = entity_data["init_pose"];
            if (init_pose) {
                auto init_pose_array = parseArray<3>(init_pose);
                entity->set_init_pose({init_pose_array[0], init_pose_array[1], init_pose_array[2]});
            }
            // Read initial velocity
            auto init_velocity = entity_data["init_velocity"];
            if (init_velocity) {
                auto init_velocity_array = parseArray<3>(init_velocity);
                entity->set_init_velocity({init_velocity_array[0], init_velocity_array[1], init_velocity_array[2]});
            }

            // Read physics properties
            auto physics = entity_data["physics"];
            if (physics) {
                auto mass = physics["mass"];
                if (mass) {
                    entity->set_mass(mass.as<float>());
                }
                auto inertia = physics["inertia"];
                if (inertia) {
                    entity->set_inertia(inertia.as<float>());
                }
                auto gravity_scale = physics["gravity_scale"];
                if (gravity_scale) {
                    entity->gravity_scale = gravity_scale.as<float>();
                }
                auto vel_damping = physics["vel_damping"];
                if (vel_damping) {
                    entity->velocity_damping = vel_damping.as<float>();
                }
                auto elasticity = physics["elasticity"];
                if (elasticity) {
                    entity->elasticity = elasticity.as<float>();
                }
                auto static_friction = physics["static_friction"];
                if (static_friction) {
                    entity->static_friction = static_friction.as<float>();
                }
                auto dynamic_friction = physics["dynamic_friction"];
                if (dynamic_friction) {
                    entity->dynamic_friction = dynamic_friction.as<float>();
                }
                auto external_forces_enabled = physics["external_forces_enabled"];
                if (external_forces_enabled) {
                    entity->external_forces_enabled = external_forces_enabled.as<bool>();
                } 

                // Read visual properties
                auto visual = entity_data["visual"];
                if (visual) {
                    auto visual_geometry = visual["geometry"];
                    if (visual_geometry) {
                        auto shape_type = visual_geometry["shape_type"].as<std::string>();
                        if (shape_type == "circle") {
                            float radius = visual_geometry["radius"].as<float>();
                            entity->set_visual_geometry(FxShape(radius));
                        } else if (shape_type == "rectangle") {
                            auto size = parseArray<2>(visual_geometry["size"]);
                            entity->set_visual_geometry(FxShape(size[0], size[1]));
                        } else if (shape_type == "polygon") {
                            // Handle polygon shape
                        }
                    }
                    auto texture = visual["texture"];
                    if (texture) {
                        if(texture.IsScalar()) {
                            // If texture is a string, set it directly
                            entity->visual_geometry()->set_fillTexture(texture.as<std::string>());
                        } else if (texture.IsSequence()) {
                            // If texture is a sequence, assume it's a list of strings
                            for (const auto& tex : texture) {
                                entity->visual_geometry()->add_fillTexture(tex.as<std::string>());
                            }
                        }
                        entity->visual_geometry()->set_fillTexture(texture.as<std::string>());
                    }
                    // Set visual pose
                    auto visual_pose = visual["pose"];
                    if (visual_pose) {
                        auto visual_pose_array = parseArray<3>(visual_pose);
                        entity->visual_geometry()->set_offset_pose({visual_pose_array[0], visual_pose_array[1], visual_pose_array[2]});
                    }

                    //set border color
                    auto border_color = visual["border_color"];
                    if (border_color) {
                        auto border_color_array = parseArray<4>(border_color);
                        entity->visual_geometry()->set_outlineColor(
                            sf::Color(border_color_array[0], border_color_array[1], border_color_array[2], border_color_array[3])
                        );
                    }
                }
            }
        }




    }



    return 0;
}
