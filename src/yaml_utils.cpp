#include "YamlUtils.h"
#include <vector>


YAML::Node FxYAML::LoadSmart(const std::string& textOrPath) {
    try {
        // Treat as file if it exists
        if (fs::exists(textOrPath) && fs::is_regular_file(textOrPath))
            return YAML::LoadFile(textOrPath);
        // Otherwise parse the string as YAML
        auto text_ = deIndent(textOrPath);
        return YAML::Load(text_);
    }
    catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML parse error: " + std::string(e.what()));
    }
    catch (const std::exception& e) {
        throw std::runtime_error("YAML load error: " + std::string(e.what()));
    }
}

// Remove the indentation (leading spaces/tabs) from each line of a multi-line string.
std::string FxYAML::deIndent(const std::string& raw) {
    // 1) Find the first non-blank line start
    size_t firstLineStart = raw.find_first_not_of("\r\n");
    if (firstLineStart == std::string::npos) 
        return "";  // empty or all-newlines
    // 2) From there, measure its indent
    size_t firstContent = raw.find_first_not_of(" \t", firstLineStart);
    size_t indent = (firstContent == std::string::npos 
                    ? raw.size() - firstLineStart 
                    : firstContent - firstLineStart);
    // 3) Strip that indent from each line
    std::istringstream in(raw);
    std::ostringstream out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.size() > indent) out << line.substr(indent);
        else out << line;
        out << '\n';
    }
    return out.str();
}


FxShape FxYAML::buildShape(const YAML::Node& config) {
    if (!config.IsMap()) {
        throw std::runtime_error("Expected a map for shape configuration.");
    }

    // 0) Read optional pose (default {0,0,0})
    std::array<float,3> pose_offset = {0.f, 0.f, 0.f};
    if (auto p = config["pose"]) {
        if (!p.IsSequence() || p.size() != 3)
            throw std::runtime_error("pose must be a 3-sequence [x,y,theta].");
        pose_offset = { p[0].as<float>(),
                        p[1].as<float>(),
                        p[2].as<float>() };
    }

    auto geom = config["geometry"];
    if (!geom || !geom.IsMap()) {
        throw std::runtime_error("Expected geometry:{…} block.");
    }
    FxShape shape;
    // 1) Circle?
    if (auto node = geom["circle"]) {
        float radius = node.as<float>();
        shape = FxShape(radius);
    }
    // 2) Rectangle?
    else if (auto node = geom["rectangle"]) {
        if (!node.IsSequence() || node.size() != 2)
            throw std::runtime_error("rectangle must be a 2-sequence.");
        auto sz = parseArray<2>(node);
        shape = FxShape({ sz[0], sz[1] });
    }
    // 3) Polygon?
    else if (auto node = geom["polygon"]) {
        if (!node.IsSequence())
            throw std::runtime_error("polygon must be a sequence.");
        std::vector<FxVec2f> verts;
        for (const auto &pt : node) {
            if (!pt.IsSequence() || pt.size() != 2)
                throw std::runtime_error("each polygon vertex must be a 2-sequence.");
            auto pt_ = parseArray<2>(pt);
            verts.push_back(FxVec2f({pt_[0], pt_[1]}));
        }
        shape = FxShape(verts);
    }
    else { throw std::runtime_error("Unknown geometry type in shape config."); }
    // 4) Set the offset pose
    shape.set_offset_pose({pose_offset[0], pose_offset[1], pose_offset[2]});
    return shape;
}


FxVisualShape FxYAML::buildVisualShape(const YAML::Node& config) {
    FxVisualShape shape = FxVisualShape(buildShape(config));

    // Read border color
    if (auto border = config["border_color"]) {
        if (!border.IsSequence() || border.size() != 4) {
            throw std::runtime_error("Border color must be a 4-sequence [R,G,B,A].");
        }
        auto color_array = (parseArray<4>(border)).as<uint8_t>();
        auto border_color = sf::Color(color_array[0], color_array[1], color_array[2], color_array[3]);
        shape.set_outlineColor(border_color);
    }

    if (auto tex = config["texture"]) {
        // 1) sequence of 4 numbers -> RGBA color
        if (tex.IsSequence() && tex.size() == 4
            && tex[0].IsScalar() && tex[1].IsScalar()
            && tex[2].IsScalar() && tex[3].IsScalar()) 
        {
            auto c = (parseArray<4>(tex)).as<uint8_t>();
            sf::Color col(c[0], c[1], c[2], c[3]);
            shape.set_fillColor(col);
        }
        // 2) single string -> one texture path
        else if (tex.IsScalar()) {
            shape.set_fillTexture(tex.as<std::string>());
        }
        else {
            throw std::runtime_error("Texture field must be a 4-sequence of numbers or string(s).");
        }
    }
    return shape;
}

std::shared_ptr<FxEntity> FxYAML::buildEntity(const std::string& entity_name, const YAML::Node& config) {
    // Check if the node is a map
    if (!config.IsMap()) {
        throw std::runtime_error("Expected a map for entity configuration.");
    }

    auto entity = std::make_shared<FxEntity>(entity_name);

    // Read initial pose
    auto init_pose = config["pose"];
    if (init_pose) {
        auto init_pose_array = parseArray<3>(init_pose);
        entity->set_init_pose({init_pose_array[0], init_pose_array[1], init_pose_array[2]});
    }

    // Read initial velocity
    auto init_velocity = config["init_velocity"];
    if (init_velocity) {
        auto init_velocity_array = parseArray<3>(init_velocity);
        entity->set_init_velocity({init_velocity_array[0], init_velocity_array[1], init_velocity_array[2]});
    }

    // Read physics properties
    auto physics = config["physics"];
    if (physics) {
        entity->set_mass(physics["mass"].as<float>(1.0f)); // default to 1.0 if not specified
        entity->set_inertia(physics["inertia"].as<float>(1.0f)); // default to 1.0 if not specified
        entity->gravity_scale = physics["gravity_scale"].as<float>(1.0f); // default to 1.0 if not specified
        entity->vel_damping = physics["vel_damping"].as<float>(0.0f); // default to 0.0 if not specified
        entity->elasticity = physics["elasticity"].as<float>(1.0f); // default to 1.0 if not specified
        entity->static_friction = physics["static_friction"].as<float>(0.0f); // default to 0.0 if not specified
        entity->dynamic_friction = physics["dynamic_friction"].as<float>(0.0f); // default to 0.2 if not specified
        entity->enable_external_forces(physics["external_forces_enabled"].as<bool>(true)); // default to true if not specified
    }

    // Read visual properties
    auto visual = config["visual"];
    if (visual) {
        FxVisualShape visual_shape = buildVisualShape(visual);
        entity->set_visual_geometry(visual_shape);
    }
    // Read collision properties
    auto collision = config["collision"];
    if (collision) {
        FxCollisionShape collision_shape = buildShape(collision);
        entity->set_collision_geometry(collision_shape);
    }
    return entity;
}

// Implementation of buildScene
FxScene FxYAML::buildScene(const YAML::Node& config) {
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
        scene.set_max_time_step(scene_physics["max_time_step"].as<float>(0.05f)); // default to 0.1 if not specified
        scene.set_real_time_factor(scene_physics["real_time_factor"].as<float>(1.0f)); // default to 1.0 if not specified
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
        for( const auto& kv : entities) {
            const std::string entity_name = kv.first.as<std::string>();
            const YAML::Node entity_node = kv.second;
            auto entity = FxYAML::buildEntity(entity_name, entity_node);
            scene.add_entity(entity);
        }
    }
    // Return the constructed scene
    return scene;
}

