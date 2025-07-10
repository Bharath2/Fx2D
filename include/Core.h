#pragma once 

#define USE_SFML 1

#include <string.h>
#include <variant>
#include <regex>
#include <stdexcept>
#include <vector>
#include <memory>
#include <functional>
#include <iostream>
#include <Eigen/Dense>

#include "MathTypes.h"


#if defined(USE_SFML) && (USE_SFML != 0)

#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Texture.hpp>


// container for visual shape 
struct FxVisualShape : public FxShape {
    private:    
        sf::Color m_outlineColor {10, 10, 10, 255};
        sf::Color m_fillColor {200, 200, 200, 255};
        float m_outlineThickness = 5.0f;
        std::shared_ptr<sf::Texture> m_fillTexture;

    public:
        // 1. “Inherit” all of FxShape’s ctors
        using FxShape::FxShape;

        // 2. Conversion‐style ctor: build a VisualShape from any Shape
        FxVisualShape(const FxShape& base)
        : FxShape(base) {}

        // Method to set a fillColor
        void set_fillColor(const sf::Color& color) {
            m_fillColor = color;
            m_fillTexture.reset();
        }

        // Method to set a fillColor
        void set_outlineColor(const sf::Color& color) {
            m_outlineColor = color;
        }

        void set_outlineThickness(const unsigned int& thickness){
            m_outlineThickness = thickness;
        }

        // Method to set a texture
        void set_fillTexture(const std::string& filePath) {
            if (!m_fillTexture)
                m_fillTexture = std::make_shared<sf::Texture>();
            if (!m_fillTexture->loadFromFile(filePath)){
                    std::cerr << "Failed to load texture from file: " << filePath << std::endl;
                    m_fillTexture.reset();
            }
        }

        // getters
        const sf::Color fillColor() const {return m_fillColor;}
        const sf::Color outlineColor() const {return m_outlineColor;}
        float outlineThickness() const {return m_outlineThickness;}
        std::shared_ptr<const sf::Texture> fillTexture() const {return m_fillTexture;}
};

#else

// No need of visual shape when there is no SFML
struct FxVisualShape : public FxShape {
    // Prevents stack allocation
    FxVisualShape() = delete; 
    // Prevents heap allocation
    static void* operator new(std::size_t) noexcept { return nullptr; } 
};

#endif

using FxCollisionShape = FxShape;

struct FxContact
{
    bool is_valid = false; // is this contact valid
    FxVec2f positon {0.0f, 0.0f}; // positon of this conctact in the world frame
    FxVec2f normal {0.0f, 0.0f};  // normal along the contact
    float pentration_depth = 0.0f; // penetration depth along the contact normal
    
    // Constructor overloads
    FxContact() = default;
    FxContact(bool valid)
        : is_valid(valid), positon(0.0f, 0.0f), normal(0.0f, 0.0f), pentration_depth(0.0f) {}
    FxContact(bool valid, const FxVec2f& pos, const FxVec2f& norm, float depth)
        : is_valid(valid), positon(pos), normal(norm), pentration_depth(depth) {}
};


// Class for entity attributes and methods
class FxEntity {
private:
    // mass and inertia
    float _mass = 1.0f;     
    float _inertia = 1.0f;   // around center of mass
    float _inv_mass = 1.0f;
    float _inv_inertia = 1.0f;
    
    // store initial state for reset
    FxVec3f _init_pose {0, 0, 0};
    FxVec3f _init_velocity {0, 0, 0};

    // visual and collision shapes
    std::shared_ptr<FxCollisionShape> m_collision;
    std::shared_ptr<FxVisualShape> m_visual = std::make_shared<FxVisualShape>();

    // total resultant force and moment on the body
    float m_eff_moment = 0.0f;
    FxVec2f m_eff_force {0, 0};

    // axis aligned bounding box in world coordinates
    FxArray<float> m_bounding_box {-1.0f, -1.0f, -1.0f, -1.0f}; 
    
public:
    // unique name for an entity
    const std::string name;

    // current state
    FxVec3f pose {0, 0, 0};    // x, y, theta
    FxVec3f velocity {0, 0, 0};   // velocity along x, y axis and angular velocity along z axis
    
    // physics config
    float elasticity = 1.0f;
    float vel_damping = 0.0f;
    float gravity_scale = 1.0f;
    float static_friction = 0.0f;
    float dynamic_friction = 0.0f;

    // contructor with name validation
    explicit FxEntity(const std::string& entityName);

    // getter for the name
    const std::string& get_name() const {return name;}

    // resets current state to inital state
    void reset();

    // methods to set mass and inertia
    void set_mass(const float& mass);
    void set_inertia(const float& inertia);
    // methods to get mass and inertia
    float mass() const { return _mass; }
    float inertia() const { return _inertia; }
    float inv_mass() const { return _inv_mass; }
    float inv_inertia() const { return _inv_inertia; }

    // methods to set initial pose and velocity
    void set_init_pose(const FxVec3f& o_pose);
    void set_init_velocity(const FxVec3f& o_velocity);
    // Enable or disable external forces and torques, including effects due to collisions
    void enable_external_forces(bool enable);

    // clear existing visual and collison shapes and assign new
    void set_visual_geometry(FxVisualShape visual){
        m_visual = std::make_shared<FxVisualShape>(std::move(visual));
    }
    void set_collision_geometry(FxCollisionShape collision){
        m_collision = std::make_shared<FxCollisionShape>(std::move(collision));
    }
    // getters for visual and collison shapes
    std::shared_ptr<FxVisualShape> visual_geometry() const {return m_visual;}
    std::shared_ptr<FxCollisionShape> collision_geometry() const {return m_collision;}
    // methods to delete visual and collision shapes
    void del_visual_geometry(){ m_visual.reset(); };
    void del_collision_geometry(){ m_collision.reset(); };

    // collision detection methods
    const FxArray<float> bounding_box() const;
    bool box_collision_check(const FxEntity& other) const;
    bool box_collision_check(const std::shared_ptr<FxEntity>& other) const;
    const FxContact sat_collision_check(const FxEntity& other) const;
    const FxContact sat_collision_check(const std::shared_ptr<FxEntity>& other) const;
     
    // apply external influences
    void apply_torque(float torque);
    void apply_force(const FxVec2f& force);
    void apply_force(const FxVec2f& force, const FxVec2f& contact_point);
    void apply_impulse(const FxVec2f& impulse);
    void apply_impulse(const FxVec2f& impulse, const FxVec2f& contact_point);

    // resolve the forces and torques and calculate acceleration;
    FxVec3f calc_acceleration();
    FxVec3f calc_acceleration(const FxVec2f& gravity);
    void step(const FxVec2f& gravity, const float& step_dt);

    ~FxEntity(){}
};


// Scene class takes care of entities motion and collisions
class FxScene {
private:
    static constexpr size_t m_enitities_limit = 4096; // no of entities in the scene can not exceed 4096
    std::vector<std::shared_ptr<FxEntity>> m_entities_vec; // stores pointers to all entities
    std::unordered_map<std::string, size_t> m_entities_map; // maps entity's name to index in the entities vector
    std::function<void(FxScene&, float dt)> m_func_step_callback; //custom callback function invoked in the step method
    
    // max and min time step values that can be use in step method 
    float m_max_time_step = 0.05f;
    float m_real_time_factor = 1.0f; 
    static constexpr float m_min_time_step = 1e-6f;
    
public:
    // scene size [x, y] units
    const FxVec2ui size; 
    // gravity config [x, y]
    FxVec2f gravity {0.0f, -9.81f}; 
    // 

    // constructor, destructor 
    FxScene(FxVec2ui SceneSize) : size(SceneSize) {};
    ~FxScene() {};

    // calls reset of all entities
    void reset();
    // simulation step, returns the real time factor.
    float step(float step_dt); 
    void set_max_time_step(const float& step_dt);
    void set_real_time_factor(const float& rt_factor);
    void set_gravity(const FxVec2f& o_gravity) { gravity = o_gravity; }
    // custom call back function called after every time step, user gets access to the scene.
    void set_step_callback(const std::function<void(FxScene&, float dt)>& callback){
        m_func_step_callback = callback;
    }
    // Returns true if added; false if an entity with the name already exists.
    bool add_entity(std::shared_ptr<FxEntity> entity);
    // Returns true if deletion succeeded, false if the entity wasn't found.
    bool delete_entity(const std::string& name);
    // Returns the entity pointer if found; otherwise returns nullptr.
    std::shared_ptr<FxEntity> get_entity(const std::string& name) const;

    // for_each_entity applies the given function on each entity in a given execution mode
    template <typename ExecPolicy, typename Func>
    void for_each_entity(ExecPolicy&& policy, Func&& func) {
        // copy to a new vector of raw pointers
        std::vector<FxEntity*> raw_entities_vec;
        raw_entities_vec.reserve(m_entities_vec.size());
        for (const auto& entity : m_entities_vec) {
            raw_entities_vec.push_back(entity.get());
        }
        // pass to std::for_each to do the required optimization
        std::for_each(std::forward<ExecPolicy>(policy),
                      raw_entities_vec.begin(),
                      raw_entities_vec.end(),
                      std::forward<Func>(func));
    }

    // transform_entities collects return values vector in a given execution mode
    template <typename ExecPolicy, typename Func>
    void transform_entities(ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<FxEntity>>>& results){
        // set results vector size
        results.resize(m_entities_vec.size()); 
        // copy to a new vector of raw pointers
        std::vector<FxEntity*> raw_entities_vec;
        raw_entities_vec.reserve(m_entities_vec.size());
        for (const auto& entity : m_entities_vec) {
            raw_entities_vec.push_back(entity.get());
        }
        // pass to std::tranform to do the required optimization
        std::transform(std::forward<ExecPolicy>(policy),
                    raw_entities_vec.begin(),
                    raw_entities_vec.end(),
                    results.begin(),
                    std::forward<Func>(func));
    }
};
