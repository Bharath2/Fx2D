#pragma once 

#include <string.h>
#include <variant>
#include <regex>
#include <stdexcept>
#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#define USE_SFML 1

#if defined(USE_SFML) && (USE_SFML != 0)
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Texture.hpp>
#endif

#include "utils.h"

// Shape Definitons
struct FxCircleDef{
    float radius = 1.0f;
    FxCircleDef() = default;
    FxCircleDef(float radius) : radius(radius) {}
    const std::string get_type() const { return "Circle"; }
};
struct FxRectangleDef{
    FxVec2f size {1.0f, 1.0f};
    FxRectangleDef() = default;
    FxRectangleDef(FxVec2f size) : size(size) {}
    const std::string get_type() const { return "Rectangle"; }
};
struct FxPolygonDef{
    std::vector<FxVec2f> vertices{{1.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}}; 
    FxPolygonDef() = default; 
    FxPolygonDef(std::vector<FxVec2f> vertices) : vertices(vertices) {} 
    const std::string get_type() const { return "Polygon"; }
};

// defining a commmon type for all shapes 
typedef std::variant<FxCircleDef, FxRectangleDef, FxPolygonDef> FxShapeDef;

#if defined(USE_SFML) && (USE_SFML != 0)
// container for visual shape 
struct FxVisualShape{
    private:    
        sf::Color m_outlineColor {10, 10, 10, 255};
        sf::Color m_fillColor {200, 200, 200, 255};
        std::shared_ptr<sf::Texture> m_fillTexture;

    public:
        // Shape definition
        FxShapeDef def = FxCircleDef(1.0f);

        // Method to set a fillColor
        void set_fillColor(const sf::Color& color) {
            m_fillColor = color;
            m_fillTexture.reset();
        }

        // Method to set a fillColor
        void set_outlineColor(const sf::Color& color) {
            m_outlineColor = color;
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
        std::shared_ptr<const sf::Texture> fillTexture() const {return m_fillTexture;}
};
#else
// No need of visual shape when there is no SFML
struct FxVisualShape {
    // Prevents stack allocation
    FxVisualShape() = delete; 
    // Prevents heap allocation
    static void* operator new(std::size_t) noexcept { return nullptr; } 
};
#endif

// container for collision shape 
struct FxCollisionShape{
    FxShapeDef def = FxCircleDef(1.0f);
};


// Class for entity attributes and methods
class FxEntity{
private:
    // store initial state for reset
    FxVec2f init_pos {0, 0};
    FxVec2f init_vel {0, 0};
    float init_theta = 0;
    float init_omega = 0;

    std::shared_ptr<FxVisualShape> m_visual = std::make_shared<FxVisualShape>();
    std::shared_ptr<FxCollisionShape> m_collision;
    
public:
    // unique name for an entity
    std::string name;

    // mass and inertia
    float mass = 1.0f;     
    float inertia = 1.0f;   // around center of mass

    // current state
    FxVec2f pos {0, 0};       // x, z coordinates
    FxVec2f vel {0, 0};       // velocity along x, z axis 
    float theta = 0;     // orientation in xz plane
    float omega = 0;     // angular velocity
    
    // physics config
    float gravity_scale = 1.0f;
    float vel_damping = 0.0f;
    float elasticity = 1.0f;
    float surface_friction = 0.0f;
    bool ext_forces = false;

    // contructor with name validation
    explicit FxEntity(const std::string& entityName);
    ~FxEntity(){}

    // resets current state to inital state
    void reset();

    // methods to set initial pose and velocity
    void set_init_pose(const FxVec3f& pose);
    void set_init_velocity(const FxVec3f& velocity);
    
    // clear existing visual and collison shapes and assign new
    void set_visual(FxVisualShape& visual){
        m_visual = std::make_shared<FxVisualShape>(std::move(visual));
    }

    void set_collision(FxCollisionShape& collision){
        m_collision = std::make_shared<FxCollisionShape>(std::move(collision));
    }

    //getters for visual and collison shapes
    std::shared_ptr<const FxVisualShape> visual() const {return m_visual;}
    std::shared_ptr<const FxCollisionShape> collision() const {return m_collision;}

    // methods to delete visual and collision shapes
    void delete_visual(){ m_visual.reset(); };
    void delete_collision(){ m_collision.reset(); };
};


// Scene class takes care of entities motion and collisions
class FxScene
{
private:
    FxVec2ui m_size; // scene size [x, z] units
    std::vector<std::shared_ptr<FxEntity>> m_entities; // stores all entities
    std::unordered_map<std::string, size_t> m_entityMap; // maps entity.name to index in m_entities
    
public:
    // physics config
    FxVec2f gravity {0.0f, -10.0f}; 
    float max_time_step = 0.001f;
    float real_time_factor = 1.0f;   // not guaranteed to follow 

    // constructor, destructor 
    FxScene(FxVec2ui SceneSize) : m_size(SceneSize) {};
    ~FxScene() {};

    // calls reset of all entities
    void reset();
    // simulation step
    void step(float dt);
    // Returns true if added; false if an entity with the name already exists.
    bool add_entity(std::shared_ptr<FxEntity> entity);
    // Returns true if deletion succeeded, false if the entity wasn't found.
    bool delete_entity(const std::string& name);
    // Returns the entity pointer if found; otherwise returns nullptr.
    std::shared_ptr<FxEntity> get_entity(const std::string& name) const;

    // for_each_entity applies the given function on each entity in a given execution mode
    template <typename ExecPolicy, typename Func>
    void for_each_entity(ExecPolicy&& policy, Func&& func) {
        std::for_each(std::forward<ExecPolicy>(policy),
                      m_entities.begin(),
                      m_entities.end(),
                      std::forward<Func>(func));
    }

    // transform_entities collects return values in a vector
    template <typename ExecPolicy, typename Func>
    void transform_entities(ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<FxEntity>>>& results){
        results.resize(m_entities.size());
        std::transform(std::forward<ExecPolicy>(policy),
                    m_entities.begin(),
                    m_entities.end(),
                    results.begin(),
                    std::forward<Func>(func));
    }
};


// #include <vector>
// #include <memory>
// #include <unordered_map>

// // Assuming you have your FxEntity class with getBoundingBox()

// class FxScene {
// private:
//     std::vector<std::shared_ptr<FxEntity>> m_entities_vector; // For fast iteration
//     std::unordered_map<std::string, std::shared_ptr<FxEntity>> m_entities_map; // For fast name lookup

//     // Uniform Grid
//     struct GridCell {
//         std::vector<std::shared_ptr<FxEntity>> entities; // Store shared_ptr to entities
//     };

//     std::vector<GridCell> m_grid; // 1D vector to represent a 2D grid
//     int m_grid_width;
//     int m_grid_height;
//     float m_cell_size;
//     Vec2f m_grid_origin;

//     // Helper to get grid cell index from world coordinates
//     int get_grid_index(const Vec2f& pos) const {
//         int x = static_cast<int>((pos.x - m_grid_origin.x) / m_cell_size);
//         int y = static_cast<int>((pos.y - m_grid_origin.y) / m_cell_size);
//         return y * m_grid_width + x;
//     }

// public:
//     // ... other members ...

//     // Constructor to set up the grid
//     FxScene(float world_width, float world_height, float cell_size)
//         : m_grid_width(static_cast<int>(world_width / cell_size)),
//           m_grid_height(static_cast<int>(world_height / cell_size)),
//           m_cell_size(cell_size),
//           m_grid_origin({0.0f, 0.0f}) { // Assuming origin is at (0,0)
//         m_grid.resize(m_grid_width * m_grid_height);
//     }

//     void add_entity(std::shared_ptr<FxEntity> entity) {
//         if (!entity) return;

//         // Add to main entity list and map
//         std::string entity_name = entity->getName();
//         m_entities_vector.push_back(entity);
//         m_entities_map[entity_name] = entity;

//         // Add to the grid cells it overlaps with
//         AABB aabb = entity->getBoundingBox();
//         // Calculate overlapping cells and add entity to their lists
//         // This is a simplified example, you'd need to iterate through all overlapping cells
//         int cell_index = get_grid_index(aabb.min);
//         if (cell_index >= 0 && cell_index getBoundingBox();
//                     AABB b = cell.entities[j]->getBoundingBox();

//                     if (is_overlapping(a, b)) {
//                         // Collision detected between cell.entities[i] and cell.entities[j]
//                         // Handle the collision
//                     }
//                 }
//             }
//         }
//     }
// };