#include "physics.h"

#include <execution>
#include <algorithm>


// FxEntity contructor with name validation
FxEntity::FxEntity(const std::string &entityName) : name(entityName) {
    static const std::regex pattern{"^[A-Za-z0-9_]+$"};  // alphanumerics charaters or underscores are only allowed
    if (!std::regex_match(name, pattern)) {
        throw std::invalid_argument("Entity name must be alphanumeric.");
    }
}

// resets current state to inital state
void FxEntity::reset() {
    pos = init_pos;
    vel = init_vel;
    theta = init_theta;
    omega = init_omega;
}

// method to set initial pose
void FxEntity::set_init_pose(const FxVec3f& pose) {
    init_pos = {pose.x(), pose.y()};
    init_theta = pose.z();
    pos = init_pos;
    theta = init_theta;
}

// method to set initial velocity
void FxEntity::set_init_velocity(const FxVec3f& velocity) {
    init_vel = {velocity.x(), velocity.y()};
    init_omega = velocity.z();
}

// calls reset of all entities
void FxScene::reset() {
    // Use for_each_entity with the sequential execution policy
    for_each_entity(std::execution::seq, [](auto entity) {
        entity->reset(); // Apply reset() to each entity
    });
}

// simulation step
void FxScene::step(float dt) {
    // Use for_each_entity with the parallel execution policy
    for_each_entity(std::execution::par, [&](auto entity) { // Capture 'dt' and 'm_size' by reference
        // Update position based on velocity and delta time
        entity->pos += entity->vel * dt;

        // Apply gravity if gravity_scale is non-zero
        if (entity->gravity_scale != 0) {
            entity->vel += entity->gravity_scale * gravity * dt;
        }

        // Boundary collision response (simple reflection)
        if (entity->pos.x() >= m_size.x()) {
            entity->vel.set_x(entity->vel.x() * -1);
        }

        if (entity->pos.y() >= m_size.y()) {
            entity->vel.set_y(entity->vel.y() * -1);
        }
    });
}

// Returns true if added; false if an entity with the name already exists.
bool FxScene::add_entity(std::shared_ptr<FxEntity> entity) {
    if (m_entityMap.find(entity->name) != m_entityMap.end()) {
        std::cerr << "Entity with name '" << entity->name << "' already exists." << std::endl;
        return false;
    }
    m_entities.push_back(entity);
    m_entityMap[entity->name] = m_entities.size() - 1;
    return true;
}

// Returns true if deletion succeeded, false if the entity wasn't found.
bool FxScene::delete_entity(const std::string& name) {
    auto it = m_entityMap.find(name);
    if (it == m_entityMap.end()) {
        std::cerr << "Entity with name '" << name << "' not found." << std::endl;
        return false;
    }
    // swap the last element with element to be deleted;
    size_t idx = it->second;
    size_t last_idx = m_entities.size() - 1;
    if (idx != last_idx) {
        m_entities[idx] = m_entities[last_idx];
        m_entityMap[m_entities[last_idx]->name] = idx;
    }
    // Remove the last element from the vector.
    m_entities.pop_back();
    // erase from map
    m_entityMap.erase(it);
    return true;
}

// Returns the entity pointer if found; otherwise returns nullptr.
std::shared_ptr<FxEntity> FxScene::get_entity(const std::string& name) const {
    auto it = m_entityMap.find(name);
    if (it == m_entityMap.end()) 
        return nullptr;
    size_t idx = it->second;
    return m_entities[idx];
}

