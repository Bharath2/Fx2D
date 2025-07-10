#include "Core.h"

#include <execution>
#include <algorithm>


// FxEntity contructor with name validation
FxEntity::FxEntity(const std::string &entityName) : name(entityName) {
    static const std::regex pattern{"^[A-Za-z0-9_]+$"};  // alphanumerics charaters or underscores are only allowed
    if (!std::regex_match(name, pattern)) {
        throw std::invalid_argument("FxEntity: Entity name must be alphanumeric.");
    }
}

// Setter for mass: Enforces inertia >= 0.
void FxEntity::set_mass(const float& mass) {
    if (mass < 0.0f ) {
        throw std::invalid_argument("FxEntity: Mass must be non-negative");
    } else if (mass > 1e5f) {
       throw std::invalid_argument("FxEntity: Mass can not be greater than 1e5");
    }
    _mass = mass;
    _inv_mass = (mass == 0.0f) ? 0.0f : 1.0f / mass;
}

// Setter for inertia: Enforces inertia >= 0.
void FxEntity::set_inertia(const float& inertia) {
    if (inertia < 0.0f) {
        throw std::invalid_argument("FxEntity: Inertia must be non-negative");
    }
    _inertia = inertia;
    _inv_inertia = (inertia == 0.0f) ? 0.0f : 1.0f / inertia;
}

// resets current state to inital state
void FxEntity::reset() {
    pose = _init_pose;
    velocity = _init_velocity;
    m_eff_force = {0.0f, 0.0f};
    m_eff_moment = 0.0f;
}

// method to set initial pose
void FxEntity::set_init_pose(const FxVec3f& o_pose) {
    _init_pose = o_pose;
    pose = o_pose;
}

// method to set initial velocity
void FxEntity::set_init_velocity(const FxVec3f& o_velocity) {
    _init_velocity = o_velocity;
    velocity = o_velocity;
}

// Enable or disable external forces and torques, including effects due to collisions
void FxEntity::enable_external_forces(bool enable) {
    if (!enable) {
        _inv_mass = 0.0f;  // Disable external forces by setting inverse mass to zero
        _inv_inertia = 0.0f; // Disable external torques by setting inverse inertia to zero
    } else {
        _inv_mass = (_mass > 0.0f) ? 1.0f / _mass : 0.0f; // Recalculate inverse mass
        _inv_inertia = (_inertia > 0.0f) ? 1.0f / _inertia : 0.0f; // Recalculate inverse inertia
    }
}

// Apply force at center of mass, affecting linear acceleration
void FxEntity::apply_force(const FxVec2f& force) {
    if (_inv_mass > 0.0f) {
        m_eff_force += force; 
    }
}

// Apply force at an arbitrary point, contributes linear and angular effects
void FxEntity::apply_force(const FxVec2f& force, const FxVec2f& contact_point) {
    if (_inv_mass > 0.0f) {
        m_eff_force += force;
    }
    if (_inv_inertia > 0.0f) {
        FxVec2f r = contact_point - pose.xy(); // r is the vector from center of mass to contact point
        float torque = r.x() * force.y() - r.y() * force.x();
        m_eff_moment += torque;
    }
}

// Directly apply moment
void FxEntity::apply_torque(float torque) {
    if (_inv_inertia > 0.0f) {
        m_eff_moment += torque;
    }
}

// For impulse applied at center of mass, the change in velocity is impulse divided by mass
void FxEntity::apply_impulse(const FxVec2f& impulse) {
    if (_inv_mass > 0.0f) {
        velocity.xy() += impulse * _inv_mass;   // apply to linear velocities
    }
}

// Apply impulse at an arbitrary point, contributes linear and angular effects
void FxEntity::apply_impulse(const FxVec2f& impulse, const FxVec2f& contact_point) {
    if (_inv_mass > 0.0f) {
        velocity.xy() += impulse * _inv_mass; // apply to linear velocities
    }
    if (_inv_inertia > 0.0f) {
        FxVec2f r = contact_point - pose.xy(); // r is the vector from center of mass to contact point
        float torque = r.x() * impulse.y() - r.y() * impulse.x();
        velocity.theta() += torque * _inv_inertia; // apply to angular velocity
    }
}

// Calculte the effect of all forces and moments with no gravity
FxVec3f FxEntity::calc_acceleration() {
    return calc_acceleration({0.0f, 0.0f});  // no gravity
}

// Calculte the effect of all forces and moments
FxVec3f FxEntity::calc_acceleration(const FxVec2f& gravity) {
    FxVec3f acc{0.0f, 0.0f, 0.0f};
    if (_mass > 0.0f) {
        acc.xy() += m_eff_force * _inv_mass;  // Linear acceleration
        acc.xy() += gravity_scale * gravity;   // Gravity effect
    }
    if (_inertia > 0.0f) {
        acc.theta() += m_eff_moment * _inv_inertia;  // Angular acceleration
    }
    return acc;
}

// Returns the axis aligned bounding box in world coordinates
const FxArray<float> FxEntity::bounding_box() const {
    return m_bounding_box; 
}

// Broad phase check: if two entities axis aligned bounding boxes overlap  
bool FxEntity::box_collision_check(const FxEntity& other) const {
    auto aabb1 =       bounding_box();
    auto aabb2 = other.bounding_box();
    // check if they are overlapping
    return !(aabb1(2) < aabb2(0) || aabb2(2) < aabb1(0) || // this.maxX < other.minX or other.maxX < this.minX
             aabb1(3) < aabb2(1) || aabb2(3) < aabb1(1));  // this.maxY < other.minY or  other.maxY < this.minY
}

// Comprehensive check with separaring axis theorem, 
const FxContact FxEntity::sat_collision_check(const FxEntity& other) const {
    // Returns contact normal and penetration depth  
    auto contact = FxContact(false);
    // Check if bounding boxes overlap first
    if (!box_collision_check(other)) return contact; 
    // If bounding boxes overlap, perform SAT collision check
    auto o_collision = other.collision_geometry();
     // the shapes are considered to be intersecting if they are not separated along any axis.
    contact.is_valid = true;
    // Check for collision using SAT
    for (const auto& axis : m_collision->axes()) {
        // Project both shapes onto the axis
        auto proj1 = m_collision->project(axis);
        auto proj2 = o_collision->project(axis);
        // Check for overlap
        if (proj1.max < proj2.min || proj2.max < proj1.min) {
            contact.is_valid = false; // No collision if projections do not overlap
            return contact;
        }
        // Calculate the overlap depth
        float depth = std::min(proj1.max, proj2.max) - std::max(proj1.min, proj2.min);
        if (contact.pentration_depth == 0.0f || depth < contact.pentration_depth) {
            contact.pentration_depth = depth;
            contact.normal = axis; // Store the normal of the separating axis
        }
    }
    for(const auto& axis : o_collision->axes()) {
        // Project both shapes onto the axis
        auto proj1 = m_collision->project(axis);
        auto proj2 = o_collision->project(axis);
        // Check for overlap
        if (proj1.max < proj2.min || proj2.max < proj1.min) {
            contact.is_valid = false; // No collision if projections do not overlap
            return contact;
        }
        // Calculate the overlap depth
        float depth = std::min(proj1.max, proj2.max) - std::max(proj1.min, proj2.min);
        if (contact.pentration_depth == 0.0f || depth < contact.pentration_depth) {
            contact.pentration_depth = depth;
            contact.normal = axis; // Store the normal of the separating axis
        }
    }
    return contact;
}

// Overload of aabb_collision() that accepts a shared pointer.
bool FxEntity::box_collision_check(const std::shared_ptr<FxEntity>& other) const {
    return box_collision_check(*other);
}

// Overload of sat_collision() that accepts a shared pointer.
const FxContact FxEntity::sat_collision_check(const std::shared_ptr<FxEntity>& other) const {
    return sat_collision_check(*other);
}

void FxEntity::step(const FxVec2f& gravity, const float& step_dt){
    // verlet integration
    velocity += 0.5f * calc_acceleration(gravity) * step_dt;
    pose += velocity * step_dt;
    velocity += 0.5f * calc_acceleration(gravity) * step_dt;


    if(m_collision != nullptr) {
        m_collision->set_world_pose(pose); // update offset pose of the collision shape
        const std::string& type = m_collision->shape_type();  // Handle each shape type differently
        if (type == "Circle") {
            float radius = m_collision->radius();
            auto offset = m_collision->offset_pose();
            float pX = pose.x() + offset.x();
            float pY = pose.y() + offset.y();
            m_bounding_box = {pX - radius, pY - radius, pX + radius, pY + radius}; // AABB for circle
        } else {
            auto vertices = m_collision->vertices();
            auto offset = m_collision->offset_pose();
            vertices.rotate_inplace(pose.theta() + offset.theta()); // rotate vertices by pose theta
            vertices += offset.get_xy(); // translate vertices by offset pose xy
            vertices += pose.get_xy();
            m_bounding_box = vertices.bounds();
        }
    }
    m_eff_force = {0.0f, 0.0f}; // reset effective force
    m_eff_moment = 0.0f; // reset effective moment
}


//---------------- FxScene Methods ----------------------
// calls reset of all entities
void FxScene::reset() {
    // Use for_each_entity with the sequential execution policy
    for_each_entity(std::execution::par, [](auto entity) {
        entity->reset(); // Apply reset() to each entity
    });
}

// set the maximum limit for time step
void FxScene::set_max_time_step(const float& step_dt){
    if (step_dt < 2e-6f){
        throw std::invalid_argument("FxScene: max time step (dt) must be greater than 2e-6");
    } else if (step_dt > 0.1f){
        throw std::invalid_argument("FxScene: max time step (dt) must be less than or equal to 0.1");
    }
    m_max_time_step = step_dt;
}

// set real time factor, slow down or speed up simulation
void FxScene::set_real_time_factor(const float& rt_factor){
    if (rt_factor < 0.001f){
        std::cerr << "FxScene: real time factor must be greater than 0.001" << std::endl;
        m_real_time_factor = 0.001f;
    } else {
        m_real_time_factor = rt_factor;
    }
}

// Returns true if added; false if an entity with the name already exists.
bool FxScene::add_entity(std::shared_ptr<FxEntity> entity) {
    if (entity.get() == nullptr) {
        std::cerr << "FxScene: Cannot add a null entity." << std::endl;
        return false;
    }
    if(m_entities_vec.size() < m_enitities_limit){
        if (m_entities_map.find(entity->get_name()) != m_entities_map.end()) {
            std::cerr << "FxScene: Entity with name '" << entity->get_name() << "' already exists." << std::endl;
            return false;
        }
        m_entities_vec.push_back(entity);
        m_entities_map[entity->get_name()] = m_entities_vec.size() - 1;
        return true;
    } else {
        std::cerr << "FxScene: Entities limit exceeded." << std::endl;
        return false;
    }
}

// Returns true if deletion succeeded, false if the entity wasn't found.
bool FxScene::delete_entity(const std::string& name) {
    auto it = m_entities_map.find(name);
    if (it == m_entities_map.end()) {
        std::cerr << "FxScene: Entity with name '" << name << "' not found." << std::endl;
        return false;
    }
    // swap the last element with element to be deleted;
    size_t idx = it->second;
    size_t last_idx = m_entities_vec.size() - 1;
    if (idx != last_idx) {
        m_entities_vec[idx] = m_entities_vec[last_idx];
        m_entities_map[m_entities_vec[last_idx]->get_name()] = idx;
    }
    // Remove the last element from the vector.
    m_entities_vec.pop_back();
    // erase from map
    m_entities_map.erase(it);
    return true;
}

// Returns the entity pointer if found; otherwise returns nullptr.
std::shared_ptr<FxEntity> FxScene::get_entity(const std::string& name) const {
    auto it = m_entities_map.find(name);
    if (it == m_entities_map.end()) 
        return nullptr;
    size_t idx = it->second;
    return m_entities_vec[idx];
}


// simulation step
float FxScene::step(float step_dt) {
    // Throw an error if dt is negative
    if (step_dt < 1e-6f) {
        throw std::invalid_argument("FxScene: dt (delta time) must be non-negative");
    }
    float dt = std::clamp(step_dt*m_real_time_factor, m_min_time_step, m_max_time_step);

    // Use for_each_entity with the parallel execution policy
    for_each_entity(std::execution::par, [&](auto entity) { // Capture 'dt' and 'm_size' by reference
        // Apply gravity and step the entity
        entity->step(gravity, dt);
        // Boundary collision response (simple reflection)
        if (entity->pose.x() >= size.x()) {
            if(entity->velocity.x() > 0.0f) {
                entity->velocity.x() *= -1;
            }
        }
        if (entity->pose.y() >= size.y()) {
            if(entity->velocity.y() > 0.0f) {
                entity->velocity.y() *= -1;
            }
        }
        if (entity->pose.x() <= 0.0) {
            if(entity->velocity.x() < 0.0f) {
                entity->velocity.x() *= -1;
            }
        }
        if (entity->pose.y() <= 0.0) {
            if(entity->velocity.y() < 0.0f) {
                entity->velocity.y() *= -1;
            }
        }
    });

    // check and resolve collison
    for(size_t idx1 = 0; idx1 < m_entities_vec.size(); ++idx1){
        auto body1 = m_entities_vec[idx1];
        for(size_t idx2 = idx1 + 1; idx2 < m_entities_vec.size(); ++idx2){
            auto body2 = m_entities_vec[idx2];
            auto contact = body1->sat_collision_check(body2);
            if(contact.is_valid) {
                // 1) build normal and relative velocity
                FxVec2f norm = (body2->pose.xy() - body1->pose.xy()).normalized();
                FxVec2f relVel = body2->velocity.xy() - body1->velocity.xy();
                float sepVel = relVel.dot(norm);
                // 2) if the bodies are separating, do nothing
                if (sepVel < 0.0f) {
                    // 3) if the bodies are colliding, apply impulse
                    float e = std::min(body1->elasticity, body2->elasticity);
                    float invSum = body1->inv_mass() + body2->inv_mass();
                    if (invSum > 0.0f) {
                        float j = -(1.0f + e) * sepVel / invSum;
                        FxVec2f Pn = norm * j;
                        body1->apply_impulse(-Pn);
                        body2->apply_impulse( Pn);pmlj
                    }
                }

                FxVec2f tangent = relVel - (relVel.dot(norm) * norm);
                std::cout<<tangent.norm()<<std::endl;
                

                // apply friction where necessary
                // float surface_friction = std::min(body1->surface_friction, body2->surface_friction);
                // if(surface_friction > 0.0f) {
                //     // Calculate the tangent vector
                //     FxVec2f tangent = relVel - (relVel.dot(norm) * norm);
                    // Normalize the tangent vector
                    // if (tangent.length() > 1e-6f) {
                    //     tangent.normalize();
                    //     // relative velocity along tangent
                    //     float vt = relVel.dot(tangent);
                    //     // Calculate the frictional force based on normal force
                    //     float normal_force = 
                    //     float frictional_force = surface_friction * ;
                    //     body1->apply_force(frictional_force);
                    //     body2->apply_force(frictional_force);
                    // }
                // }

            }
        }
    }

    // If a custom step callback function is set, call it
    if (m_func_step_callback) {
        m_func_step_callback(*this, dt);
    }
    // Calculate the actual real time factor based on the step_dt and dt
    float actual_real_time_factor = (dt/step_dt);
    return actual_real_time_factor;
}