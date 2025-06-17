#pragma once 

#include <Eigen/Core>
#include <cstdint>

// Custom 2D float vector with .x() getter and .set_x() setter.
class FxVec2f : public Eigen::Vector2f {
public:
    // Inherit constructors
    using Eigen::Vector2f::Vector2f;

    // Getter for x and y.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }

    // Setter for x and y.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }
};


// Custom 3D float vector with .x(), .y(), .z() getters and corresponding setters.
class FxVec3f : public Eigen::Vector3f {
public:
    using Eigen::Vector3f::Vector3f;

    // Getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }
    float z() const { return (*this)(2); }

    // Setters.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }
    void set_z(float val) { (*this)(2) = val; }
};


// Custom 4D float vector with .x(), .y(), .z(), .a() getters and corresponding setters.
class FxVec4f : public Eigen::Vector4f {
public:
    using Eigen::Vector4f::Vector4f;

    // Getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }
    float z() const { return (*this)(2); }
    float a() const { return (*this)(3); }

    // Setters.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }
    void set_z(float val) { (*this)(2) = val; }
    void set_a(float val) { (*this)(3) = val; }
};


// Custom 2D unsigned int vector with .x() getter and .set_x() setter.
class FxVec2ui : public Eigen::Matrix<unsigned int, 2, 1> {
public:
    using Base = Eigen::Matrix<unsigned int, 2, 1>;
    using Base::Base;

    unsigned int x() const { return (*this)(0); }
    unsigned int y() const { return (*this)(1); }

    void set_x(unsigned int val) { (*this)(0) = val; }
    void set_y(unsigned int val) { (*this)(1) = val; }
};


// And a 4D 8-bit unsigned integer vector.
class FxVec4ui8 : public Eigen::Matrix<uint8_t, 4, 1> {
public:
    using Base = Eigen::Matrix<uint8_t, 4, 1>;
    using Base::Base;

    uint8_t x() const { return (*this)(0); }
    uint8_t y() const { return (*this)(1); }
    uint8_t z() const { return (*this)(2); }
    uint8_t a() const { return (*this)(3); }

    void set_x(uint8_t val) { (*this)(0) = val; }
    void set_y(uint8_t val) { (*this)(1) = val; }
    void set_z(uint8_t val) { (*this)(2) = val; }
    void set_a(uint8_t val) { (*this)(3) = val; }
};


// Custom 2x2 float matrix with .a(), .b(), .c(), .d() getters and corresponding setters.
class FxMat2f : public Eigen::Matrix2f {
public:
    using Eigen::Matrix2f::Matrix2f;

    float a() const { return (*this)(0, 0); }
    float b() const { return (*this)(0, 1); }
    float c() const { return (*this)(1, 0); }
    float d() const { return (*this)(1, 1); }

    void set_a(float val) { (*this)(0, 0) = val; }
    void set_b(float val) { (*this)(0, 1) = val; }
    void set_c(float val) { (*this)(1, 0) = val; }
    void set_d(float val) { (*this)(1, 1) = val; }
};
