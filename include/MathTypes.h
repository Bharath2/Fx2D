#pragma once 

#include <Eigen/Core>
#include <cstdint>
// #include <Eigen/Dense>
#include <vector>
#include <initializer_list>
#include <algorithm> 
#include <iostream>
#include <string>
// #include <stdexcept>
#include <type_traits>
#include <concepts> 

template<typename T>
concept Numeric = std::integral<T> || std::floating_point<T>; // float, double, long double, int etc..

// Custom 2D float vector with .x() getter and .set_x() setter.
class FxVec2f : public Eigen::Vector2f {
public:
    // Inherit constructors
    using Eigen::Vector2f::Vector2f;

    // Getter for x and y.
    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }

    // Const getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }

    // Setter for x and y.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }

    // — in-place rotation by θ radians
    FxVec2f& rotate_inplace(float theta)  noexcept {
        const float c = std::cos(theta), s = std::sin(theta);
        float xi = x(), yi = y();
        set_x(xi * c - yi * s);
        set_y(xi * s + yi * c);
        return *this;
    }

    // — non-mutating rotation: returns a rotated copy
    FxVec2f rotate(float theta) const  noexcept {
        return FxVec2f(*this).rotate_inplace(theta);
    }
};

using FxVec2fMap = Eigen::Map<Eigen::Vector2f>;

// Custom 3D float vector with .x(), .y(), .z() getters and corresponding setters.
class FxVec3f : public Eigen::Vector3f {
public:
    using Eigen::Vector3f::Vector3f;

    // Getters.
    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }
    float& z() { return (*this)(2); }
    // to use the third value as orientation 
    float& theta() { return (*this)(2); }

     // Const getters.
    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }
    float z() const { return (*this)(2); }
    float theta() const { return (*this)(2); }

    // Setters.
    void set_x(float val) { (*this)(0) = val; }
    void set_y(float val) { (*this)(1) = val; }
    void set_z(float val) { (*this)(2) = val; }
    // when used as orientation
    void set_theta(float val) { (*this)(2) = val; }

    FxVec2fMap xy() { return FxVec2fMap(this->data()); }
    FxVec2f get_xy() { return FxVec2f(this->data()); }
    FxVec2f xy() const { return this->head<2>(); }
    void set_xy(const FxVec2f& v2) { this->head<2>() = v2; }
};


// Custom 4D float vector with .x(), .y(), .z(), .a() getters and corresponding setters.
class FxVec4f : public Eigen::Vector4f {
public:
    using Eigen::Vector4f::Vector4f;

    // Getters.
    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }
    float& z() { return (*this)(2); }
    float& a() { return (*this)(3); }

    // Const getters.
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

// FxVec2f scalar operations with generic Numeric s
template<Numeric S>
inline FxVec2f operator*(FxVec2f const& v, S s) { return FxVec2f(v.array() * static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator*(S s, FxVec2f const& v) { return v * s; }
template<Numeric S>
inline FxVec2f operator/(FxVec2f const& v, S s) { return FxVec2f(v.array() / static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator+(FxVec2f const& v, S s) { return FxVec2f(v.array() + static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator+(S s, FxVec2f const& v) { return v + s; }
template<Numeric S>
inline FxVec2f operator-(FxVec2f const& v, S s) { return FxVec2f(v.array() - static_cast<float>(s)); }
template<Numeric S>
inline FxVec2f operator-(S s, FxVec2f const& v) { return FxVec2f((FxVec2f::Scalar(static_cast<float>(s)) * FxVec2f::Ones()).array() - v.array()); }

// FxVec3f scalar operations with generic Numeric s
template<Numeric S>
inline FxVec3f operator*(FxVec3f const& v, S s) { return FxVec3f(v.array() * static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator*(S s, FxVec3f const& v) { return v * s; }
template<Numeric S>
inline FxVec3f operator/(FxVec3f const& v, S s) { return FxVec3f(v.array() / static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator+(FxVec3f const& v, S s) { return FxVec3f(v.array() + static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator+(S s, FxVec3f const& v) { return v + s; }
template<Numeric S>
inline FxVec3f operator-(FxVec3f const& v, S s) { return FxVec3f(v.array() - static_cast<float>(s)); }
template<Numeric S>
inline FxVec3f operator-(S s, FxVec3f const& v) { return FxVec3f((FxVec3f::Scalar(static_cast<float>(s)) * FxVec3f::Ones()).array() - v.array()); }

// FxVec4f scalar operations with generic Numeric s
template<Numeric S>
inline FxVec4f operator*(FxVec4f const& v, S s) { return FxVec4f(v.array() * static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator*(S s, FxVec4f const& v) { return v * s; }
template<Numeric S>
inline FxVec4f operator/(FxVec4f const& v, S s) { return FxVec4f(v.array() / static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator+(FxVec4f const& v, S s) { return FxVec4f(v.array() + static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator+(S s, FxVec4f const& v) { return v + s; }
template<Numeric S>
inline FxVec4f operator-(FxVec4f const& v, S s) { return FxVec4f(v.array() - static_cast<float>(s)); }
template<Numeric S>
inline FxVec4f operator-(S s, FxVec4f const& v) { return FxVec4f((FxVec4f::Scalar(static_cast<float>(s)) * FxVec4f::Ones()).array() - v.array()); }

// FxVec2f in-place scalar operations
template<Numeric S>
inline FxVec2f& operator+=(FxVec2f& v, S s) {
    v.array() += static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec2f& operator-=(FxVec2f& v, S s) {
    v.array() -= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec2f& operator*=(FxVec2f& v, S s) {
    v.array() *= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec2f& operator/=(FxVec2f& v, S s) {
    v.array() /= static_cast<float>(s);
    return v;
}

// FxVec3f in-place scalar operations
template<Numeric S>
inline FxVec3f& operator+=(FxVec3f& v, S s) {
    v.array() += static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec3f& operator-=(FxVec3f& v, S s) {
    v.array() -= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec3f& operator*=(FxVec3f& v, S s) {
    v.array() *= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec3f& operator/=(FxVec3f& v, S s) {
    v.array() /= static_cast<float>(s);
    return v;
}

// FxVec4f in-place scalar operations
template<Numeric S>
inline FxVec4f& operator+=(FxVec4f& v, S s) {
    v.array() += static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec4f& operator-=(FxVec4f& v, S s) {
    v.array() -= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec4f& operator*=(FxVec4f& v, S s) {
    v.array() *= static_cast<float>(s);
    return v;
}
template<Numeric S>
inline FxVec4f& operator/=(FxVec4f& v, S s) {
    v.array() /= static_cast<float>(s);
    return v;
}

// free non-member operators
inline FxVec2f& operator*=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() *= rhs.array();
    return lhs;
}
inline FxVec2f& operator/=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() /= rhs.array();
    return lhs;
}
inline FxVec2f& operator+=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() += rhs.array();
    return lhs;
}
inline FxVec2f& operator-=(FxVec2f& lhs, const FxVec2f& rhs) {
    lhs.array() -= rhs.array();
    return lhs;
}
// FxVec2f scalar/vector division: s / v
template<Numeric S>
inline FxVec2f operator/(S s, FxVec2f const& v) {
    return FxVec2f((FxVec2f::Scalar(static_cast<float>(s)) * FxVec2f::Ones()).array() / v.array());
}

// FxVec3f scalar/vector division: s / v
template<Numeric S>
inline FxVec3f operator/(S s, FxVec3f const& v) {
    return FxVec3f((FxVec3f::Scalar(static_cast<float>(s)) * FxVec3f::Ones()).array() / v.array());
}

// FxVec4f scalar/vector division: s / v
template<Numeric S>
inline FxVec4f operator/(S s, FxVec4f const& v) {
    return FxVec4f((FxVec4f::Scalar(static_cast<float>(s)) * FxVec4f::Ones()).array() / v.array());
}

// Custom 2D unsigned int vector with .x() getter and .set_x() setter.
class FxVec2ui : public Eigen::Matrix<unsigned int, 2, 1> {
public:
    using Base = Eigen::Matrix<unsigned int, 2, 1>;
    using Base::Base;

    unsigned int& x() { return (*this)(0); }
    unsigned int& y() { return (*this)(1); }

    // Const getters.
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

    uint8_t& x() { return (*this)(0); }
    uint8_t& y() { return (*this)(1); }
    uint8_t& z() { return (*this)(2); }
    uint8_t& a() { return (*this)(3); }

     // Const getters.
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

    float& a() { return (*this)(0, 0); }
    float& b() { return (*this)(0, 1); }
    float& c() { return (*this)(1, 0); }
    float& d() { return (*this)(1, 1); }

     // Const getters.
    float a() const { return (*this)(0, 0); }
    float b() const { return (*this)(0, 1); }
    float c() const { return (*this)(1, 0); }
    float d() const { return (*this)(1, 1); }

    void set_a(float val) { (*this)(0, 0) = val; }
    void set_b(float val) { (*this)(0, 1) = val; }
    void set_c(float val) { (*this)(1, 0) = val; }
    void set_d(float val) { (*this)(1, 1) = val; }

    //The inverse (transpose) of the rotation matrix.
    FxMat2f inv_rotation() const {
        return this->transpose();
    }

};


class FxMat3f : public Eigen::Matrix3f {
public:
    using Eigen::Matrix3f::Matrix3f;

    // Accessors for each element of the 3x3 matrix
    // Row 0
    float& a() { return (*this)(0, 0); }
    float& b() { return (*this)(0, 1); }
    float& c() { return (*this)(0, 2); }
    // Row 1
    float& d() { return (*this)(1, 0); }
    float& e() { return (*this)(1, 1); }
    float& f() { return (*this)(1, 2); }
    // Row 2
    float& g() { return (*this)(2, 0); }
    float& h() { return (*this)(2, 1); }
    float& i() { return (*this)(2, 2); }

    // Const getters.
    float a() const { return (*this)(0, 0); }
    float b() const { return (*this)(0, 1); }
    float c() const { return (*this)(0, 2); }
    float d() const { return (*this)(1, 0); }
    float e() const { return (*this)(1, 1); }
    float f() const { return (*this)(1, 2); }
    float g() const { return (*this)(2, 0); }
    float h() const { return (*this)(2, 1); }
    float i() const { return (*this)(2, 2); }
    
    // Setters for each element of the 3x3 matrix
    // Row 0
    void set_a(float val) { (*this)(0, 0) = val; }
    void set_b(float val) { (*this)(0, 1) = val; }
    void set_c(float val) { (*this)(0, 2) = val; }
    // Row 1
    void set_d(float val) { (*this)(1, 0) = val; }
    void set_e(float val) { (*this)(1, 1) = val; }
    void set_f(float val) { (*this)(1, 2) = val; }
    // Row 2
    void set_g(float val) { (*this)(2, 0) = val; }
    void set_h(float val) { (*this)(2, 1) = val; }
    void set_i(float val) { (*this)(2, 2) = val; }

    // For a homogeneous transformation matrix M = [ R  t ]
    //                                             [ 0  1 ]
    // its inverse is: M⁻¹ = [ Rᵀ  -Rᵀt ]
    //                       [  0    1  ]
    // Extract the 2x2 rotation matrix from the upper-left block as an FxMat2f
    FxMat2f Rot() const { return this->block<2, 2>(0, 0).eval(); }
    // Extract the translation vector (first two elements of the third column) as an FxVec2f
    FxVec2f t() const { return FxVec2f((*this)(0, 2), (*this)(1, 2)); }

    // set the rotation part from an FxMat2f
    void set_Rot(const FxMat2f &R) {
         (*this)(0, 0) = R(0, 0);
         (*this)(0, 1) = R(0, 1);
         (*this)(1, 0) = R(1, 0);
         (*this)(1, 1) = R(1, 1);
    }

    //set the translation part from an FxVec2f
    void set_t(const FxVec2f &trans) {
         (*this)(0, 2) = trans.x();  // Assuming FxVec2f has x() method.
         (*this)(1, 2) = trans.y();  // And a y() method.
    }

    // Compute and return the inverse transformation.
    FxMat3f inv_transform() const {
        FxMat3f inv;  // This will store the inverse.

        // Compute the rotation transpose (which is the inverse of a rotation).
        inv(0, 0) = (*this)(0, 0);
        inv(0, 1) = (*this)(1, 0);
        inv(1, 0) = (*this)(0, 1);
        inv(1, 1) = (*this)(1, 1);

        // Compute the new translation vector as -Rᵀt.
        float t0 = (*this)(0, 2); // original translation x value.
        float t1 = (*this)(1, 2); // original translation y value.
        inv(0, 2) = - (inv(0, 0) * t0 + inv(0, 1) * t1);
        inv(1, 2) = - (inv(1, 0) * t0 + inv(1, 1) * t1);

        // Set the homogeneous row.
        inv(2, 0) = 0;
        inv(2, 1) = 0;
        inv(2, 2) = 1;

        return inv;
    }
};

//────────────────────────────────────────────────────────────────────────────
// FxArray: numpy style 2d array
//────────────────────────────────────────────────────────────────────────────

// 1) define the concepts
template<typename T>
concept NumericOrFxVec =
       std::integral<T>               // all integer types
    || std::floating_point<T>          // float, double, long double
    || std::same_as<T, FxVec2f>
    || std::same_as<T, FxVec3f>
    || std::same_as<T, FxVec4f>;
    // || std::same_as<T, FxVecXf>;

template<typename T>
concept FxVecT =
       std::same_as<T, FxVec2f>
    || std::same_as<T, FxVec3f>
    || std::same_as<T, FxVec4f>;     


template<typename U, typename T>
concept ConvertibleOrNumeric = std::convertible_to<U, T> || Numeric<U>;



// define the FxArray class template
template<NumericOrFxVec T>
class FxArray {
  private:
    std::size_t    m_size  = 0;
    std::unique_ptr<T[]> m_arr;

  protected:
    // throws if index is out of bounds
    template<std::integral I>
    std::size_t checkIndex(I idx) const {
        auto i = static_cast<long long>(idx);         // signed capture for error text
        auto u = static_cast<std::size_t>(i);         // cast for bounds test
        if (i < 0 || u >= m_size) {
            throw std::out_of_range(
                "FxArray::[] index " + std::to_string(i)
           + " out of range for [0," + std::to_string(m_size) + ")");
        }
        return u;
    }

    // throws if empty
    void throw_if_empty(char const* what) const {
        if (m_size == 0)
        throw std::runtime_error(std::string("FxArray::") + what + " on empty array");
    } 

    // throws if size mismatch
    void throw_if_size_mismatch(char const* what, size_t o_size) const {
        if (m_size != o_size)
        throw std::invalid_argument(std::string("FxArray::operator") + what + " size mismatch");
    } 

  public:
    // 1) default (empty)
    FxArray() = default;
    // 2) n sized ctor (all zeros)
    explicit FxArray(std::size_t n)
      : m_size(n)
      , m_arr(std::make_unique<T[]>(n))
    {}

    // 3) Dedicated init_list ctor — for braced lists
    FxArray(std::initializer_list<T> init)
      : FxArray(init.size())
    { std::copy(init.begin(), init.end(), m_arr.get()); }

    // 4) one ctor for std::vector
    FxArray(std::vector<T> const& v)
      : FxArray(v.size())
    { std::copy(v.begin(), v.end(), m_arr.get()); }

    // 5) one overload for c style array
    template<std::size_t N>
    FxArray(T const (&arr)[N])
      : FxArray(N)
    { std::copy_n(arr, N, m_arr.get()); }

    // deep‐copy copy‐ctor using copy_n
    FxArray(FxArray const& o)
      : FxArray(o.m_size)
    { std::copy_n(o.m_arr.get(), m_size, m_arr.get()); }

    // move-assignment (O(1) swap of pointers and size)
    FxArray(FxArray&&) noexcept = default;

    // = move-assignment
    FxArray& operator=(FxArray const& o) {
      if (&o == this) return *this;
      FxArray tmp(o);
      swap(tmp);
      return *this;
    }

    // assign from std::vector<T>
    FxArray& operator=(std::vector<T> const& v) {
        // reuse your vector‐ctor + swap
        FxArray tmp(v);
        swap(tmp);
        return *this;
    }

    // assign from initializer_list<T>
    FxArray& operator=(std::initializer_list<T> init) {
        FxArray tmp(init);
        swap(tmp);
        return *this;
    }

    FxArray& operator=(FxArray&&) noexcept = default;

    // 1) UNCHECKED, inlined, noexcept operator[] for hot loops
    T&       operator[](size_t i)       noexcept { return m_arr[i]; }
    T const& operator[](size_t i) const noexcept { return m_arr[i]; }

    template<std::integral I>
    T& operator()(I i) noexcept { return m_arr[static_cast<size_t>(i)]; }

    template<std::integral I>
    T const& operator()(I i) const noexcept { return m_arr[static_cast<size_t>(i)]; }
    
    // 2) BOUNDS-CHECKED at(), still accepts *any* integral index
    template<std::integral I>
    T& at(I idx) { return m_arr[checkIndex(idx)]; }
    template<std::integral I>
    T const& at(I idx) const { return m_arr[checkIndex(idx)]; }

    // for iterators
    T*       begin()       noexcept { return data(); }
    T const* begin() const noexcept { return data(); }
    T*       end()         noexcept { return data() + m_size; }
    T const* end()   const noexcept { return data() + m_size; }

    // size & raw data
    size_t size()  const noexcept { return m_size; }
    bool   empty() const noexcept { return m_size == 0; }
    T*       data()       noexcept      { return m_arr.get(); }
    T const* data() const noexcept      { return m_arr.get(); }

    void swap(FxArray& o) noexcept {
      std::swap(m_size,  o.m_size);
      std::swap(m_arr,  o.m_arr);
    }

    template<typename U> requires (Numeric<T> && Numeric<U>)
    FxArray<U> as() const {
        FxArray<U> result(m_size);
        for (std::size_t i = 0; i < m_size; ++i)
            result[i] = static_cast<U>(m_arr[i]);
        return result;
    }

    template<typename Compare>
    T min(Compare cmp) const {
        throw_if_empty("min");
        T best = m_arr[0];
        for (std::size_t i = 1; i < m_size; ++i)
        if (cmp(m_arr[i], best))
            best = m_arr[i];
        return best;
    }

    template<typename Compare>
    T max(Compare cmp) const {
        throw_if_empty("max");
        T best = m_arr[0];
        for (std::size_t i = 1; i < m_size; ++i)
        if (cmp(best, m_arr[i]))
            best = m_arr[i];
        return best;
    }

    // overloads that use operator< by default
    T min() const { return min(std::less<T>{}); }
    T max() const { return max(std::less<T>{}); }

    // --- mean: requires T() + T+= U + T /= scalar ---
    T mean() const {
        throw_if_empty("mean");
        T sum{};                              // assumes T{} zero‐initializes
        for (std::size_t i = 0; i < m_size; ++i)
            sum += m_arr[i];
        return sum / m_size;
    }

    // mean as float (casts each element to double for accuracy, returns float)
    float meanf() const requires Numeric<T> {
        throw_if_empty("meanf");
        double sum = 0.0;
        for (std::size_t i = 0; i < m_size; ++i)
            sum += static_cast<double>(m_arr[i]);
        return static_cast<float>(sum / m_size);
    }

    //population standard deviation as float
    float stddev() const requires Numeric<T> {
        throw_if_empty("stddev");
        double m = meanf();
        double acc = 0.0;
        for (std::size_t i = 0; i < m_size; ++i) {
            double d = static_cast<double>(m_arr[i]) - m;
            acc += d * d;
        }
        return static_cast<float>( std::sqrt(acc / m_size) );
    }

    // --- unary minus (element‐wise negate) ---
    FxArray operator-() const {
        FxArray result(m_size);
        for (std::size_t i = 0; i < m_size; ++i)
            result.m_arr[i] = -m_arr[i];
        return result;
    }

    // --- in-place with a single T ---
    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] += v;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] -= v;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] *= v;
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(U const& v) {
        for (std::size_t i = 0; i < m_size; ++i) m_arr[i] /= v;
        return *this;
    }

    // --- in-place element‐wise with another FxArray ---
    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator+=(FxArray<U> const& o) {
        throw_if_size_mismatch("+=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] += o.m_arr[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator-=(FxArray<U> const& o) {
        throw_if_size_mismatch("-=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] -= o.m_arr[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator*=(FxArray<U> const& o) {
        throw_if_size_mismatch("*=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] *= o.m_arr[i];
        return *this;
    }

    template<typename U> requires ConvertibleOrNumeric<U, T>
    FxArray& operator/=(FxArray<U> const& o) {
        throw_if_size_mismatch("/=", o.size());
        for (std::size_t i = 0; i < m_size; ++i)
            m_arr[i] /= o.m_arr[i];
        return *this;
    }

    // dot with one FxVec2f → returns FxArray<float>
    FxArray<float> dot(T const& v) const requires FxVecT<T>{
        FxArray<float> out(this->size());
        for (size_t i = 0; i < this->size(); ++i)
            out[i] = (*this)[i].dot(v);
        return out;
    }

    // element-wise dot with another FxVec2fArray
    FxArray<float> dot(FxArray<T> const& o) const requires FxVecT<T>{
        this->throw_if_size_mismatch("dot", o.size());
        FxArray<float> out(this->size());
        for (size_t i = 0; i < this->size(); ++i)
            out[i] = (*this)[i].dot(o[i]);
        return out;
    }

    // in-place rotation by θ (radians)
    FxArray& rotate_inplace(float theta) requires std::same_as<T, FxVec2f>{
        float c = std::cos(theta), s = std::sin(theta);
        for (auto& e : *this) {
            float x = e.x(), y = e.y();
            e.x() = x * c - y * s;
            e.y() = x * s + y * c;
        }
        return *this;
    }

    // non-mutating rotate → new array
    FxArray rotate(float theta) const requires std::same_as<T, FxVec2f>{
        FxArray tmp = *this;
        tmp.rotate_inplace(theta);
        return tmp;
    }

    FxArray<float> bounds() const requires std::same_as<T, FxVec2f>{
        this->throw_if_empty("extrema");
        const auto& v0 = (*this)[0];
        float minx = v0.x(), maxx = v0.x();
        float miny = v0.y(), maxy = v0.y();
        for (size_t i = 1, n = this->size(); i < n; ++i) {
            const auto& v = (*this)[i];
            float x = v.x(), y = v.y();
            if (x < minx)      minx = x;
            else if (x > maxx) maxx = x;
            if (y < miny)      miny = y;
            else if (y > maxy) maxy = y;
        }
        return { minx, miny, maxx, maxy };
    }

};

template<NumericOrFxVec T>
void swap(FxArray<T>& a, FxArray<T>& b) noexcept {
  a.swap(b);
}


//────────────────────────────────────────────────────────────────────────────
// FxVec2fArray: fixed‐size array of FxVec2f
//────────────────────────────────────────────────────────────────────────────
using FxVec2fArray = FxArray<FxVec2f>;

// Scalar–Array
template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator+(FxArray<T> a, U const& scalar) { return a += scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator-(FxArray<T> a, U const& scalar) { return a -= scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator*(FxArray<T> a, U const& scalar) { return a *= scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator/(FxArray<T> a, U const& scalar) { return a /= scalar; }

// Scalar on the left
template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator+(U const& scalar, FxArray<T> a) { return a += scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator*(U const& scalar, FxArray<T> a) { return a *= scalar; }

template<typename T, typename U> requires ConvertibleOrNumeric<U, T>
inline FxArray<T> operator-(U const& scalar, FxArray<T> a) { return -a + scalar; }

// Scalar–Array division: scalar / array
template<typename T, Numeric U> 
inline FxArray<T> operator/(U const& scalar, FxArray<T> const& a) {
  FxArray<T> result(a.size());
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = scalar / a[i];
  return result;
}


template<typename T>
inline FxArray<T> operator/(T const& scalar, FxArray<T> const& a) {
  FxArray<T> result(a.size());
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = scalar / a[i];
  return result;
}

// Array–Array
template<typename T>
inline FxArray<T> operator+(FxArray<T> a, const FxArray<T>& b) { return a += b; }

template<typename T>
inline FxArray<T> operator-(FxArray<T> a, const FxArray<T>& b) { return a -= b; }

template<typename T>
inline FxArray<T> operator*(FxArray<T> a, const FxArray<T>& b) { return a *= b; }

template<typename T>
inline FxArray<T> operator/(FxArray<T> a, const FxArray<T>& b) { return a /= b; }

template<Numeric T>
inline std::ostream& operator<<(std::ostream& os, FxArray<T> const& a) {
    os << "FxArray { ";
    for (size_t i = 0; i < a.size(); ++i) os << a[i] << ", ";
    os << "}";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, FxVec2fArray const& a) {
    os << "FxVec2fArray { ";
    for(size_t i = 0; i < a.size(); ++i)
        os << "("<<a[i].x()<<" "<<a[i].y()<<"), ";
    os <<"} ";
    return os;
}

//---------------------------------------------
// Shape Definition
//---------------------------------------------
struct FxShape {
  private:
    std::string    m_shape_type;                     // "Circle" or "Polygon"
    float          m_radius;                         // bounding radius 
    FxVec2fArray   m_vertices;                       // only used for polygons relative to centroid
    FxVec3f        m_offset_pose {0.0f, 0.0f, 0.0f};          // initial offset pose in world coordinatesates
    FxVec2fArray m_
    
  protected:
    // 1) Compute the bounding radius from (0,0)
    static float calc_radius(const FxVec2fArray& verts) {
        float maxSq = 0.0f;
        for (size_t i = 0; i < verts.size(); ++i) {
            const FxVec2f& v = verts[i];
            float d2 = v.x()*v.x() + v.y()*v.y();
            if (d2 > maxSq) maxSq = d2;
        }
        return std::sqrt(maxSq);
    }
    // 2) Shoelace‐formula area must exceed 2e–6f
    static bool is_valid_area(const FxVec2fArray& verts) {
        const float minArea = 1e-6f;
        size_t n = verts.size();
        if (n < 3) return false;
        float sum = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& P = verts[i];
            const FxVec2f& Q = verts[(i+1) % n];
            sum += P.x()*Q.y() - Q.x()*P.y();
        }
        float area = 0.5f * sum;
        return std::fabs(area) > minArea;
    }
    // 3) Convexity: all cross‐products have same sign
    static bool is_convex(const FxVec2fArray& verts) {
        size_t n = verts.size();
        if (n < 3) return false;
        bool gotPos = false, gotNeg = false;
        for (size_t i = 0; i < n; ++i) {
            const FxVec2f& A = verts[i];
            const FxVec2f& B = verts[(i+1)%n];
            const FxVec2f& C = verts[(i+2)%n];
            float cross =
                (B.x()-A.x())*(C.y()-B.y()) -
                (B.y()-A.y())*(C.x()-B.x());
            if      (cross > 0) gotPos = true;
            else if (cross < 0) gotNeg = true;
            if (gotPos && gotNeg) return false;
        }
        return true;
    }

  public:
    // default ctor
    FxShape() : m_shape_type("Circle"), m_radius(0.5f) {}

    //–– Circle ctor
    FxShape(float radius) {
        if (radius <= 1e-6f)
            throw std::invalid_argument("FxShape: radius must be > 0");
        m_shape_type = "Circle";
        m_radius     = radius;
    }

    //–– Polygon from arbitrary vertices
    FxShape(const FxVec2fArray& vertices) {
        if (!is_valid_area(m_vertices))
            throw std::invalid_argument("FxShape: area ≤ 2e-6");
        if (!is_convex(m_vertices))
            throw std::invalid_argument("FxShape: not convex");
        m_shape_type = "Polygon";
        // centroid will be pushed to {0.0f, 0.0f}
        m_vertices   = vertices - vertices.mean();
        m_radius     = calc_radius(m_vertices);
    }

    //–– Rectangle centered at origin, width=size.x(), height=size.y()
    FxShape(const FxVec2f& size) {
        if (size.x() <= 0.0f || size.y() <= 0.0f)
            throw std::invalid_argument("FxShape: dimensions must be > 0");
        float hx = size.x() * 0.5f;
        float hy = size.y() * 0.5f;
        // Check for valid area
        if (hx*hy <= 1e-6f)
            throw std::runtime_error("FxShape: degenerate rectangle");
         // build CCW rectangle around (0, 0)
        m_vertices = { { -hx, -hy }, {  hx, -hy }, {  hx,  hy }, { -hx,  hy }};
        m_shape_type = "Polygon";
        m_radius     = std::sqrt(hx*hx + hy*hy);
    }

    // getters for shape properties
    std::string shape_type() const { return m_shape_type; }
    float radius() const { return m_radius; }
    FxVec2fArray vertices() const { return m_vertices; }

    // offset pose setter and getter 
    void set_offset_pose(const FxVec3f& o_pose){ m_offset_pose = o_pose; }
    FxVec3f offset_pose() const { return m_offset_pose; }

};