#ifndef COMMON_H_
#define COMMON_H_
#ifndef GLM_ENABLE_EXPERIMENTAL
    #define GLM_ENABLE_EXPERIMENTAL
#endif

#include <vector>
#include <iostream>
#include <cmath>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

const char k_project_name[] = "RIGID BODIES";

// World Params --------------------------------------------------------------------------//
const int k_world_edge_size = 128;
const glm::vec3 k_gravity = {0.0f, 0.0f, -30.0};

// Timer --------------------------------------------------------------------//
const float k_time_step = 0.01;                 // sec
const unsigned int k_max_display_time = 3000;   // sec

// Collision --------------------------------------------------------------------//
const float k_restitution = 0.5f;
struct collision_result
{
    glm::vec3 r_a;
    glm::vec3 n;

    inline bool operator!=(collision_result cr)
    {
        return (r_a != cr.r_a) || (n != cr.n);
    }
    inline bool operator==(collision_result cr)
    {
        return (r_a == cr.r_a) && (n == cr.n);
    }
};
const collision_result k_null_collision_result =
{
    {-1.0f, -1.0f, -1.0f},
    {-1.0f, -1.0f, -1.0f}
};

// Integrator --------------------------------------------------------------------//
enum integrator 
{
    ex_euler, im_euler, rk2
};
const integrator integration_method  = integrator::ex_euler;

// Rigid Body -------------------------------------------------------------------//
glm::mat3 rotation_matrix(float beta, float gamma, float alpha)
{
    /** 
     * (alpha, beta, gamma) about axes (x, y, z) 
     * ref: https://en.wikipedia.org/wiki/Rotation_matrix
     */
    glm::mat3 r;
    r[0][0] =  cos(beta)  * cos(gamma);
    r[0][1] =  sin(alpha) * sin(beta)  * cos(gamma) - cos(alpha) * sin(gamma);
    r[0][2] =  cos(alpha) * sin(beta)  * cos(gamma) + sin(alpha) * sin(gamma);
    r[1][0] =  cos(beta)  * sin(gamma);
    r[1][1] =  cos(alpha) * cos(gamma) + sin(alpha) * sin(beta)  * sin(gamma);
    r[1][2] =  cos(alpha) * sin(beta)  * sin(gamma) - sin(alpha) * cos(gamma);
    r[2][0] = -sin(beta);
    r[2][1] =  sin(alpha) * cos(beta);
    r[2][2] =  cos(alpha) * cos(beta);
    return r;
}

inline glm::mat3 normalize_matrix_by_row(glm::mat3 mat)
{
    return 
    {
        glm::normalize(glm::vec3{mat[0][0], mat[0][1], mat[0][2]}),  
        glm::normalize(glm::vec3{mat[1][0], mat[1][1], mat[1][2]}),  
        glm::normalize(glm::vec3{mat[2][0], mat[2][1], mat[2][2]}),  
    };
}

const float k_hexahedron_mass = 12; // kg
const glm::vec3 k_hexahedron_init_position = {k_world_edge_size * 0.6f, k_world_edge_size * 0.5f, k_world_edge_size * 1.0f};
const glm::vec3 k_hexahedron_init_velocity = {0.0f, 0.0f, 0.0f};
const glm::vec3 k_hexahedron_init_angular_velocity = {0.0f, 0.0f, 0.0f};
const glm::vec3 k_hexahedron_init_rotate_axis = {0.0f, 0.0f, 0.0f}; //glm::normalize(glm::vec3({0.0f, 0.0f, 0.0f}));
const float k_hexahedron_init_rotate_angle = 0;

struct state_d
{
    glm::vec3 v;       // velocity of center of mass = (P / m)
    glm::quat q_d;     // derivative of quaternion = 1/2 * ω * q
    glm::vec3 P_d;     // derivative of momentum = (sum of force)
    glm::vec3 L_d;     // derivative of angular momentum = (sum of torque)

    inline state_d operator*(float h)
    {
        return {
            v    * h,
            q_d * h,
            P_d * h,
            L_d * h
        };
    }
};

struct state
{
    /**
     * quat: http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions/#how-do-i-create-a-quaternion-in-c-
    */
    glm::vec3 x;        // position of center of mass
    glm::quat q;        // quaternion
    glm::vec3 P;        // linear momentum = (m * v)
    glm::vec3 L;        // angular momentum = (I * ω)

    inline void operator+=(state_d st_dt) 
    {
        x += st_dt.v;
        q += st_dt.q_d;
        P += st_dt.P_d;
        L += st_dt.L_d;
    }
    inline state operator+(state_d st_dt) 
    {
        return 
        {
            x + st_dt.v,
            q + st_dt.q_d,
            P + st_dt.P_d,
            L + st_dt.L_d
        };
    }
};


// OpenGL -------------------------------------------------------------------//
inline glm::vec3 transform_phy2gl(glm::vec3 v) { return (v * 2.0f / (float)k_world_edge_size) - 1.0f; }
inline glm::vec3 transform_gl2phy(glm::vec3 v) { return (v + 1.0f) * (float)(k_world_edge_size / 2.0f); }

// Debug --------------------------------------------------------------------//
void print_vec(glm::vec3 vec, std::string var_name = "var_name")
{
    std::cout << var_name << ": ";
    std::cout << "[" << vec[0] << ", " 
                     << vec[1] << ", " 
                     << vec[2] << "]\n";
}

#endif // COMMON_H_