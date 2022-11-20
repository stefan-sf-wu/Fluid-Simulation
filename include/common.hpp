#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <string>

#include <glm/glm.hpp>

const char k_project_name[] = "RIGID BODIES";

// World Params --------------------------------------------------------------------------//
const int k_world_edge_size = 128;
const glm::vec3 k_gravity = {0.0f, 0.0f, -10.0};

// Timer --------------------------------------------------------------------//
const float k_time_step = 0.01;                 // sec
const unsigned int k_max_display_time = 3000;   // sec

// Integrator --------------------------------------------------------------------//
enum integrator 
{
    ex_euler, im_euler, rk2
};
const integrator integration_method  = integrator::rk2;

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
const glm::vec3 k_hexahedron_init_position = {k_world_edge_size * 0.5f, k_world_edge_size * 0.5f, k_world_edge_size * 0.8f};
const glm::vec3 k_hexahedron_init_velocity = {0.0f, 0.0f, 0.0f};
const glm::mat3 k_hexahedron_init_rotation_matrix = rotation_matrix(M_PI*0.4, M_PI*0.3, M_PI);
const glm::vec3 k_hexahedron_init_angular_velocity = {10.0f, 10.0f, 10.0f};
const glm::vec3 k_hexahedron_mesh_color = {0.8f, 0.25f, 0.25f};

struct state_dt
{
    glm::vec3 v;        // velocity of center of mass = (P / m)
    glm::mat3 R_dt;     // derivative of rotation matrix = (I_inverse * L)
    glm::vec3 P_dt;     // derivative of momentum = (sum of force)
    glm::vec3 L_dt;     // derivative of angular momentum = (sum of torque)

    inline state_dt operator*(float f)
    {
        return {
            v    * f,
            R_dt * f,
            P_dt * f,
            L_dt * f
        };
    }
};

struct state
{
    glm::vec3 x;        // position of center of mass
    glm::mat3 R;        // rotation matrix
    glm::vec3 P;        // linear momentum = (m * v)
    glm::vec3 L;        // angular momentum = (I * ω)

    inline void operator+=(state_dt st_dt) 
    {
        x += st_dt.v;
        R += st_dt.R_dt;
        P += st_dt.P_dt;
        L += st_dt.L_dt;
    }
    inline state operator+(state_dt st_dt) 
    {
        return 
        {
            x += st_dt.v,
            R += st_dt.R_dt,
            P += st_dt.P_dt,
            L += st_dt.L_dt
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