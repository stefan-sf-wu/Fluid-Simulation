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
const integrator integration_method  = integrator::ex_euler;

// Rigid Body -------------------------------------------------------------------//
struct state
{
    glm::vec3 x;        // position of center of mass
    glm::mat3 R;        // rotation matrix
    glm::vec3 P;        // momentum
    glm::vec3 L;        // angular momentum
};



// OpenGL -------------------------------------------------------------------//
inline glm::vec3 transform_phy2gl(glm::vec3 v) {
    return {
        (v[0] * 2 / k_world_edge_size) - 1,
        (v[1] * 2 / k_world_edge_size) - 1,
        (v[2] * 2 / k_world_edge_size) - 1,
    };
}

inline glm::vec3 transform_gl2phy(glm::vec3 v) {
    return {
        (v[0] + 1) * (k_world_edge_size / 2),
        (v[1] + 1) * (k_world_edge_size / 2),
        (v[2] + 1) * (k_world_edge_size / 2),
    };
}


// Debug --------------------------------------------------------------------//
void print_vec(glm::vec3 vec, std::string var_name = "var_name")
{
    std::cout << var_name << ": ";
    std::cout << "[" << vec[0] << ", " 
                     << vec[1] << ", " 
                     << vec[2] << "]\n";
}

#endif // COMMON_H_