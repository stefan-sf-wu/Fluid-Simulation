#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <string>

#include <glm/glm.hpp>

const char k_project_name[] = "FLUID SIMULATION";

// World Params --------------------------------------------------------------------------//
const int k_world_edge_size = 128;
const glm::vec3 k_gravity = {0.0f, 0.0f, -20.0};

// Timer --------------------------------------------------------------------//
const float k_time_step = 0.01;                 // sec
const unsigned int k_max_display_time = 60;     // sec

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

// Particle -----------------------------------------------------------------//
const unsigned int k_num_particles = 1000;


// Viscosity -----------------------------------------------------------------//
struct viscosities
{
    float density;      // kg/m^3
    float dynamic;      // kg/m-s
    float kinematic;    // m^2/s
};

const viscosities water_viscosities = {998, 1000, 1};
const viscosities mercury_viscosities = {13500, 1550, 0.115};
const viscosities air_viscosities = {1.18, 18.7, 15.8};


// Integrator --------------------------------------------------------------------//
enum integrator 
{
    ex_euler, im_euler, rk2
};
const integrator integration_method  = integrator::im_euler;


// OpenGL -------------------------------------------------------------------//
inline glm::vec3 transform_world2gl(glm::vec3 &v) { return (v * 2.0f / (float)k_world_edge_size) - 1.0f; }
inline glm::vec3 transform_gl2world(glm::vec3 &v) { return (v + 1.0f) * (float)(k_world_edge_size / 2.0f); }

// Debug --------------------------------------------------------------------//
void print_vec(glm::vec3 vec, std::string var_name = "var_name")
{
    std::cout << var_name << ": ";
    std::cout << "[" << vec[0] << ", " 
                     << vec[1] << ", " 
                     << vec[2] << "]\n";
}

#endif // COMMON_H_