#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <map>

#include <glm/glm.hpp>

const char k_project_name[] = "FLUID SIMULATION";

// World Params --------------------------------------------------------------------------//
const int k_world_edge_size = 128;
const glm::vec3 k_gravity = {0.0f, 0.0f, -20.0};

// Timer --------------------------------------------------------------------//
const float k_time_step = 0.01;                 // sec
const unsigned int k_max_display_time = 60;     // sec

// Material -----------------------------------------------------------------//
struct viscosity
{
    float density;      // kg/m^3
    float dynamic;      // kg/m-s
    float kinematic;    // m^2/s
};

enum material
{
    water, mercury, air
};

std::map<material, viscosity> material_viscosity_map = 
{
    { material::water, {998, 1000, 1} },
    { material::mercury, {13500, 1550, 0.115} },
    { material::air, {1.18, 18.7, 15.8} }
};

material k_material = material::water;
viscosity k_viscosity = material_viscosity_map[k_material];

// Particle ----------------------------------------------------------------------//
const unsigned int k_num_particle = 10;
const float k_fluid_volume = std::pow(k_world_edge_size, 3) / 2;  // fill half of the box
const float k_particle_weight = k_viscosity.density * k_fluid_volume / k_num_particle;
const float k_particle_radius = std::pow((((3 * k_particle_weight) / (4 * M_PI * k_viscosity.density))), 1/3);


// SPH System ----------------------------------------------------------------------//
const unsigned int k_num_neighboring_particle = 5;
const float k_sph_kernal_s = std::pow((3 * k_fluid_volume * k_num_neighboring_particle) / (4 * M_PI * k_num_particle), 1/3);

static std::vector<std::vector<unsigned int>> neighborhood(k_num_particle, std::vector<unsigned int>(0));

inline float sph_kernel_function(float sph_kernel_r)
{
    sph_kernel_r = std::abs(sph_kernel_r);
    
    float rs_ratio = sph_kernel_r / k_sph_kernal_s;
    float ret = (1 / (M_PI * std::pow(k_sph_kernal_s, 3)));

    if (rs_ratio < 0 || rs_ratio > 2) 
    {
        return ret * 0.0f;
    } 
    else if (0 <= rs_ratio && rs_ratio <= 1)
    {
        return ret * (1 - (3/2) * std::pow(rs_ratio, 2) + (3/4) * std::pow(rs_ratio, 3));
    } 
    else if (1 <= rs_ratio && rs_ratio <= 2)
    {
        return ret * (1/4) * std::pow((2 - rs_ratio), 3);
    }
    return -1;
}


// Integrator --------------------------------------------------------------------//
enum integrator 
{
    ex_euler, im_euler, rk2, verlet
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