#ifndef SOLVER_H_
#define SOLVER_H_

#include <vector>
#include <iostream>

#include <glm/glm.hpp>
#include <omp.h>
#include <tbb/tbb.h>
#include <cyCodeBase/cyPointCloud.h>

#include "common.hpp"
#include "particle.hpp"

class Solver
{
private:
    std::vector<glm::vec3> acc_buffer;
    std::vector<glm::vec3> pos_buffer;
    std::vector<glm::vec3> vel_buffer;

    Particle particles;
    cy::PointCloud<glm::vec3, float, 3> kdtree;

public: 
    Solver()
    : acc_buffer(k_num_particle)
    , pos_buffer(k_num_particle)
    , vel_buffer(k_num_particle)
    {
        // acc_buffer.resize(k_num_particle);
        // pos_buffer.resize(k_num_particle);
        // vel_buffer.resize(k_num_particle);
    };

    void compute_next_state()
    {
        compute_forces();

        switch (integration_method)
        {
            case integrator::im_euler:
                integrated_by_im_euler();
                break;
            case integrator::ex_euler:
                integrated_by_ex_euler();
                break;
            case integrator::rk2:
                integrated_by_rk2();
                break;
            case integrator::verlet:
                integrated_by_verlet();
                break;
            default:
                break;
        }
    }

    std::vector<glm::vec3> &get_gl_particle_position() { return particles.get_gl_particle_position(); }
    std::vector<glm::vec3> &get_gl_particle_color() { return particles.get_particle_color(); }

    ~Solver()
    {
    };

private:
    void integrated_by_verlet()
    {
        #pragma omp parallel for 
        for (int i = 0; i < k_num_particle; i++)
        {
            pos_buffer.at(i) = particles.get_particle_position().at(i)
                + particles.get_particle_velocity().at(i)
                + particles.get_particle_acceleration().at(i) * (float)(std::pow(k_time_step, 2) / 2);
            
            acc_buffer.at(i) = particles.get_particle_force().at(i) / particles.get_particle_density().at(i);
            vel_buffer.at(i) = particles.get_particle_velocity().at(i) 
                + (particles.get_particle_acceleration().at(i) + acc_buffer.at(i)) * (k_time_step / 2);
        }
        particles.set_particle_position(pos_buffer);
        particles.set_particle_velocity(vel_buffer);
        particles.set_particle_acceleration(acc_buffer);
    }

    void integrated_by_ex_euler()
    {
    }

    void integrated_by_im_euler()
    {
    }

    void integrated_by_rk2()
    {
    }

    void compute_forces()
    {
        compute_neighborhood();
        compute_density();

    }

    void compute_neighborhood()
    {
        glm::vec3 *pos = &particles.get_particle_position()[0];
        kdtree.Build(k_num_particle, pos);
        for(auto i : neighborhood) { i.clear(); }

        #pragma omp parallel for 
        for (int i = 0; i < k_num_particle; i++)
        {
            kdtree.GetPoints(i, particles.get_particle_position().at(i), k_sph_kernal_s, compute_neighborhood_callback);
        }
    }

    void compute_density()
    {
    }

    static void compute_neighborhood_callback(unsigned int target_index, unsigned int index, glm::vec3 const &p, float distanceSquared, float &radiusSquared)
    {
        neighborhood[target_index].push_back(index);
    }
    

};

#endif // SOLVER_H_