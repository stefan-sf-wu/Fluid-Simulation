#ifndef PARTICLE_HPP_
#define PARTICLE_HPP_

#include <vector>

#include <glm/glm.hpp>
#include <omp.h>

#include "common.hpp"
#include "rand_generator.hpp"

class Particle
{
private:
    // std::vector<glm::vec3> velocity;
    std::vector<glm::vec3> position_;
    std::vector<glm::vec3> gl_position_;
    std::vector<glm::vec3> color_;

    RandGenerator rand_generator;

public:
    Particle() 
    {
        position_.resize(k_num_particles);
        gl_position_.resize(k_num_particles);
        color_.resize(k_num_particles, {1.0f, 1.0f, 1.0f});
        initialize_particle_position();
    };

    std::vector<glm::vec3> &get_particle_position() { return position_; }
    std::vector<glm::vec3> &get_gl_particle_position() 
    { 
        #pragma parallel for collapse(1)
        for (int i = 0; i < k_num_particles; i++)
        {
            gl_position_.at(i) = transform_world2gl(position_[i]);
        }
        return gl_position_;
    }
    std::vector<glm::vec3> &get_particle_color() { return color_; }

    ~Particle() {};

private:
    void initialize_particle_position()
    {
        #pragma parallel for
        for (int i = 0; i < k_num_particles; i++)
        {
            color_.at(i) = rand_generator.generate_random_uniform_vec3(0.3, 1.0);
            position_.at(i) = rand_generator.generate_random_uniform_vec3(0, k_world_edge_size);
        }
    }
};

#endif // PARTICLE_HPP_