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
    std::vector<glm::vec3> velocity_;
    std::vector<glm::vec3> acceleration_;
    std::vector<glm::vec3> position_;
    std::vector<glm::vec3> force_;
    std::vector<glm::vec3> density_;

    std::vector<glm::vec3> gl_position_;
    std::vector<glm::vec3> color_;

    RandGenerator rand_generator;

public:
    Particle() 
    {
        position_.resize(k_num_particle);
        velocity_.resize(k_num_particle);
        acceleration_.resize(k_num_particle);
        force_.resize(k_num_particle);
        density_.resize(k_num_particle);

        gl_position_.resize(k_num_particle);
        color_.resize(k_num_particle, {1.0f, 1.0f, 1.0f});

        initialize_particle_state();
    };

    inline std::vector<glm::vec3> &get_particle_position() { return position_; }
    inline std::vector<glm::vec3> &get_particle_velocity() { return velocity_; }
    inline std::vector<glm::vec3> &get_particle_acceleration() { return acceleration_; }
    inline std::vector<glm::vec3> &get_particle_force() { return force_; }
    inline std::vector<glm::vec3> &get_particle_density() { return density_; }

    inline void set_particle_position(std::vector<glm::vec3> &new_position) { position_ = new_position; }
    inline void set_particle_velocity(std::vector<glm::vec3> &new_velocity) { velocity_ = new_velocity; }
    inline void set_particle_acceleration(std::vector<glm::vec3> &new_acceleration) { acceleration_ = new_acceleration; }

    std::vector<glm::vec3> &get_gl_particle_position() 
    { 
        #pragma omp parallel for
        for (int i = 0; i < k_num_particle; i++)
        {
            gl_position_.at(i) = transform_world2gl(position_[i]);
        }
        return gl_position_;
    }
    std::vector<glm::vec3> &get_particle_color() { return color_; }

    ~Particle() {};

private:
    void initialize_particle_state()
    {
        #pragma omp parallel for
        for (int i = 0; i < k_num_particle; i++)
        {
            color_.at(i) = {153/255, 255/255, 255/255};
            position_.at(i) = rand_generator.generate_random_uniform_vec3(0, k_world_edge_size);
            velocity_.at(i) = {0.0f, 0.0f, 0.0f};
        }
    }
};

#endif // PARTICLE_HPP_