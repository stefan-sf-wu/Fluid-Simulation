#ifndef SOLVER_H_
#define SOLVER_H_
#ifndef GLM_ENABLE_EXPERIMENTAL
    #define GLM_ENABLE_EXPERIMENTAL
#endif

#include <vector>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <omp.h>

#include "common.hpp"
#include "rigid_hexahedron.hpp"
#include "collision_detector.hpp"

class Solver
{
private:
    Rigid_Hexahedron rigid_hexahedron;
    Collision_Detector collision_detector;

public: 
    Solver()
    {
    };

    void compute_next_state()
    {
        switch (integration_method)
        {
            case integrator::im_euler:
                compute_next_state_im_euler();
                break;
            case integrator::ex_euler:
                compute_next_state_ex_euler();
                break;
            case integrator::rk2:
                compute_next_state_rk2();
            default:
                break;
        }
    }

    bool is_resting()
    {
        return false;
    }

    std::vector<glm::vec3> &get_hexahedron_vertices()
    {
        rigid_hexahedron.update_mesh_vertices();
        return rigid_hexahedron.get_mesh_vertices();
    }

    std::vector<unsigned int> &get_hexahedron_indices() 
    { 
        return rigid_hexahedron.get_mesh_indices(); 
    }

    std::vector<glm::vec3> &get_hexahedron_color()
    {
        return rigid_hexahedron.get_color();
    }

    ~Solver()
    {
    };

private:
    void compute_next_state_ex_euler()
    {
        state curr_state, next_state;
        state_d curr_state_d;

        curr_state = rigid_hexahedron.get_curr_state();
        curr_state_d = F(curr_state);
        next_state = (curr_state + curr_state_d * k_time_step);

        // collision
        std::vector<collision_result> result = collision_detector.detect_collision(curr_state, next_state, rigid_hexahedron.get_vertices());
        if(!result.empty())
        {
            next_state = handle_collision(curr_state, curr_state_d, result);
        }
        
        // update
        next_state.q = glm::normalize(next_state.q);
        rigid_hexahedron.set_curr_state(next_state);
    }


    void compute_next_state_im_euler()
    {
    }

    void compute_next_state_rk2()
    {
        state curr_state, next_state;
        state_d k1, k2;
        
        curr_state = rigid_hexahedron.get_curr_state();
        k1 = F(curr_state);

        // // collision detection with k1
        // next_state = (curr_state + k1 * k_time_step);
        
        k2 = F(curr_state + k1 * 0.5f * k_time_step);
        next_state = (curr_state + k2 * k_time_step);
        
        std::vector<collision_result> result = collision_detector.detect_collision(curr_state, next_state, rigid_hexahedron.get_vertices());
        if(!result.empty())
        {
            next_state = handle_collision(curr_state, k2, result);
        }

        next_state.q = glm::normalize(next_state.q);
        rigid_hexahedron.set_curr_state(next_state);
    }


    state handle_collision(state curr_state, state_d curr_state_d, std::vector<collision_result> result)
    {
        glm::mat3 R = glm::toMat3(curr_state.q);
        glm::mat3 moment_of_inertia_inverse = R * glm::inverse(rigid_hexahedron.get_moment_of_inertia()) * glm::transpose(R);
        glm::vec3 omega = moment_of_inertia_inverse * curr_state.L;

        glm::vec3 delta_P = {0.0f, 0.0f, 0.0f};
        glm::vec3 delta_L = {0.0f, 0.0f, 0.0f};
        #pragma parallel for reduction(+: delta_P, delta_L)
        for (auto res : result)
        {
            float v_in  = glm::dot((curr_state_d.v + glm::cross(omega, res.r_a)), res.n);
            float j = (-(1 + k_restitution) * v_in)
                / ((1/k_hexahedron_mass) + glm::dot(res.n, (moment_of_inertia_inverse * glm::cross(glm::cross(res.r_a, res.n), res.r_a))));
            glm::vec3 J = j * res.n;
            delta_P += J;
            delta_L += glm::cross(res.r_a, J);
        }

        return 
        {
            curr_state.x,
            curr_state.q,
            curr_state.P += delta_P,
            curr_state.L += delta_L
        };
    }

    state_d F(state st)
    {
        state_d st_dt;

        st_dt.v = st.P / k_hexahedron_mass;

        glm::mat3 R = glm::toMat3(st.q);

        glm::mat3 moment_of_inertia_inverse = R * glm::inverse(rigid_hexahedron.get_moment_of_inertia()) * glm::transpose(R);
        glm::vec3 omega = moment_of_inertia_inverse * st.L;
        
        st_dt.q_d = 0.5f * glm::quat(0, omega) * st.q;

        st_dt.P_d = compute_body_force() + compute_point_force(); // sum of forces
        st_dt.L_d = compute_torque(); // sum of torques

        return st_dt;
    }

    glm::vec3 compute_body_force()
    {
        return k_hexahedron_mass * k_gravity;   // only gravity here for now
    }

    glm::vec3 compute_point_force()
    {
        return {0, 0, 0};
    }

    glm::vec3 compute_torque()
    {
        return {0, 0, 0};
    }
};

#endif // SOLVER_H_