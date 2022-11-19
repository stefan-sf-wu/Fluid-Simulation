#ifndef SOLVER_H_
#define SOLVER_H_

#include <vector>
#include <iostream>

#include <glm/glm.hpp>
#include <omp.h>

#include "common.hpp"
#include "rigid_hexahedron.hpp"
#include "collision_handler.hpp"

class Solver
{
private:
    Rigid_Hexahedron rigid_hexahedron;
    Collision_Handler collision_handler;

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

    std::vector<glm::vec3> &get_hexahedron_vertices()
    {
        rigid_hexahedron.update_mesh_vertices();
        return rigid_hexahedron.get_vertices();
    }

    std::vector<unsigned int> &get_hexahedron_indices() 
    { 
        return rigid_hexahedron.get_indices(); 
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
        state curr_state = rigid_hexahedron.get_curr_state();
        state_dt curr_state_dt = F(curr_state, k_time_step);
        curr_state += curr_state_dt * k_time_step;
        curr_state.R = normalize_matrix(curr_state.R);
        rigid_hexahedron.set_curr_state(curr_state);
    }

    void compute_next_state_im_euler()
    {

    }

    void compute_next_state_rk2()
    {

    }

    state_dt F(state st, float h)
    {
        state_dt st_dt;

        st_dt.v = st.P / k_hexahedron_mass;

        glm::mat3 moment_of_inertia_inverse = st.R * glm::inverse(rigid_hexahedron.get_moment_of_inertia()) * glm::transpose(st.R);
        glm::vec3 omega = moment_of_inertia_inverse * st.L;
        glm::mat3 omega_star =
        {
            {0.0f, -omega[2], omega[1]}, 
            {omega[2], 0.0f, -omega[0]},
            {-omega[1], omega[0], 0.0f}
        };
        st_dt.R_dt = omega_star * st.R;

        st_dt.P_dt = glm::vec3({0, 0, 0}); // sum of forces
        st_dt.L_dt = glm::vec3({0, 0, 0}); // sum of torques

        return st_dt;
    }
};

#endif // SOLVER_H_