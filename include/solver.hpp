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
        rigid_hexahedron.get_curr_state().x += glm::vec3({0, 0, 0});
    }

    void compute_next_state_im_euler()
    {

    }

    void compute_next_state_rk2()
    {

    }
};

#endif // SOLVER_H_