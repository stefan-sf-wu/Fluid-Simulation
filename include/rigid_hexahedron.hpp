#ifndef RIGID_HEXAHEDRON_HPP_
#define RIGID_HEXAHEDRON_HPP_

#include <vector>
#include <cmath>

#include <glm/glm.hpp>
#include <omp.h>

#include "common.hpp"

class Rigid_Hexahedron
{
private:
    state curr_state;
    // state curr_state_dt;
    
    std::vector<glm::vec3> vertices; // relative position of vertices of hexahedron to com 

    glm::mat3 I_0;                   // moment of inertia tensor for hexahedron

    std::vector<glm::vec3> mesh_vertices;
    std::vector<unsigned int> mesh_indices;
    std::vector<glm::vec3> mesh_color;

public:
    Rigid_Hexahedron()
    {
        float w = k_world_edge_size/6;
        float l = k_world_edge_size/3; 
        float h = k_world_edge_size/7;
        build_vertices(w, l, h);
        build_moment_of_inertia_matrix(w, l, h);

        build_mesh();

        // initialize the state
        curr_state.x = k_hexahedron_init_position;
        curr_state.R = k_hexahedron_init_rotation_matrix;
        curr_state.P = k_hexahedron_init_velocity * k_hexahedron_mass;
        curr_state.L = I_0 * k_hexahedron_init_angular_velocity;
                
    };

    void update_mesh_vertices()
    {
        int i;
        #pragma parallel for private(i)
        for(i = 0; i < vertices.size(); i++) { mesh_vertices[i] = transform_phy2gl(curr_state.x + curr_state.R * vertices[i]); }
    }

    std::vector<glm::vec3> &get_vertices() { return mesh_vertices; }
    std::vector<unsigned int> &get_indices() { return mesh_indices; }
    std::vector<glm::vec3> &get_color() { return mesh_color; }
    glm::mat3 &get_moment_of_inertia() { return I_0; }
    state &get_curr_state() { return curr_state; }

    void set_curr_state(state st) { curr_state = st; }

    ~Rigid_Hexahedron() 
    {
    };

private:
void build_moment_of_inertia_matrix(float w, float l, float h)
{
    I_0 = 
    {
        {k_hexahedron_mass / 12 * (w * w + h * h), 0.0f, 0.0f},
        {0.0f, k_hexahedron_mass / 12 * (w * w + l * l), 0.0f},
        {0.0f, 0.0f, k_hexahedron_mass / 12 * (l * l + h * h)},
    };
}
void build_mesh()
{
    mesh_vertices.resize(vertices.size()); 

    mesh_indices.insert(mesh_indices.end(), {0, 4, 1});
    mesh_indices.insert(mesh_indices.end(), {1, 5, 3});
    mesh_indices.insert(mesh_indices.end(), {3, 7, 2});
    mesh_indices.insert(mesh_indices.end(), {2, 6, 0});
    mesh_indices.insert(mesh_indices.end(), {4, 5, 1});
    mesh_indices.insert(mesh_indices.end(), {5, 7, 3});
    mesh_indices.insert(mesh_indices.end(), {7, 6, 2});
    mesh_indices.insert(mesh_indices.end(), {6, 4, 0});
    mesh_indices.insert(mesh_indices.end(), {4, 5, 6});
    mesh_indices.insert(mesh_indices.end(), {5, 6, 7});
    mesh_indices.insert(mesh_indices.end(), {0, 1, 2});
    mesh_indices.insert(mesh_indices.end(), {1, 2, 3});

    for(int i = 0; i < mesh_vertices.size(); i++) { mesh_color.push_back(k_hexahedron_mesh_color); }
}

void build_vertices(float w, float l, float h)
{
    w /= 2; l /= 2; h /= 2;
    vertices.push_back({-w, -l, -h});
    vertices.push_back({+w, -l, -h});
    vertices.push_back({-w, +l, -h});
    vertices.push_back({+w, +l, -h});
    vertices.push_back({-w, -l, +h});
    vertices.push_back({+w, -l, +h});
    vertices.push_back({-w, +l, +h});
    vertices.push_back({+w, +l, +h});
}
};



#endif // RIGID_HEXAHEDRON_HPP_
