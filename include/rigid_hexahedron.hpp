#ifndef RIGID_HEXAHEDRON_HPP_
#define RIGID_HEXAHEDRON_HPP_

#include <vector>

#include <glm/glm.hpp>
#include <omp.h>

#include "common.hpp"

class Rigid_Hexahedron
{
private:
    state curr_state;
    state curr_state_dt;
    
    std::vector<glm::vec3> vertices;        // relative position of vertices of hexahedron to com 

    std::vector<glm::vec3> mesh_vertices;
    std::vector<unsigned int> mesh_indices;

public:
    Rigid_Hexahedron()
    {
        curr_state.x = {0.0f, 0.0f, k_world_edge_size * 0.8};

        /** TODO: tentative rotation matrix **/
        curr_state.R = glm::mat3({1, 0, 0}, {0, 1, 0}, {0, 0, 1});

        float w = k_world_edge_size/5;
        float h = k_world_edge_size/6; 
        float c = k_world_edge_size/7;

        build_rigid_hexahedron(w, h, c);

        mesh_vertices.resize(vertices.size());
        update_mesh_vertices();
    };

    void update_mesh_vertices()
    {
        int i;
        #pragma parallel for private(i)
        for(i = 0; i < vertices.size(); i++) { mesh_vertices[i] = transform_phy2gl(curr_state.x + curr_state.R * vertices[i]); }
    }

    std::vector<glm::vec3> &get_vertices() { return mesh_vertices; }

    std::vector<unsigned int> &get_indices() { return mesh_indices; }

    state &get_curr_state() { return curr_state; }

    void set_curr_state(state st) { curr_state = st; }

    ~Rigid_Hexahedron() 
    {
    };

private:
void build_rigid_hexahedron(float w, float h, float c)
{
    // simulation world
    w /= 2; h /= 2; c /= 2;
    vertices.push_back({-w, -h, -c});
    vertices.push_back({+w, -h, -c});
    vertices.push_back({-w, +h, -c});
    vertices.push_back({+w, +h, -c});
    vertices.push_back({-w, -h, +c});
    vertices.push_back({+w, -h, +c});
    vertices.push_back({-w, +h, +c});
    vertices.push_back({+w, +h, +c});

    // opengl
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
}
};



#endif // RIGID_HEXAHEDRON_HPP_
