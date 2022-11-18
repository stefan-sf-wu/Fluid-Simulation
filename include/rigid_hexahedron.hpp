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
    std::vector<glm::vec3> mesh_color;

public:
    Rigid_Hexahedron()
    {
        curr_state.x = {k_world_edge_size * 0.5f, k_world_edge_size * 0.5f, k_world_edge_size * 0.8f};

        /** TODO: tentative rotation matrix **/
        curr_state.R = glm::mat3({1, 0, 0}, {0, 1, 0}, {0, 0, 1});

        float w = k_world_edge_size/6;
        float h = k_world_edge_size/3; 
        float c = k_world_edge_size/7;
        build_rigid_hexahedron_vertices(w, h, c);

        build_rigid_hexahedron_mesh();
    };

    void update_mesh_vertices()
    {
        int i;
        #pragma parallel for private(i)
        for(i = 0; i < vertices.size(); i++) { mesh_vertices[i] = transform_phy2gl(curr_state.x + curr_state.R * vertices[i]); }

        print_vec(curr_state.x, "cuur_state.x");
        print_vec(transform_phy2gl(curr_state.x), "transformed");
    }

    std::vector<glm::vec3> &get_vertices() { return mesh_vertices; }

    std::vector<unsigned int> &get_indices() { return mesh_indices; }

    std::vector<glm::vec3> &get_color() { return mesh_color; }

    state &get_curr_state() { return curr_state; }

    void set_curr_state(state st) { curr_state = st; }

    ~Rigid_Hexahedron() 
    {
    };

private:
void build_rigid_hexahedron_mesh()
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

void build_rigid_hexahedron_vertices(float w, float h, float c)
{
    w /= 2; h /= 2; c /= 2;
    vertices.push_back({-w, -h, -c});
    vertices.push_back({+w, -h, -c});
    vertices.push_back({-w, +h, -c});
    vertices.push_back({+w, +h, -c});
    vertices.push_back({-w, -h, +c});
    vertices.push_back({+w, -h, +c});
    vertices.push_back({-w, +h, +c});
    vertices.push_back({+w, +h, +c});
}
};



#endif // RIGID_HEXAHEDRON_HPP_
