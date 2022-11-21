#ifndef GROUND_MESH_H
#define GROUND_MESH_H

#include <vector>
#include <iostream>

#include <glm/glm.hpp>

namespace GLObj
{

const float slices = 20;
const float ground_mesh_edge_size = 4.0f;
const glm::vec3 ground_mesh_color = {1.0f, 1.0f, 1.0f};

std::vector<glm::vec3> ground_mesh_vertices;
std::vector<glm::uvec4> ground_mesh_indices;

void build_ground_mesh()
{
    float x, y, z;
    int i, j, row1, row2;
    for (j = 0; j <= slices; ++j)
    {
        for (i = 0; i <= slices; ++i)
        {
            x = ground_mesh_edge_size * (i / slices) - (ground_mesh_edge_size / 2);
            y = ground_mesh_edge_size * (j / slices) - (ground_mesh_edge_size / 2);
            z = -1.0f;
            ground_mesh_vertices.push_back(glm::vec3(x, y, z));
        }
    }

    for (j = 0; j < slices; ++j) 
    {
        for (i = 0; i < slices; ++i) 
        {
            row1 =    j  * (slices+1);
            row2 = (j+1) * (slices+1);

            ground_mesh_indices.push_back(glm::uvec4(row1+i,    row1+i+1,   row1+i+1,   row2+i+1));
            ground_mesh_indices.push_back(glm::uvec4(row2+i+1,  row2+i,     row2+i,     row1+i));
        }
    }
}

} // namespace GLObj

#endif // GROUND_MESH_H


// ref: https://stackoverflow.com/questions/58494179/how-to-create-a-grid-in-opengl-and-drawing-it-with-lines
