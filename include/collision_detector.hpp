#ifndef COLLISION_DETECTOR_H_
#define COLLISION_DETECTOR_H_

#include <iostream>

#include <glm/glm.hpp>

#include "common.hpp"

class Collision_Detector
{
    struct edge
    {
        glm::vec3 p;
        glm::vec3 e;
    };

    struct plain
    {
        glm::vec3 p;
        glm::vec3 n;
    };

private:
    edge tetrahedron_edge;
    plain ground_plain;
    
    std::vector<glm::vec3> curr_vertices;
    std::vector<glm::vec3> next_vertices;

public: 
    Collision_Detector() 
    {
        tetrahedron_edge = 
        {
            transform_gl2phy({-0.1f, 0.1f, -0.2f}),
            transform_gl2phy({ 0.1f, 0.5f, 0.4f}) - transform_gl2phy({-0.1f, 0.1f, -0.2f})
        };

        ground_plain = 
        {
            transform_gl2phy({0.0f, 0.0f, -1.0f}),
            {0.0f, 0.0f, 1.0f}
        };

        curr_vertices.resize(8);
        next_vertices.resize(8);
    };

    collision_result detect_collision(state &curr_state, state &next_state, std::vector<glm::vec3> &vertices)
    {
        int i;
        glm::mat3 curr_R = glm::toMat3(curr_state.q);
        glm::mat3 next_R = glm::toMat3(next_state.q);

        #pragma parallel for private(i) num_threads(4)
        for (i = 0; i < vertices.size(); i++)
        {
            curr_vertices[i] = (curr_state.x + curr_R * vertices[i]);
            next_vertices[i] = (next_state.x + next_R * vertices[i]);
        }

        // edge-edge collision detection: p.205
        collision_result edge_edge_result = detect_edge_edge_collision(curr_state.x);
        return edge_edge_result;

        // collision_result vertex_face_result = detect_vertex_face_collision(curr_state.x);
        // return vertex_face_result;
        // vertex-face
        // return {NULL, NULL, NULL};
    }

    ~Collision_Detector() {};

private:
    collision_result detect_vertex_face_collision(glm::vec3 com_position)
    {
        int i;
        std::vector<collision_result> collision_results;
        #pragma parallel for private(i) num_threads(4)
        for (i = 0; i < curr_vertices.size(); i++)
        {
            if ((std::signbit(glm::dot(curr_vertices[i] - ground_plain.p, ground_plain.n)) != std::signbit(glm::dot(next_vertices[i] - ground_plain.p, ground_plain.n))))
            {
                collision_results.push_back(
                {
                    curr_vertices[i] - com_position, // p_a - x = r_a
                    ground_plain.n
                });
            }
            else return k_null_collision_result;
        }
        // if (collision_results.size() != 0)
        // {
        //     return collision_results;
        // }
    }

    collision_result detect_edge_edge_collision(glm::vec3 com_position)
    {
        edge curr_edge = 
        {
            curr_vertices[2],
            curr_vertices[3] - curr_vertices[2]
        };

        edge next_edge = 
        {
            next_vertices[2],
            next_vertices[3] - next_vertices[2]
        };

        glm::vec3 n = glm::normalize(glm::cross(curr_edge.e, tetrahedron_edge.e));
        glm::vec3 r = tetrahedron_edge.p - curr_edge.p;

        // barycentric coord test
        float s = (glm::dot(r, glm::normalize(glm::cross(tetrahedron_edge.e, n)))) / (glm::dot(curr_edge.e, glm::normalize(glm::cross(tetrahedron_edge.e, n))));
        float t = -(glm::dot(r, glm::normalize(glm::cross(curr_edge.e, n)))) / (glm::dot(tetrahedron_edge.e, glm::normalize(glm::cross(curr_edge.e, n))));
        

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1 && 
            (std::signbit(glm::dot(next_edge.p - tetrahedron_edge.p, n)) != std::signbit(glm::dot(curr_edge.p - tetrahedron_edge.p, n))))
        {
            return
            {
                (curr_edge.p + s * curr_edge.e) - com_position, // p_a - x = r_a
                n
            };
        } 
        else return k_null_collision_result;  
    }
};

#endif // COLLISION_DETECTOR_H_