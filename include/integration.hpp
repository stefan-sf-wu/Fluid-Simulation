#ifndef INTEGRATION_H_
#define INTEGRATION_H_

#include <glm/glm.hpp>

#include "common.hpp"

namespace Integration
{

inline glm::vec3 euler_integrator(vertex &v, glm::vec3 acc)
{
    v.velocity += k_time_step * acc;
    v.position += k_time_step * v.velocity;
}




}


#endif // INTEGRATION_H_