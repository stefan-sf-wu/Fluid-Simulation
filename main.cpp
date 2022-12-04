#include <iostream>
#include <vector>

#include "glm/glm.hpp"
#include <cyCodeBase/cyPointCloud.h>

#include "renderer.hpp"
#include "common.hpp"


// void compute_density_callback(unsigned int index, glm::vec3 const &p, float distanceSquared, float &radiusSquared)
// {
//     std::cout << index << "\n";
// }

int main() 
{
    Renderer renderer;
    renderer.initialize();
    renderer.start_looping();

    // glm::vec3 ptr[3] = {
    //     {0, 0, 0},
    //     {1, 1, 1},
    //     {1.1, 1, 1}
    // };

    // cy::PointCloud<glm::vec3, float, 3> kdtree;
    // kdtree.Build(3, ptr);
    // kdtree.GetPoints(glm::vec3{1, 1, 1}, 0.2, compute_density_callback);


	return 0;
}


/**
 *    (\_/)
 *    ( •_•)  
 *    / > ""
 * 
 */