#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>

namespace math
{

const float sph_kernal_s = 2.0f;

inline float sph_kernel_function(float sph_kernel_r)
{
    sph_kernel_r = std::abs(sph_kernel_r);
    
    float rs_ratio = sph_kernel_r / sph_kernal_s;
    float ret = (1 / (M_PI * std::pow(sph_kernal_s, 3)));

    if (rs_ratio < 0 || rs_ratio > 2) 
    {
        return ret * 0.0f;
    } 
    else if (0 <= rs_ratio && rs_ratio <= 1)
    {
        return ret * (1 - (3/2) * std::pow(rs_ratio, 2) + (3/4) * std::pow(rs_ratio, 3));
    } 
    else if (1 <= rs_ratio && rs_ratio <= 2)
    {
        return ret * (1/4) * std::pow((2 - rs_ratio), 3);
    }
    return -1;
}

} // namespace math

#endif // MATH_HPP_