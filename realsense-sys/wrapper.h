#include "librealsense2/rsutil.h"

void _rs2_project_point_to_pixel(float pixel[2], const struct rs2_intrinsics * intrin, const float point[3])
{
    rs2_project_point_to_pixel(pixel, intrin, point);
}

void _rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
{
    rs2_deproject_pixel_to_point(point, intrin, pixel, depth);
}

void _rs2_transform_point_to_point(float to_point[3], const struct rs2_extrinsics * extrin, const float from_point[3])
{
    rs2_transform_point_to_point(to_point, extrin, from_point);
}

void _rs2_fov(const struct rs2_intrinsics * intrin, float to_fov[2])
{
    rs2_fov(intrin, to_fov);
}

void _next_pixel_in_line(float curr[2], const float start[2], const float end[2])
{
    next_pixel_in_line(curr, start, end);
}

bool _is_pixel_in_line(const float curr[2], const float start[2], const float end[2])
{
    return is_pixel_in_line(curr, start, end);
}

void _adjust_2D_point_to_boundary(float p[2], int width, int height)
{
    adjust_2D_point_to_boundary(p, width, height);
}

void _rs2_project_color_pixel_to_depth_pixel(float to_pixel[2],
    const uint16_t* data, float depth_scale,
    float depth_min, float depth_max,
    const struct rs2_intrinsics* depth_intrin,
    const struct rs2_intrinsics* color_intrin,
    const struct rs2_extrinsics* color_to_depth,
    const struct rs2_extrinsics* depth_to_color,
    const float from_pixel[2])
{
    rs2_project_color_pixel_to_depth_pixel(to_pixel, data, depth_scale, depth_min, depth_max, depth_intrin, color_intrin, color_to_depth, depth_to_color, from_pixel);
}
