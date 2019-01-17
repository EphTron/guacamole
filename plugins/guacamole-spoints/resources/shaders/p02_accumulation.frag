@include "shaders/common/header.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/pack_vec3.glsl"


//in vec3 pass_point_color;

in VertexDataOut {
  vec3 color;
  float log_depth;
  vec2 uv_coords;
} FragmentIn;


layout (location = 0) out vec3 out_accumulated_color;
layout (location = 3) out vec2 out_accumulated_weight_and_depth;

uniform float voxel_half_size = 0.0;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////

const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

const float sqrt_of_two = 1.4142135623730951;
void main() {

  vec2 uv_coords = FragmentIn.uv_coords;
  float blend_weight = 1.0;

  float fragment_position_length = dot(uv_coords, uv_coords);
  if( fragment_position_length >= 1.0) 
    discard;
  else
    blend_weight = 0.001 + 1.0 - fragment_position_length;//gaussian[(int)(round(length(uv_coords) * 31.0))];


/*
  vec2 centered_point_coord = (gl_PointCoord.xy - 0.5) * 2.0;
  vec2 max_abs_center_coord = abs(centered_point_coord);
  float blend_weight = sqrt_of_two - max(max_abs_center_coord.x, max_abs_center_coord.y );
*/
  //loat dist_to_center = (length( FragmentIn.ms_curr_pos - FragmentIn.ms_center_pos) ); // / length( vec3(voxel_half_size, voxel_half_size, voxel_half_size) );
  //float blend_weight = gaussian[(int)(round(length(uv_coords) * 31.0))];
  //blend_weight = gaussian[min(31, max(0,(int)(blend_weight * 15.5)))];

  //gl_FragDepth = (gl_FragCoord.z * gua_clip_far) / gua_clip_far;

  out_accumulated_color = vec3(blend_weight * FragmentIn.color);

  out_accumulated_weight_and_depth = vec2(blend_weight, blend_weight * FragmentIn.log_depth);

}
