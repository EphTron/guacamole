@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

uniform bool enable_backface_culling;
uniform mat4 inverse_transpose_model_view_matrix;

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VertexData {
  //input from geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexIn[];

out VertexData {
  vec2 pass_uv_coords;
  //float pass_log_depth;
  float pass_es_linear_depth;
  float pass_es_shift;
  vec3 pass_world_position;
} VertexOut;



void main() {

  vec4 object_normal = vec4(normalize(VertexIn[0].pass_normal), 0.0);
  vec4 view_normal  = inverse_transpose_model_view_matrix * object_normal;

  if(enable_backface_culling == false || view_normal.z > 0.0 ) {

      // --------------------------- common attributes -----------------------------------

      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].pass_ms_u,
                              VertexIn[0].pass_ms_v);

      float es_linear_depth_center = (gua_model_view_matrix * vec4(step_uv[0],1.0)).z;
      float es_shift = 0.0;
      float es_shift_scale = 2.0;

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        VertexOut.pass_uv_coords        = uv_multiplier.yz;
        vec4 q_pos_ms                   = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position                     = gua_model_view_projection_matrix * q_pos_ms;
        VertexOut.pass_world_position = (gua_model_matrix * q_pos_ms).xyz;
        float es_linear_depth_corner = (gua_model_view_matrix * q_pos_ms).z;

        es_shift       = abs(es_linear_depth_corner - es_linear_depth_center) * es_shift_scale;
        gl_Position.z  = ( ( -(es_linear_depth_corner + es_shift ) ) / gua_clip_far);
        gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
        gl_Position.z  *= gl_Position.w;
        
        VertexOut.pass_es_linear_depth = (-es_linear_depth_corner) / gua_clip_far;

        VertexOut.pass_es_shift = es_shift;
        EmitVertex();
      }

      EndPrimitive();
  }

}

