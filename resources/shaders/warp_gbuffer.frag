/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

@include "common/header.glsl"
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "gbuffer_warp_modes.glsl"

// -----------------------------------------------------------------------------
#if WARP_MODE == WARP_MODE_GRID || WARP_MODE == WARP_MODE_ADAPTIVE_GRID // -----
// -----------------------------------------------------------------------------

flat in uint cellsize;
in vec2 texcoords;

// output
layout(location=0) out vec3 gua_out_color;

void main() {
  #if @debug_cell_colors@ == 1
    float intensity = log2(cellsize) / 5.0;
    gua_out_color =  0.6*(vec3(1, 0.1, 0.1) * (1-intensity) + vec3(0.1, 1, 0.1) * intensity);
  #else
    gua_out_color = gua_get_color(texcoords);
  #endif
}


// -----------------------------------------------------------------------------
#else // all other modes -------------------------------------------------------
// -----------------------------------------------------------------------------

in vec3 color;
in vec3 normal;

// output
layout(location=0) out vec3 gua_out_color;

void main() {
  gua_out_color = color;
}

#endif
