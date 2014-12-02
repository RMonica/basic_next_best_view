/*
 * Copyright (c) 2013-2014, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BASIC_NEXT_BEST_VIEW_H
#define BASIC_NEXT_BEST_VIEW_H

// input topics
#define PARAM_NAME_OCCUPANCY_TOPIC      "occupancy_topic"
#define PARAM_DEFAULT_OCCUPANCY_TOPIC   "/basic_next_best_view/occupancy"

#define PARAM_NAME_POI_TOPIC            "poi_topic"
#define PARAM_DEFAULT_POI_TOPIC         "/basic_next_best_view/poi"

// this allows to override the candidate generation
#define PARAM_NAME_CANDIDATES_TOPIC     "candidates_topic"
#define PARAM_DEFAULT_CANDIDATES_TOPIC  "/basic_next_best_view/candidates"

#define PARAM_NAME_VOXEL_FILTER_TOPIC    "voxel_filter_topic"
#define PARAM_DEFAULT_VOXEL_FILTER_TOPIC "/basic_next_best_view/voxel_filter"

#define PARAM_NAME_EXECUTE_ACTION       "execute_action"
#define PARAM_DEFAULT_EXECUTE_ACTION    "/basic_next_best_view/execute"

// output topics
#define PARAM_NAME_ACK_TOPIC            "ack_topic"
#define PARAM_DEFAULT_ACK_TOPIC         "/basic_next_best_view/ack"

// parameters
#define PARAM_NAME_VOXEL_RESOLUTION     "voxel_resolution"
#define PARAM_DEFAULT_VOXEL_RESOLUTION  ((double)(0.02))

#define PARAM_NAME_SENSOR_RANGE_MAX     "sensor_range_max"
#define PARAM_DEFAULT_SENSOR_RANGE_MAX  ((double)(1.0))

#define PARAM_NAME_SPHERE_RADIUS        "sphere_radius"
#define PARAM_DEFAULT_SPHERE_RADIUS     ((double)(1.0))

  // sensor horizontal resolution
#define PARAM_NAME_HORIZONTAL_RES       "horizontal_resolution"
#define PARAM_DEFAULT_HORIZONTAL_RES    ((int)(30))
  // sensor vertical resolution
#define PARAM_NAME_VERTICAL_RES         "vertical_resolution"
#define PARAM_DEFAULT_VERTICAL_RES      ((int)(30))
  // horizontal half field of view, in degrees
#define PARAM_NAME_HALF_FOV_HOR         "half_fov_horizontal"
#define PARAM_DEFAULT_HALF_FOV_HOR      ((double)(30.0))
  // vertical half field of view, in degrees
#define PARAM_NAME_HALF_FOV_VER         "half_fov_vertical"
#define PARAM_DEFAULT_HALF_FOV_VER      ((double)(30.0))

// latitude, longitude and z-rotation sampling
#define PARAM_NAME_LATITUDE_MAX         "latitude_max"
#define PARAM_DEFAULT_LATITUDE_MAX      ((double)(60.0))

#define PARAM_NAME_LATITUDE_MIN         "latitude_min"
#define PARAM_DEFAULT_LATITUDE_MIN      ((double)(0.0))

#define PARAM_NAME_LATITUDE_STEP        "latitude_step"
#define PARAM_DEFAULT_LATITUDE_STEP     ((double)(10.0))

#define PARAM_NAME_LONGITUDE_MAX         "longitude_max"
#define PARAM_DEFAULT_LONGITUDE_MAX      ((double)(330.0))

#define PARAM_NAME_LONGITUDE_MIN         "longitude_min"
#define PARAM_DEFAULT_LONGITUDE_MIN      ((double)(0.0))

#define PARAM_NAME_LONGITUDE_STEP        "longitude_step"
#define PARAM_DEFAULT_LONGITUDE_STEP     ((double)(30.0))

#define PARAM_NAME_ZROTATION_MAX         "zrotation_max"
#define PARAM_DEFAULT_ZROTATION_MAX      ((double)(330.0))

#define PARAM_NAME_ZROTATION_MIN         "zrotation_min"
#define PARAM_DEFAULT_ZROTATION_MIN      ((double)(0.0))

#define PARAM_NAME_ZROTATION_STEP        "zrotation_step"
#define PARAM_DEFAULT_ZROTATION_STEP     ((double)(30.0))

#define PARAM_NAME_OCCUPIED_WEIGHT       "occupied_voxel_weight"
#define PARAM_DEFAULT_OCCUPIED_WEIGHT    ((double)(0.0))

#define PARAM_NAME_UNKNOWN_WEIGHT        "unknown_voxel_weight"
#define PARAM_DEFAULT_UNKNOWN_WEIGHT     ((double)(1.0))

#define PARAM_NAME_WORLD_FRAME           "world_frame"
#define PARAM_DEFAULT_WORLD_FRAME        "/world_frame"

#endif // BASIC_NEXT_BEST_VIEW_H
