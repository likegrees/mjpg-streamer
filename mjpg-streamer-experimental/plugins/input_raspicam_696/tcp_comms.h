/*
 * Copyright (C) 2017 by Daniel Clouse.
 *
 * This file is dual licensed: you can use it either under the terms of
 * the GPL, or the BSD license, at your option.
 *
 *  a) This library is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of the
 *     License, or (at your option) any later version.
 *
 *     This library is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public
 *     License along with this library; if not, write to the Free
 *     Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 *     MA 02110-1301 USA
 *
 * Alternatively,
 *
 *  b) Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *     1. Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *     2. Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *     CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *     INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *     CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *     SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *     NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *     OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *     EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "RaspiCamControl.h"

typedef struct {
    int fd;
    struct sockaddr_storage saddr;
    socklen_t saddr_len;
    pthread_t pthread_id;
} Tcp_Host_Info;

typedef struct {
    Tcp_Host_Info server;
    int client_count;
    Tcp_Host_Info* client;
    pthread_mutex_t lock_mutex;           /// mutual exclusion lock

    // Camera state
    MMAL_COMPONENT_T camera_ptr;
    RASPICAM_CAMERA_PARAMETERS cam_params;
    bool test_image_enable;
    unsigned char detect_yuv_min[3];
    unsigned char detect_yuv_max[3];
} Tcp_Comms;

#define RASPICAM_SATURATION              1
#define RASPICAM_SHARPNESS               2
#define RASPICAM_CONTRAST                3
#define RASPICAM_BRIGHTNESS              4
#define RASPICAM_ISO                     5
#define RASPICAM_METERING_MODE           6
#define RASPICAM_VIDEO_STABILISATION     7
#define RASPICAM_EXPOSURE_COMPENSATION   8
#define RASPICAM_EXPOSURE_MODE           9
#define RASPICAM_AWB_MODE               10
#define RASPICAM_AWB_GAINS              11
#define RASPICAM_IMAGE_FX               12
#define RASPICAM_COLOUR_FX              13
#define RASPICAM_ROTATION               14
#define RASPICAM_FLIPS                  15
#define RASPICAM_ROI                    16
#define RASPICAM_SHUTTER_SPEED          17
#define RASPICAM_DRC                    18
#define RASPICAM_STATS_PASS             19
//#define RASPICAM_ANNOTATE               20
//#define RASPICAM_STEREO_MODE            21
#define RASPICAM_TEST_IMAGE_ENABLE      22
#define RASPICAM_DETECT_YUV             23

typedef struct {
    unsigned char tag;
    unsigned char c[12];
} Raspicam_Char_Msg;

typedef struct {
    unsigned char tag;
    unsigned char filler[3];
    int int0;
    int int1;
    int int2;
} Raspicam_Int_Msg;

typedef struct {
    unsigned char tag;
    unsigned char filler[3];
    float float0;
    float float1;
    float float2;
    float float3;
} Raspicam_Float_Msg;

int tcp_comms_construct(Tcp_Comms* comms_ptr,
                        unsigned short port_number);
