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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "tcp_comms.h"
#include "get_ip_addr_str.h"


static inline int_limit(int low, int high, int value) {
    return (value < low) ? low : (value > high) ? high : value;
}

static inline float_limit(float low, float high, float value) {
    return (value < low) ? low : (value > high) ? high : value;
}

static void* connection_thread(void* void_args_ptr) {
#define INT1    8
#define INT2   12
#define INT3   16
#define FLOAT1  8
#define FLOAT2 12
#define FLOAT4 24

#define MAX_CLIENT_STRING 64
    char client_string[MAX_CLIENT_STRING];
#define MAX_MESG 64
    char mesg[MAX_MESG];
    Tcp_Comms* p = (Tcp_Comms*)void_args_ptr;
    Raspicam_Char_Msg* char_msg_ptr = (Raspicam_Char_Msg*)mesg;
    Raspicam_Int_Msg* int_msg_ptr = (Raspicam_Int_Msg*)mesg;
    Raspicam_Float_Msg* float_msg_ptr = (Raspicam_Float_Msg*)mesg;
    bool error_seen = false;

    while ((bytes = recv(p->fd, &mesg, sizeof(mesg), 0)) > 0) {
        float timestamp = get_usecs() / (float)USECS_PER_SECOND;
        pthread_mutex_lock(&p->lock_mutex);
        switch (mesg[0]) {
        case RASPICAM_SATURATION:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.saturation =
                               int_limit(-100, 100, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_saturation(p->camera_ptr, 
                                               p->cam_params.saturation);
            }
            break;
        case RASPICAM_SHARPNESS :
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.sharpness =
                               int_limit(-100, 100, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_sharpness(p->camera_ptr, 
                                              p->cam_params.sharpness);
            }
            break;
        case RASPICAM_CONTRAST:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.contrast =
                               int_limit(-100, 100, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_contrast(p->camera_ptr, 
                                             p->cam_params.contrast);
            }
            break;
        case RASPICAM_BRIGHTNESS:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.brightness =
                                   int_limit(0, 100, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_brightness(p->camera_ptr, 
                                               p->cam_params.brightness);
            }
            break;
        case RASPICAM_ISO:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.ISO = ntohl(int_msg_ptr->int0);
                raspicamcontrol_set_ISO(p->camera_ptr, p->cam_params.ISO);
            }
            break;
        case RASPICAM_METERING_MODE:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.exposureMeterMode =
                       (MMAL_PARAM_EXPOSURE_METERINGMODE_T)int_limit(
                                                0, 3, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_metering_mode(p->camera_ptr,
                                               p->cam_params.exposureMeterMode);
            }
            break;
        case RASPICAM_VIDEO_STABILISATION:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.videoStabilisation =
                                     int_limit(0, 1, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_video_stabilisation(p->camera_ptr, 
                                             p->cam_params.videoStabilisation);
            }
            break;
        case RASPICAM_EXPOSURE_COMPENSATION:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.exposureCompensation =
                                   int_limit(-10, 10, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_exposure_compensation(p->camera_ptr, 
                                            p->cam_params.exposureCompensation);
            }
            break;
        case RASPICAM_EXPOSURE_MODE:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.exposureMode = (MMAL_PARAMETER_EXPOSURE_MODE_T)
                                     int_limit(0, 12, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_exposure_mode(p->camera_ptr, 
                                                  p->cam_params.exposureMode);
            }
            break;
        case RASPICAM_AWB_MODE:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.awbMode = (MMAL_PARAMETER_AWB_MODE_T)
                                    int_limit(0, 9, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_awb_mode(p->camera_ptr,
                                             p->cam_params.awbMode);
            }
            break;
        case RASPICAM_AWB_GAINS:
            if (bytes < FLOAT2) {
                error_seen = true;
            } else {
                p->cam_params.awb_gains_r = float_limit(0.00001, 1.0,
                                                 ntohl(float_msg_ptr->float0));
                p->cam_params.awb_gains_b = float_limit(0.00001, 1.0,
                                                 ntohl(float_msg_ptr->float1));
                raspicamcontrol_set_awb_gains(p->camera_ptr,
                                              p->cam_params.awb_gains_4,
                                              p->cam_params.awb_gains_b);
            }
            break;
        case RASPICAM_IMAGE_FX:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.imageEffect = (MMAL_PARAMETER_IMAGEFX_T)
                                    int_limit(0, 22, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_imageFX(p->camera_ptr, 
                                            p->cam_params.imageEffect);
            }
            break;
        case RASPICAM_COLOUR_FX:
            if (bytes < INT3) {
                error_seen = true;
            } else {
                p->cam_params.colourEffects.enable =
                                    int_limit(0, 1, ntohl(int_msg_ptr->int0));
                p->cam_params.colourEffects.u =
                                    int_limit(0, 255, ntohl(int_msg_ptr->int1));
                p->cam_params.colourEffects.v =
                                    int_limit(0, 255, ntohl(int_msg_ptr->int2));
                raspicamcontrol_set_imageFX(p->camera_ptr, 
                                            p->cam_params.colourEffects);
            }
            break;
        case RASPICAM_ROTATION:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.rotation = int_limit(0, 359,
                                               ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_awb_mode(p->camera_ptr,
                                             p->cam_params.rotation);
            }
            break;
        case RASPICAM_FLIPS:
            if (bytes < INT2) {
                error_seen = true;
            } else {
                p->cam_params.hflip = int_limit(0, 1, ntohl(int_msg_ptr->int0));
                p->cam_params.vflip = int_limit(0, 1, ntohl(int_msg_ptr->int1));
                raspicamcontrol_set_flips(p->camera_ptr, p->cam_params.hflip,
                                          p->cam_params.vflip);
            }
            break;
        case RASPICAM_AWB_GAINS:
            if (bytes < FLOAT4) {
                error_seen = true;
            } else {
                p->cam_params.roi.x = float_limit(0.0, 1.0,
                                              ntohl(float_msg_ptr->float0));
                p->cam_params.roi.y = float_limit(0.0, 1.0,
                                              ntohl(float_msg_ptr->float1));
                p->cam_params.roi.width = float_limit(0.0, 1.0,
                                              ntohl(float_msg_ptr->float2));
                p->cam_params.roi.height = float_limit(0.0, 1.0,
                                              ntohl(float_msg_ptr->float3));
                raspicamcontrol_set_ROI(p->camera_ptr, p->cam_params.roi);
            }
            break;
        case RASPICAM_SHUTTER_SPEED:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.shutter_speed = ntohl(int_msg_ptr->int0);
                raspicamcontrol_set_shutter_speed(p->camera_ptr,
                                                  p->cam_params.shutter_speed);
            }
            break;
        case RASPICAM_DRC:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.drc_level = (MMAL_PARAMETER_DRC_STRENGTH_T)
                                      int_limit(0, 3, ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_DRC(p->camera_ptr, p->cam_params.drc_level);
            }
            break;
        case RASPICAM_STATS_PASS:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->cam_params.stats_pass = int_limit(0, 1,
                                                     ntohl(int_msg_ptr->int0));
                raspicamcontrol_set_stats_pass(p->camera_ptr,
                                               p->cam_params.stats_pass);
            }
            break;
        case RASPICAM_TEST_IMAGE_ENABLE:
            if (bytes < INT1) {
                error_seen = true;
            } else {
                p->test_image_enable = int_limit(0, 1,
                                                 ntohl(int_msg_ptr->int0));
            }
            break;
        case RASPICAM_DETECT_YUV:
            if (bytes < 7) {
                error_seen = true;
            } else {
                p->detect_yuv_min[0] = char_msg_ptr->c[0];
                p->detect_yuv_max[0] = char_msg_ptr->c[1];
                p->detect_yuv_min[1] = char_msg_ptr->c[2];
                p->detect_yuv_max[1] = char_msg_ptr->c[3];
                p->detect_yuv_min[2] = char_msg_ptr->c[4];
                p->detect_yuv_max[2] = char_msg_ptr->c[5];
            }
            break;
        default:
            LOG_ERROR("at %.3f, unexpected message %d from %s\n",
                      timestamp, mesg[0],
                      get_ip_addr_str(p->saddr, client_string,
                                      MAX_CLIENT_STRING));
        }
        pthread_mutex_unlock(&p->lock_mutex);
        if (error_seen) {
            LOG_ERROR("at %.3f, too few bytes (%d) in message %d from %s\n",
                      timestamp, bytes, mesg[0],
                      get_ip_addr_str(p->saddr, client_string,
                                      MAX_CLIENT_STRING));
        }
    }
    LOG_ERROR("at %.3f, can't recv from %s; errno= %d\n",
              get_usecs() / (float)USECS_PER_SECOND,
              get_ip_addr_str(p->saddr, client_string, MAX_CLIENT_STRING),
              errno);
    return NULL;
}

/**
 * Start up the message_loop().
 */
static void* server_thread(void* void_args_ptr) {
    Tcp_Comms* comms_ptr = (Tcp_Comms*)void_args_ptr;
    int socket_fd = comms_ptr->socket.fd;
    int client_fd;
    struct sockaddr_storage client_addr;

    socklen_t bytes = sizeof(client_addr);

    // We always keep around space for one unused client to work in.

    pthread_mutex_lock(&comms_ptr->lock_mutex);
    comms_ptr->client = (Tcp_Host_Info*)malloc(sizeof(Tcp_Host_Info));
    comms_ptr->client_count = 0;
    pthread_mutex_unlock(&comms_ptr->lock_mutex);
    Tcp_Host_Info* unused_ptr = comms_ptr->client[comms_ptr->client_count];

    while ((unused_ptr->fd = accept(socket_fd,
                                    (struct sockaddr*)&unused_ptr->saddr,
                                    &unused_ptr->saddr_len)) >= 0) {
        pthread_mutex_lock(&comms_ptr->lock_mutex);
        ++comms_ptr->client_count;

        // Start a new thread to handle all communications with the new client.

        status = pthread_create(&unused_ptr->pthread_id, NULL,
                                connection_thread, unused_ptr);
        if (status != 0) {
            LOG_ERROR("at %.3f, can't pthread_create (%d)\n",
                      get_uscs / (float)USECS_PER_SECOND, status);
            return NULL;
        }

        // Make space for a new (unused) client.

        comms_ptr->client = realloc(comms_ptr->client,
                     (comms_ptr->client_count + 1) * sizeof(Tcp_Host_Info));
        unused_ptr = comms_ptr->client[comms_ptr->client_count];
        pthread_mutex_unlock(&comms_ptr->lock_mutex);
    }
    LOG_ERROR("at %.3f, can't accept; errno=%d\n",
              get_uscs / (float)USECS_PER_SECOND, errno);
    return NULL;
}

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
} Tcp_Comms;

int tcp_comms_construct(Tcp_Comms* comms_ptr,
                        MMAL_COMPONENT_T camera_ptr,
                        unsigned short port_number) {
    int comms_ptr->server.fd = socket(AF_INET, SOCK_STREAM, 0);
    if (comms_ptr->server.fd < 0) {
        LOG_ERROR("can't create socket; errno=%d\n", errno);
        return -1;
    }

    comms_ptr->saddr_len = sizeof(struct sockaddr_in);
    struct sockaddr_in* saddr_ptr = (struct sockaddr_in*)&comms_ptr->saddr;
    saddr_ptr->sin_family = AF_INET;
    saddr_ptr->sin_addr.s_addr = INADDR_ANY;
    saddr_ptr->sin_port = htons(port_number);

    if (bind(comms_ptr->server.fd,
             (struct sockaddr*)saddr_ptr, comms_ptr->saddr_len) < 0) {
        LOG_ERROR("can't bind; errno=%d\n", errno);
        return -1;
    }

    if (listen(comms_ptr->server.fd, 3) < 0) {
        LOG_ERROR("can't listen; errno=%d\n", errno);
        return -1;
    }

    // Initialize shared fields of *comms_ptr.

    comms_ptr->client_count = 0;
    comms_ptr->client = NULL;
    comms_ptr->camera_ptr = camera_ptr;
    comms_ptr->test_image_enable = false;
    raspicamcontrol_set_defaults(&comms_ptr->cam_params);
    int status = pthread_mutex_init(&comms_ptr->lock_mutex, NULL);
    if (status != 0) {
        LOG_ERROR("can't pthread_mutex_init (%d)\n", status);
        return -1;
    }

    // Start server_loop thread.

    status = pthread_create(&comms_ptr->server_pthread_id, NULL,
                            server_thread, comms_ptr);
    if (status != 0) {
        LOG_ERROR("can't pthread_create (%d)\n", status);
        return -1;
    }
    return 0;
}
