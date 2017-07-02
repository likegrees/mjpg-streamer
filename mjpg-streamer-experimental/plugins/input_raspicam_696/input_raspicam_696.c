/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2007 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
 *******************************************************************************/
#include <unistd.h>  /* for sleep */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>
#include <time.h>

#include <linux/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

#include "../../mjpg_streamer.h"
#include "../../utils.h"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "mmal/mmal.h"
#include "mmal/util/mmal_default_components.h"
#include "mmal/util/mmal_connection.h"
#include "mmal/util/mmal_util.h"
#include "overwrite_tif_tags.h"
#include "detect_color_blobs.h"

#include "RaspiCamControl.c"

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2
// Port configuration for the splitter component
#define SPLITTER_CALLBACK_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Stills format information
#define STILLS_FRAME_RATE_NUM 3
#define STILLS_FRAME_RATE_DEN 1
/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

#define INPUT_PLUGIN_NAME "team 696 raspicam input plugin"

// Layer that preview window should be displayed on
#define PREVIEW_LAYER      2
// Frames rates of 0 implies variable, but denominator needs to be 1 to prevent div by 0
#define PREVIEW_FRAME_RATE_NUM 0
#define PREVIEW_FRAME_RATE_DEN 1

#define LOG_ERROR(...) \
    { char _bf[1024] = {0}; \
      snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__); \
      fprintf(stderr, "%s", _bf); \
      syslog(LOG_INFO, "%s+%d: %s", __FILE__, __LINE__, _bf); }

typedef struct {
    MMAL_COMPONENT_T* component_ptr;
    MMAL_POOL_T* pool_ptr;
} Component_Pool;

typedef struct {
    bool write_images;
    bool detect_yuv;
    unsigned char blob_yuv_min[3];
    unsigned char blob_yuv_max[3];
    unsigned int frame_no;
    MMAL_POOL_T* pool_ptr;
    FILE* raw_fp;
    Blob_List blob_list;
    pthread_mutex_t bbox_mutex;
#define MAX_BBOXES 20
    unsigned short bbox_element_count;
    unsigned short bbox_element[MAX_BBOXES * 4];
} Splitter_Callback_Data;

static init_splitter_callback_data(Splitter_Callback_Data* p) {
    p->detect_yuv = false;
    p->write_images = false;
    p->blob_yuv_min[0] = 0;
    p->blob_yuv_min[1] = 0;
    p->blob_yuv_min[2] = 0;
    p->blob_yuv_max[0] = 0;
    p->blob_yuv_max[1] = 0;
    p->blob_yuv_max[2] = 0;
    p->frame_no = UINT_MAX; // encoder throws away 1st frame so we should too
    p->pool_ptr = NULL;
    p->raw_fp = NULL;
    p->bbox_element_count = 0;
    pthread_mutex_init(&p->bbox_mutex, NULL);
}

/* private functions and variables to this plugin */
static pthread_t   worker;
static globals     *pglobal;
static pthread_mutex_t controls_mutex;
static int plugin_number;

void *worker_thread(void *);
void worker_cleanup(void *);
void help(void);

static int fps = 5;
static int width = 640;
static int height = 480;
static int quality = 85;
static int usestills = 0;
static int wantPreview = 0;
static int wantTimestamp = 0;
static RASPICAM_CAMERA_PARAMETERS c_params;
static Splitter_Callback_Data splitter_callback_data;

static struct timeval timestamp;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
  FILE *file_handle; /// File handle to write buffer data to.
  VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
  MMAL_POOL_T *pool; /// pointer to our state in case required in callback
  Splitter_Callback_Data* splitter_data_ptr;
  uint32_t offset;
  unsigned int frame_no;
  unsigned int width;
  unsigned int height;
} PORT_USERDATA;


//#define DSC
#ifdef DSC
FILE* debug_fp = NULL;
#endif

/*** plugin interface functions ***/

/******************************************************************************
  Description.: parse input parameters
  Input Value.: param contains the command line string and a pointer to globals
  Return Value: 0 if everything is ok
 ******************************************************************************/
int input_init(input_parameter *param, int plugin_no)
{
#ifdef DSC
  debug_fp = fopen("debug.txt", "w");
#endif
  int i;
  if (pthread_mutex_init(&controls_mutex, NULL) != 0)
  {
    IPRINT("could not initialize mutex variable\n");
    exit(EXIT_FAILURE);
  }

  init_splitter_callback_data(&splitter_callback_data);

  param->argv[0] = INPUT_PLUGIN_NAME;
  plugin_number = plugin_no;

  //setup the camera control st
  raspicamcontrol_set_defaults(&c_params);

  /* show all parameters for DBG purposes */
  for (i = 0; i < param->argc; i++)
  {
    DBG("argv[%d]=%s\n", i, param->argv[i]);
  }

  reset_getopt();
  while(1) {
    int option_index = 0, c = 0;
    static struct option long_options[] = {
      {"h", no_argument, 0, 0},                       // 0
      {"help", no_argument, 0, 0},                    // 1
      {"x", required_argument, 0, 0},                 // 2
      {"width", required_argument, 0, 0},             // 3
      {"y", required_argument, 0, 0},                 // 4
      {"height", required_argument, 0, 0},            // 5
      {"fps", required_argument, 0, 0},               // 6
      {"framerate", required_argument, 0, 0},         // 7
      {"sh", required_argument, 0, 0},                // 8
      {"co", required_argument, 0, 0},                // 9
      {"br", required_argument, 0, 0},                // 10
      {"sa", required_argument, 0, 0},                // 11
      {"ISO", required_argument, 0, 0},               // 12
      {"vs", no_argument, 0, 0},                      // 13
      {"ev", required_argument, 0, 0},                // 14
      {"ex", required_argument, 0, 0},                // 15
      {"awb", required_argument, 0, 0},               // 16
      {"ifx", required_argument, 0, 0},               // 17
      {"cfx", required_argument, 0, 0},               // 18
      {"mm", required_argument, 0, 0},                // 19
      {"rot", required_argument, 0, 0},               // 20
      {"hf", no_argument, 0, 0},                      // 21
      {"vf", no_argument, 0, 0},                      // 22
      {"quality", required_argument, 0, 0},           // 23
      {"usestills", no_argument, 0, 0},               // 24
      {"preview", no_argument, 0, 0},                 // 25
      {"timestamp", no_argument, 0, 0},               // 26
      {"stats", no_argument, 0, 0},                   // 27
      {"drc", required_argument, 0, 0},               // 28
      {"shutter", required_argument, 0, 0},           // 29
      {"awbgainR", required_argument, 0, 0},          // 30
      {"awbgainB", required_argument, 0, 0},          // 31
      {"blobyuv", required_argument, 0, 0},           // 32
      {"writeimages", no_argument, 0, 0},             // 33
      {0, 0, 0, 0}
    };

    c = getopt_long_only(param->argc, param->argv, "", long_options, &option_index);

    /* no more options to parse */
    if(c == -1)
      break;

    /* unrecognized option */
    if (c == '?')
    {
      help();
      return 1;
    }

    switch(option_index) {
      /* h, help */
      case 0:
      case 1:
        DBG("case 0,1\n");
        help();
        return 1;
        break;
        /* width */
      case 2:
      case 3:
        DBG("case 2,3\n");
        width = atoi(optarg);
        break;
        /* height */
      case 4:
      case 5:
        DBG("case 4,5\n");
        height = atoi(optarg);
        break;
        /* fps */
      case 6:
      case 7:
        DBG("case 6, 7\n");
        fps = atoi(optarg);
        break;
      case 8:
        //sharpness
        sscanf(optarg, "%d", &c_params.sharpness);
        break;
      case 9:
        //contrast
        sscanf(optarg, "%d", &c_params.contrast);
        break;
      case 10:
        //brightness
        sscanf(optarg, "%d", &c_params.brightness);
        break;
      case 11:
        //saturation
        sscanf(optarg, "%d", &c_params.saturation);
        break;
      case 12:
        //ISO
        sscanf(optarg, "%d", &c_params.ISO);
        break;
      case 13:
        //video stabilisation
        c_params.videoStabilisation = 1;
        break;
      case 14:
        //ev
        sscanf(optarg, "%d", &c_params.exposureCompensation);
        break;
      case 15:
        //exposure
        c_params.exposureMode = exposure_mode_from_string(optarg);
        break;
      case 16:
        //awb mode
        c_params.awbMode = awb_mode_from_string(optarg);
        break;
      case 17:
        //img effect
        c_params.imageEffect = imagefx_mode_from_string(optarg);
        break;
      case 18:
        //color effects
        sscanf(optarg, "%d:%d", &c_params.colourEffects.u, &c_params.colourEffects.u);
        c_params.colourEffects.enable = 1;
        break;
      case 19:
        //metering mode
        c_params.exposureMeterMode = metering_mode_from_string(optarg);
        break;
      case 20:
        //rotation
        sscanf(optarg, "%d", &c_params.rotation);
        break;
      case 21:
        //hflip
        c_params.hflip  = 1;
        break;
      case 22:
        //vflip
        c_params.vflip = 1;
        break;
      case 23:
        //quality
        quality = atoi(optarg);
        break;
      case 24:
        //use stills
        usestills = 1;
        break;
      case 25:
        //display preview
        wantPreview = 1;
        break;
      case 26:
        //timestamp
        wantTimestamp = 1;
        break;
      case 27:
        // use stats
        c_params.stats_pass = MMAL_TRUE;
        break;
      case 28:
        // Dynamic Range Compensation DRC
        c_params.drc_level = drc_mode_from_string(optarg);
        break;
      case 29:
        // shutter speed in microseconds
        sscanf(optarg, "%d", &c_params.shutter_speed);
        break;
      case 30:
        // awb gain red
        sscanf(optarg, "%f", &c_params.awb_gains_r);
        break;
      case 31:
        // awb gain blue
        sscanf(optarg, "%f", &c_params.awb_gains_b);
        break;
      case 32:
        // blobyuv
        {
          unsigned int opt0;
          unsigned int opt1;
          unsigned int opt2;
          unsigned int opt3;
          unsigned int opt4;
          unsigned int opt5;
          if (sscanf(optarg, "%u,%u,%u,%u,%u,%u",
                     &opt0, &opt1, &opt2, &opt3, &opt4, &opt5) != 6) {
            fprintf(stderr,
                    "-blobyuv requires 6 unsigned int arguments\n");
            help();
            return 1;
          }
          if (opt0 > 255) opt0 = 255;
          if (opt1 > 255) opt1 = 255;
          if (opt2 > 255) opt2 = 255;
          if (opt3 > 255) opt3 = 255;
          if (opt4 > 255) opt4 = 255;
          if (opt5 > 255) opt5 = 255;
          splitter_callback_data.blob_yuv_min[0] = opt0;
          splitter_callback_data.blob_yuv_max[0] = opt1;
          splitter_callback_data.blob_yuv_min[1] = opt2;
          splitter_callback_data.blob_yuv_max[1] = opt3;
          splitter_callback_data.blob_yuv_min[2] = opt4;
          splitter_callback_data.blob_yuv_max[2] = opt5;
          splitter_callback_data.detect_yuv = true;
#define MAX_RUNS 10000
#define MAX_BLOBS 1000
          splitter_callback_data.blob_list = blob_list_init(MAX_RUNS,
                                                            MAX_BLOBS);
        }
        break;
      case 33:
        //writeimages
        splitter_callback_data.write_images = true;
        break;
      default:
        DBG("default case\n");
        help();
        return 1;
    }
  }

  pglobal = param->global;

  IPRINT("fps.............: %i\n", fps);
  IPRINT("resolution........: %i x %i\n", width, height);
  IPRINT("camera parameters..............:\n\n");
  raspicamcontrol_dump_parameters(&c_params);

  return 0;
}

/******************************************************************************
  Description.: stops the execution of the worker thread
  Input Value.: -
  Return Value: 0
 ******************************************************************************/
int input_stop(int id)
{
  DBG("will cancel input thread\n");
  pthread_cancel(worker);

  return 0;
}

/**************************************************
  Print which status
 **************************************************/
void log_mmal_status(MMAL_STATUS_T status)
{
  if (status != MMAL_SUCCESS)
  {
    switch (status)
    {
      case MMAL_ENOMEM : LOG_ERROR("Out of memory\n"); break;
      case MMAL_ENOSPC : LOG_ERROR("Out of resources (other than memory)\n"); break;
      case MMAL_EINVAL: LOG_ERROR("Argument is invalid\n"); break;
      case MMAL_ENOSYS : LOG_ERROR("Function not implemented\n"); break;
      case MMAL_ENOENT : LOG_ERROR("No such file or directory\n"); break;
      case MMAL_ENXIO : LOG_ERROR("No such device or address\n"); break;
      case MMAL_EIO : LOG_ERROR("I/O error\n"); break;
      case MMAL_ESPIPE : LOG_ERROR("Illegal seek\n"); break;
      case MMAL_ECORRUPT : LOG_ERROR("Data is corrupt \attention FIXME: not POSIX\n"); break;
      case MMAL_ENOTREADY :LOG_ERROR("Component is not ready \attention FIXME: not POSIX\n"); break;
      case MMAL_ECONFIG : LOG_ERROR("Component is not configured \attention FIXME: not POSIX\n"); break;
      case MMAL_EISCONN : LOG_ERROR("Port is already connected\n"); break;
      case MMAL_ENOTCONN : LOG_ERROR("Port is disconnected\n"); break;
      case MMAL_EAGAIN : LOG_ERROR("Resource temporarily unavailable. Try again later\n"); break;
      case MMAL_EFAULT : LOG_ERROR("Bad address\n"); break;
      default : LOG_ERROR("Unknown status error\n"); break;
    }
  }
}


/**
 *  buffer header callback function for splitter
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void splitter_buffer_callback(MMAL_PORT_T *port,
                                     MMAL_BUFFER_HEADER_T *buffer) {
    MMAL_BUFFER_HEADER_T *new_buffer;
    Splitter_Callback_Data *pData = (Splitter_Callback_Data*)port->userdata;
    fprintf(stderr, "splitter: %u %lld %lld %lld len= %d\n",
            pData->frame_no, buffer->pts, buffer->dts, vcos_getmicrosecs64(), buffer->length);

    if (pData != NULL) {
        mmal_buffer_header_mem_lock(buffer);
        if (pData->detect_yuv) {
            detect_color_blobs(&pData->blob_list, pData->blob_yuv_min[0],
                    pData->blob_yuv_min[1], pData->blob_yuv_max[1],
                    pData->blob_yuv_min[2], pData->blob_yuv_max[2],
                    false, port->format->es->video.width,
                    port->format->es->video.height, buffer->data);
#define MIN_PIXELS_PER_BLOB 30
            (void)blob_list_purge_small_bboxes(&pData->blob_list,
                                               MIN_PIXELS_PER_BLOB);
            pthread_mutex_lock(&pData->bbox_mutex);
            pData->bbox_element_count =
                    copy_best_bounding_boxes(&pData->blob_list, MAX_BBOXES * 4,
                                             pData->bbox_element);
            pthread_mutex_unlock(&pData->bbox_mutex);
            printf("bbox_element_count= %d\n", pData->bbox_element_count);
        }
        mmal_buffer_header_mem_unlock(buffer);
    } else {
        LOG_ERROR("Received a camera buffer callback with no state");
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);
    ++pData->frame_no;

    // and send one back to the port (if still open)
    if (port->is_enabled)
    {
        MMAL_STATUS_T status;
        new_buffer = mmal_queue_get(pData->pool_ptr->queue);
        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }
        if (!new_buffer || status != MMAL_SUCCESS) {
            LOG_ERROR("Unable to return a buffer to the splitter port");
        }
    }
}


/******************************************************************************
  Callback from mmal JPEG encoder
 ******************************************************************************/
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  int complete = 0;

  // We pass our file handle and other stuff in via the userdata field.
  PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
  fprintf(stderr, "encoder: %u %lld %lld %lld len= %d flags= %x\n", pData->frame_no,
          buffer->pts, buffer->dts, vcos_getmicrosecs64(), buffer->length, buffer->flags);

  if (pData)
  {
    if (buffer->length)
    {
      mmal_buffer_header_mem_lock(buffer);

      //fprintf(stderr, "The flags are %x of length %i offset %i\n", buffer->flags, buffer->length, pData->offset);

      //Write bytes
      /* copy JPG picture to global buffer */
      if(pData->offset == 0)
        pthread_mutex_lock(&pglobal->in[plugin_number].db);

#define SEND_BBOXES
#ifdef SEND_BBOXES
      if (buffer->data[0] == 0xff &&
          buffer->data[1] == 0xd8 &&
          buffer->data[2] == 0xff &&
          buffer->data[3] == 0xe1) {

          /* Send the bounding boxes in the image packets by overwriting the
             tiff tags in the header supplied by Broadcom. */

          unsigned int cols = pData->width;
          unsigned int rows = pData->height;
          const unsigned int SIZE_FIELD_OFFSET = 4;
          unsigned int total_header_bytes = SIZE_FIELD_OFFSET +
                      (((unsigned int)buffer->data[4] << 8) | buffer->data[5]);
          const unsigned int TIFF_TAGS_OFFSET = 12;
          pthread_mutex_lock(&pData->splitter_data_ptr->bbox_mutex);
          overwrite_tif_tags(cols, rows,
                             pData->splitter_data_ptr->bbox_element_count,
                             pData->splitter_data_ptr->bbox_element,
                             buffer->data);
          pthread_mutex_unlock(&pData->splitter_data_ptr->bbox_mutex);
      }
#endif
#ifdef DSC
      if (debug_fp != NULL) {
        if (buffer->data[0] == 0xff &&
            buffer->data[1] == 0xd8 &&
            buffer->data[2] == 0xff &&
            buffer->data[3] == 0xe1) {
            unsigned int len = ((unsigned int)buffer->data[4] << 8) | buffer->data[5];
            fprintf(debug_fp, "ff d8 ff e1");
            int ii;
            for (ii = 0; ii < len; ++ii) {
                fprintf(debug_fp, " %02x", buffer->data[ii + 4]);
            }
            fprintf(debug_fp, "\n");
            fflush(debug_fp);
        }
      }
#endif
      memcpy(pData->offset + pglobal->in[plugin_number].buf, buffer->data, buffer->length);
      pData->offset += buffer->length;
      //fwrite(buffer->data, 1, buffer->length, pData->file_handle);
      mmal_buffer_header_mem_unlock(buffer);
    }

    // Now flag if we have completed
    if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
    {
      //set frame size
      pglobal->in[plugin_number].size = pData->offset;

      //Set frame timestamp
      if(wantTimestamp)
      {
        gettimeofday(&timestamp, NULL);
        pglobal->in[plugin_number].timestamp = timestamp;
      }

      //mark frame complete
      complete = 1;

      pData->offset = 0;
      ++pData->frame_no;
      /* signal fresh_frame */
      pthread_cond_broadcast(&pglobal->in[plugin_number].db_update);
      pthread_mutex_unlock(&pglobal->in[plugin_number].db);
    }
  }
  else
  {
    LOG_ERROR("Received a encoder buffer callback with no state\n");
  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled)
  {
    MMAL_STATUS_T status;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pData->pool->queue);

    if (new_buffer)
    {
      status = mmal_port_send_buffer(port, new_buffer);

      if(status != MMAL_SUCCESS)
      {
        LOG_ERROR("Failed returning a buffer to the encoder port \n");
        log_mmal_status(status);
      }

    }
    else
    {
      LOG_ERROR("Unable to return a buffer to the encoder port\n");
    }

  }

  if (complete)
    vcos_semaphore_post(&(pData->complete_semaphore));

}

/**
 * buffer header callback function for camera control
 *
 * No actions taken in current version
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_control_callback(MMAL_PORT_T *port,
                                    MMAL_BUFFER_HEADER_T *buffer) {
    if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED) {
#define DSC1
#ifdef DSC1
        MMAL_EVENT_PARAMETER_CHANGED_T *param =
            (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
        switch (param->hdr.id) {
            case MMAL_PARAMETER_CAMERA_SETTINGS: {
                 MMAL_PARAMETER_CAMERA_SETTINGS_T *settings =
                     (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
                printf(
                 "Exposure now %u, analog gain %u/%u, digital gain %u/%u ",
                       settings->exposure,
                       settings->analog_gain.num, settings->analog_gain.den,
                       settings->digital_gain.num, settings->digital_gain.den);
                printf("AWB R=%u/%u, B=%u/%u\n",
                       settings->awb_red_gain.num, settings->awb_red_gain.den,
                       settings->awb_blue_gain.num,
                       settings->awb_blue_gain.den);
                break;
            }
            /*
            case MMAL_PARAMETER_CAPTURE_STATUS: {
                 MMAL_PARAMETER_CAPTURE_STATUS_T* status_ptr =
                                    (MMAL_PARAMETER_CAPTURE_STATUS_T*)param;
                if (status_ptr->status ==
                              MMAL_PARAM_CAPTURE_STATUS_CAPTURE_STARTED) {
                    printf("Capture Started\n");
                } else if (status_ptr->status ==
                              MMAL_PARAM_CAPTURE_STATUS_CAPTURE_ENDED) {
                    printf("Capture Ended\n");
                }
                break;
            }
            */
        }
#else
        fprintf(stderr, "camera_control\n");
#endif
    } else {
        LOG_ERROR("Received unexpected camera control callback event %d",
                  buffer->cmd);
    }

    mmal_buffer_header_release(buffer);
}


/**
 * Create the splitter component, set up its ports
 *
 * @param camera_component Pointer to camera component
 *
 * @return The newly created splitter component and pool.  On failure
 *         Component_Pool.component_ptr will be NULL, and the error will be
 *         logged.
 *
 */
static Component_Pool create_splitter_component(
                                        MMAL_COMPONENT_T* camera_component)
{
   Component_Pool pair = { NULL, NULL };
   MMAL_COMPONENT_T *splitter = 0;
   MMAL_PORT_T *splitter_output = NULL;
   MMAL_ES_FORMAT_T *format;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;
   int i;

   if (camera_component == NULL)
   {
      status = MMAL_ENOSYS;
      LOG_ERROR("Camera component must be created before splitter");
      log_mmal_status(status);
      goto error;
   }

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

   if (status != MMAL_SUCCESS)
   {
      LOG_ERROR("Failed to create splitter component");
      log_mmal_status(status);
      goto error;
   }

   if (!splitter->input_num)
   {
      status = MMAL_ENOSYS;
      LOG_ERROR("Splitter doesn't have any input port");
      log_mmal_status(status);
      goto error;
   }

   if (splitter->output_num < 2)
   {
      status = MMAL_ENOSYS;
      LOG_ERROR("Splitter doesn't have enough output ports");
      log_mmal_status(status);
      goto error;
   }

   /* Ensure there are enough buffers to avoid dropping frames: */
   mmal_format_copy(splitter->input[0]->format, camera_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);

   if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   status = mmal_port_format_commit(splitter->input[0]);

   if (status != MMAL_SUCCESS)
   {
      LOG_ERROR("Unable to set format on splitter input port");
      log_mmal_status(status);
      goto error;
   }

   /* Splitter can do format conversions, configure format for its output port: */
   for (i = 0; i < splitter->output_num; i++)
   {
      mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);

      if (i == SPLITTER_CALLBACK_PORT)
      {
         format = splitter->output[i]->format;
         format->encoding = MMAL_ENCODING_I420;
         format->encoding_variant = MMAL_ENCODING_I420;
      }

      status = mmal_port_format_commit(splitter->output[i]);

      if (status != MMAL_SUCCESS)
      {
         LOG_ERROR("Unable to set format on splitter output port %d", i);
         log_mmal_status(status);
         goto error;
      }
   }

   /* Enable component */
   status = mmal_component_enable(splitter);

   if (status != MMAL_SUCCESS)
   {
      LOG_ERROR("splitter component couldn't be enabled");
      log_mmal_status(status);
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   splitter_output = splitter->output[SPLITTER_CALLBACK_PORT];
   pool = mmal_port_pool_create(splitter_output, splitter_output->buffer_num, splitter_output->buffer_size);

   if (!pool)
   {
      LOG_ERROR("Failed to create buffer header pool for splitter output port %s", splitter_output->name);
   }

   pair.pool_ptr = pool;
   pair.component_ptr = splitter;
   return pair;

error:

   if (splitter)
      mmal_component_destroy(splitter);

   return pair;
}


/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_splitter_component(Component_Pool* pair_ptr)
{
   // Get rid of any port buffers first
   if (pair_ptr->pool_ptr)
   {
      mmal_port_pool_destroy(
                        pair_ptr->component_ptr->output[SPLITTER_CALLBACK_PORT],
                        pair_ptr->pool_ptr);
      pair_ptr->pool_ptr = NULL;
   }

   if (pair_ptr->component_ptr)
   {
      mmal_component_destroy(pair_ptr->component_ptr);
      pair_ptr->component_ptr = NULL;
   }
}


/******************************************************************************
  Description.: starts the worker thread and allocates memory
  Input Value.: -
  Return Value: 0
 ******************************************************************************/
int input_run(int id)
{
  pglobal->in[id].buf = malloc(width * height * 3);
  if (pglobal->in[id].buf == NULL)
  {
    fprintf(stderr, "could not allocate memory\n");
    exit(EXIT_FAILURE);
  }

  if (pthread_create(&worker, 0, worker_thread, NULL) != 0)
  {
    free(pglobal->in[id].buf);
    fprintf(stderr, "could not start worker thread\n");
    exit(EXIT_FAILURE);
  }
  pthread_detach(worker);

  return 0;
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
  MMAL_STATUS_T status;

  status = mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

  if (status == MMAL_SUCCESS)
  {
    status = mmal_connection_enable(*connection);
    if (status != MMAL_SUCCESS)
    {
      LOG_ERROR("Error enabling mmal connection\n");
      log_mmal_status(status);
      mmal_connection_destroy(*connection);
      *connection = NULL;
    }
  } else {
    LOG_ERROR("Error creating mmal connection\n");
    log_mmal_status(status);
    *connection = NULL;
  }

  return status;
}


/******************************************************************************
  Description.: print help message
  Input Value.: -
  Return Value: -
 ******************************************************************************/
void help(void)
{
  fprintf(stderr, " ---------------------------------------------------------------\n" \
      " Help for input plugin..: "INPUT_PLUGIN_NAME"\n" \
      " ---------------------------------------------------------------\n" \
      " The following parameters can be passed to this plugin:\n\n" \
      " [-fps | --framerate]...: set video framerate, default 5 frame/sec \n"\
      " [-x | --width ]........: width of frame capture, default 640\n" \
      " [-y | --height]........: height of frame capture, default 480 \n"\
      " [-quality].............: set JPEG quality 0-100, default 85 \n"\
      " [-usestills]...........: uses stills mode instead of video mode \n"\
      " [-preview].............: Enable full screen preview\n"\
      " [-timestamp]...........: Get timestamp for each frame\n"
      " \n"\
      " -sh  : Set image sharpness (-100 to 100)\n"\
      " -co  : Set image contrast (-100 to 100)\n"\
      " -br  : Set image brightness (0 to 100)\n"\
      " -sa  : Set image saturation (-100 to 100)\n"\
      " -ISO : Set capture ISO\n"\
      " -vs  : Turn on video stablisation\n"\
      " -ev  : Set EV compensation\n"\
      " -ex  : Set exposure mode (see raspistill notes)\n"\
      " -awb : Set AWB mode (see raspistill notes)\n"\
      " -ifx : Set image effect (see raspistill notes)\n"\
      " -cfx : Set colour effect (U:V)\n"\
      " -mm  : Set metering mode (see raspistill notes)\n"\
      " -rot : Set image rotation (0-359)\n"\
      " -stats : Compute image stats for each picture (reduces noise for -usestills)\n"\
      " -drc : Dynamic range compensation level (see raspistill notes)\n"\
      " -hf  : Set horizontal flip\n"\
      " -vf  : Set vertical flip\n"\
      " ---------------------------------------------------------------\n");

}

/******************************************************************************
  Description.: setup mmal and callback
  Input Value.: arg is not used
  Return Value: NULL
 ******************************************************************************/
void *worker_thread(void *arg)
{
  int i = 0;

  /* set cleanup handler to cleanup allocated resources */
  pthread_cleanup_push(worker_cleanup, NULL);
  //Lets not let this thread be cancelled, it needs to clean up mmal on exit
  if (pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL) != 0)
  {
    fprintf(stderr, "Unable to set cancel state\n");
    exit(EXIT_FAILURE);
  }

  IPRINT("Starting Camera\n");

  //Camera variables
  MMAL_COMPONENT_T *camera = 0;
  MMAL_COMPONENT_T *preview = 0;
  MMAL_ES_FORMAT_T *format;
  MMAL_STATUS_T status;
  MMAL_PORT_T *camera_preview_port = NULL;
  MMAL_PORT_T *camera_video_port = NULL;
  MMAL_PORT_T *camera_still_port = NULL;
  MMAL_PORT_T *preview_input_port = NULL;
  MMAL_PORT_T *splitter_callback_port = NULL;


  MMAL_CONNECTION_T *preview_connection = 0;

  //Encoder variables
  MMAL_COMPONENT_T *encoder = 0;
  MMAL_PORT_T *encoder_input = NULL;
  MMAL_PORT_T *encoder_output = NULL;
  MMAL_POOL_T *pool;
  MMAL_CONNECTION_T *encoder_connection;

  // Splitter variables
  Component_Pool splitter_pair;
  MMAL_CONNECTION_T *splitter_connection = NULL;

  //fps count
  struct timespec t_start, t_finish;
  double t_elapsed;
  int frames;

  //Create camera code
  bcm_host_init();
  DBG("Host init, starting mmal stuff\n");

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "error create camera\n");
    exit(EXIT_FAILURE);
  }

  if (!camera->output_num)
  {
    fprintf(stderr, "Camera doesn't have output ports\n");
    mmal_component_destroy(camera);
    exit(EXIT_FAILURE);
  }

  camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
  camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
  camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

  //Enable camera control port
  // Enable the camera, and tell it its control callback function
#ifdef DSC1
  MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
                         { { MMAL_PARAMETER_CHANGE_EVENT_REQUEST,
                             sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T) },
                          MMAL_PARAMETER_CAMERA_SETTINGS, 1 };
  (void)mmal_port_parameter_set(camera->control, &change_event_request.hdr);
  /* Doesn't work with video
  MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request2 =
                         { { MMAL_PARAMETER_CHANGE_EVENT_REQUEST,
                             sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T) },
                          MMAL_PARAMETER_CAPTURE_STATUS, 1 };
  (void)mmal_port_parameter_set(camera->control, &change_event_request2.hdr);
  */
#endif
  status = mmal_port_enable(camera->control, camera_control_callback);
#ifdef DSC1
  /* THIS IS HOW YOU GET THE TIME FROM THE CAMERA. 
     Time is in microseconds (since boot).
     This should match the frame time stamp (if you are in raw mode).
  */
  typedef struct {
      MMAL_PARAMETER_HEADER_T hdr;
      uint64_t time;
  } Time_Request;
  Time_Request time_request = { { MMAL_PARAMETER_SYSTEM_TIME,
                                  sizeof(Time_Request) },
                                  0 };
  mmal_port_parameter_get(camera->control, &time_request.hdr);
  printf("*********** time = %lld\n", time_request.time);

  /* Query GPU for how much memory it is using. See raspicamcontrol_get_mem().
     Memory is in megabytes.
     The split between GPU and CPU memory is set by running "sudo raspi-config"
     */
  char response[80] = "";
  int gpu_mem = 0;
  if (vc_gencmd(response, sizeof response, "get_mem gpu") == 0)
  vc_gencmd_number_property(response, "gpu", &gpu_mem);
  printf("*********** gpu_mem = %d\n", gpu_mem);

#endif

  if (status)
  {
    fprintf(stderr, "Unable to enable camera port\n");
    mmal_component_destroy(camera);
    exit(EXIT_FAILURE);
  }


  {
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
      { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
      .max_stills_w = width,
      .max_stills_h = height,
      .stills_yuv422 = 0,
      .one_shot_stills = (usestills ? 1 : 0),
      .max_preview_video_w = width,
      .max_preview_video_h = height,
      .num_preview_video_frames = 3,
      .stills_capture_circular_buffer_height = 0,
      .fast_preview_resume = 0,
      .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
      // consider .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
    };
    mmal_port_parameter_set(camera->control, &cam_config.hdr);
  }

  //Set camera parameters
  if (raspicamcontrol_set_all_parameters(camera, &c_params))
    fprintf(stderr, "camera parameters couldn't be set\n");

  // Set the encode format on the Preview port

  format = camera_preview_port->format;
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->encoding_variant = MMAL_ENCODING_I420;  // DSC
  format->es->video.width = VCOS_ALIGN_UP(width, 32);
  format->es->video.height = VCOS_ALIGN_UP(height, 16);
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = width;
  format->es->video.crop.height = height;
  format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
  format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;

  status = mmal_port_format_commit(camera_preview_port);

  // Create preview component
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &preview);

  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "Unable to create preview component\n");
    exit(EXIT_FAILURE);
  }

  if (!preview->input_num)
  {
    status = MMAL_ENOSYS;
    fprintf(stderr, "No input ports found on preview component");
    exit(EXIT_FAILURE);
  }

  preview_input_port = preview->input[0];

  MMAL_DISPLAYREGION_T param;
  param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
  param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);

  param.set = MMAL_DISPLAY_SET_LAYER;
  param.layer = PREVIEW_LAYER;

  param.set |= MMAL_DISPLAY_SET_ALPHA;
  param.alpha = 255;

  param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
  param.fullscreen = 1;

  status = mmal_port_parameter_set(preview_input_port, &param.hdr);

  if (status != MMAL_SUCCESS && status != MMAL_ENOSYS)
  {
   fprintf(stderr, "unable to set preview port parameters (%u)", status);
   exit(EXIT_FAILURE);
  }

  if (!usestills)
  {
    // Set the encode format on the video port
    format = camera_video_port->format;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->encoding = MMAL_ENCODING_I420;
    format->es->video.width = width;
    format->es->video.height = height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = width;
    format->es->video.crop.height = height;
    format->es->video.frame_rate.num = fps;
    format->es->video.frame_rate.den = 1;
    status = mmal_port_format_commit(camera_video_port);
    if (status)
      fprintf(stderr, "camera video format couldn't be set");

    // Ensure there are enough buffers to avoid dropping frames
    if (camera_video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      camera_video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
  }

  format = camera_still_port->format;

  // Set our stills format on the stills (for encoder) port
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = width;
  format->es->video.height = height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = width;
  format->es->video.crop.height = height;
  format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
  format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;

  status = mmal_port_format_commit(camera_still_port);

  if (status)
  {
    fprintf(stderr, "format couldn't be set\n");
    mmal_component_destroy(camera);
    exit(EXIT_FAILURE);
  }

  /* Ensure there are enough buffers to avoid dropping frames */
  if (camera_still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    camera_still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


  /* Enable component */
  status = mmal_component_enable(camera);
  if (status)
  {
    fprintf(stderr, "camera couldn't be enabled\n");
    mmal_component_destroy(camera);
    exit(EXIT_FAILURE);
  }

  /* Enable component */
  status = mmal_component_enable(preview);
  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "Unable to enable preview/null sink component (%u)\n", status);
    mmal_component_destroy(preview);
    mmal_component_destroy(camera);
    exit(EXIT_FAILURE);
  }

  DBG("Camera enabled, creating encoder\n");

  //Create Encoder
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "Unable to create JPEG encoder component\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }

  if (!encoder->input_num || !encoder->output_num)
  {
    fprintf(stderr, "Unable to create JPEG encoder input/output ports\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }

  encoder_input = encoder->input[0];
  encoder_output = encoder->output[0];


  // We want same format on input and output
  mmal_format_copy(encoder_output->format, encoder_input->format);

  // Specify out output format JPEG
  encoder_output->format->encoding = MMAL_ENCODING_JPEG;

  encoder_output->buffer_size = encoder_output->buffer_size_recommended;


  if (encoder_output->buffer_size < encoder_output->buffer_size_min)
    encoder_output->buffer_size = encoder_output->buffer_size_min;

  fprintf(stderr,"Encoder Buffer Size %i\n", encoder_output->buffer_size);

  encoder_output->buffer_num = encoder_output->buffer_num_recommended;

  if (encoder_output->buffer_num < encoder_output->buffer_num_min)
    encoder_output->buffer_num = encoder_output->buffer_num_min;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(encoder_output);

  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "Unable to set video format output ports\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }

  // Set the JPEG quality level
  status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, quality);

  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "Unable to set JPEG quality\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }


  // Enable encoder component
  status = mmal_component_enable(encoder);

  if (status)
  {
    fprintf(stderr, "Unable to enable encoder component\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }


  DBG("Encoder enabled, creating pool and connecting ports\n");

  /* Create pool of buffer headers for the output port to consume */
  pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

  if (!pool)
  {
    fprintf(stderr, "Can't create buffer header pool for encoder output port\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }


  // Set up optional splitter component

  status = MMAL_SUCCESS;
  if (splitter_callback_data.detect_yuv || splitter_callback_data.write_images)
  {
      splitter_pair = create_splitter_component(camera);
      if (splitter_pair.component_ptr == NULL) status = MMAL_ENOSYS;
  }

  if (status == MMAL_SUCCESS) {
    if (wantPreview)
    {
      if (splitter_callback_data.detect_yuv ||
          splitter_callback_data.write_images) {
        // Connect camera preview output to splitter input
  
        status = connect_ports(camera_preview_port,
                               splitter_pair.component_ptr->input[0],
                               &splitter_connection);
  
        if (status != MMAL_SUCCESS)
        {
           LOG_ERROR("Can't connect camera preview output to splitter input");
           log_mmal_status(status);
        } else {
  
           // Connect splitter preview output to preview input
  
           status = connect_ports(
                   splitter_pair.component_ptr->output[SPLITTER_PREVIEW_PORT],
                   preview_input_port, &preview_connection);
           if (status != MMAL_SUCCESS)
           {
             LOG_ERROR("Can't connect splitter preview output to preview input");
             log_mmal_status(status);
           }
        }
  
      } else {
        // Connect camera to preview
        status = connect_ports(camera_preview_port, preview_input_port,
                               &preview_connection);
        if (status != MMAL_SUCCESS)
        {
          LOG_ERROR("Can't connect camera to preview input");
          log_mmal_status(status);
        }
      }
    } else {
      if (splitter_callback_data.detect_yuv ||
          splitter_callback_data.write_images)
      {
        // Connect camera to splitter
        status = connect_ports(camera_preview_port,
                               splitter_pair.component_ptr->input[0],
                               &splitter_connection);
  
        if (status != MMAL_SUCCESS)
        {
          LOG_ERROR("Can't connect camera preview port to splitter input");
          log_mmal_status(status);
        }
      } else {
        // Camera preview port is not used.
        status = MMAL_SUCCESS;
      }
    }
  } 

  // Now connect the camera to the encoder
  if (status == MMAL_SUCCESS) {
    if(usestills){
      status = connect_ports(camera_still_port, encoder->input[0], &encoder_connection);
    } else {
      status = connect_ports(camera_video_port, encoder->input[0], &encoder_connection);
    }
  }

  if (status == MMAL_SUCCESS && splitter_connection != NULL) {
    splitter_callback_data.pool_ptr = splitter_pair.pool_ptr;
    splitter_callback_port =
                splitter_pair.component_ptr->output[SPLITTER_CALLBACK_PORT];
    splitter_callback_port->userdata =
                      (struct MMAL_PORT_USERDATA_T *)&splitter_callback_data;
    status = mmal_port_enable(splitter_callback_port, splitter_buffer_callback);
    if (status != MMAL_SUCCESS)
    {
      LOG_ERROR("Can't enable splitter callback port");
    }
  }

  if (status)
  {
    if (preview_connection) {
      mmal_connection_destroy(preview_connection);
    }
    if (splitter_connection) {
      mmal_connection_destroy(splitter_connection);
    }
    if (encoder_connection) {
      mmal_connection_destroy(encoder_connection);
    }
    if (preview) {
      mmal_component_destroy(preview);
    }
    mmal_component_destroy(camera);
    if (encoder) {
      mmal_component_destroy(encoder);
    }
    exit(EXIT_FAILURE);
  }

  // Set up our userdata - this is passed though to the callback where we need the information.
  // Null until we open our filename
  PORT_USERDATA callback_data;
  callback_data.file_handle = NULL;
  callback_data.pool = pool;
  callback_data.offset = 0;
  callback_data.frame_no = 0;
  callback_data.width = width;   // width of encode image
  callback_data.height = height; // height of encode image
  callback_data.splitter_data_ptr = &splitter_callback_data;

  vcos_assert(vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0) == VCOS_SUCCESS);

  encoder->output[0]->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;



  // Enable the encoder output port and tell it its callback function
  status = mmal_port_enable(encoder->output[0], encoder_buffer_callback);
  if (status)
  {
    fprintf(stderr, "Unable to enable encoder component\n");
    mmal_component_destroy(camera);
    if (encoder)
      mmal_component_destroy(encoder);
    exit(EXIT_FAILURE);
  }

  if(usestills){
    DBG("Starting stills output\n");

    //setup fps
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    frames = 0;
    int delay = (1000 * 1000) / fps;

    while(!pglobal->stop) {
      //Wait the delay
      usleep(delay);

      // Send all the buffers to the encoder output port
      int num = mmal_queue_length(pool->queue);
      int q;
      for (q=0;q<num;q++)
      {
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

        if (!buffer)
          fprintf(stderr, "Unable to get a required buffer from pool queue");

        if (mmal_port_send_buffer(encoder->output[0], buffer)!= MMAL_SUCCESS)
          fprintf(stderr, "Unable to send a buffer to encoder output port");
      }

      if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
      {
        fprintf(stderr, "starting capture failed");
      }
      else
      {
        // Wait for capture to complete
        // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
        // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
        vcos_semaphore_wait(&callback_data.complete_semaphore);
        //DBG("Jpeg Captured\n");
        frames++;
      }

      frames++;
      if (frames == 100)
      {
        //calculate fps
        clock_gettime(CLOCK_MONOTONIC, &t_finish);
        t_elapsed = (t_finish.tv_sec - t_start.tv_sec);
        t_elapsed += (t_finish.tv_nsec - t_start.tv_nsec) / 1000000000.0;
        fprintf(stderr, "%i frames captured in %f seconds (%f fps)\n", frames, t_elapsed, (frames / t_elapsed));
        frames = 0;
        clock_gettime(CLOCK_MONOTONIC, &t_start);
      }
    }

  }
  else
  { //if(usestills)
    //Video Mode
    DBG("Starting video output\n");
    // Send all the buffers to the encoder output port
    int num = mmal_queue_length(pool->queue);
    int q;
    for (q=0;q<num;q++)
    {
      MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
      if (!buffer)
      {
        LOG_ERROR("Unable to get a required buffer from pool queue");
      }
      if (mmal_port_send_buffer(encoder->output[0], buffer)!= MMAL_SUCCESS)
      {
        LOG_ERROR("Unable to send a buffer to encoder output port");
      }
    }
    if (splitter_callback_port != NULL)
    {
      int num = mmal_queue_length(splitter_callback_data.pool_ptr->queue);
      int q;
      for (q = 0; q < num; q++)
      {
         MMAL_BUFFER_HEADER_T *buffer =
                       mmal_queue_get(splitter_callback_data.pool_ptr->queue);

         if (!buffer) {
            LOG_ERROR("Unable to get a required buffer %d from pool queue", q);
         }

         if (mmal_port_send_buffer(splitter_callback_port, buffer)
                                                            != MMAL_SUCCESS) {
            LOG_ERROR("Unable to send a buffer to splitter output port (%d)", q);
         }
      }
    }
    if (mmal_port_parameter_set_boolean(
                camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
      LOG_ERROR("Can't start capture");
    }

    while(!pglobal->stop) usleep(1000);
  }

  vcos_semaphore_delete(&callback_data.complete_semaphore);

  //Close everything MMAL
  if (usestills)
  {
    if (camera_video_port && camera_video_port->is_enabled)
      mmal_port_disable(camera_video_port);
  }
  else
  {
    if (camera_still_port && camera_still_port->is_enabled)
      mmal_port_disable(camera_still_port);
  }
  if (preview_connection)
    mmal_connection_destroy(preview_connection);

  if (encoder->output[0] && encoder->output[0]->is_enabled)
    mmal_port_disable(encoder->output[0]);

  mmal_connection_destroy(encoder_connection);

  // Disable components
  if (encoder)
    mmal_component_disable(encoder);
  if (preview)
    mmal_component_disable(preview);
  if (camera)
    mmal_component_disable(camera);

  //Destroy encoder component
  // Get rid of any port buffers first
  if (pool)
  {
    mmal_port_pool_destroy(encoder->output[0], pool);
  }

  if (encoder)
  {
    mmal_component_destroy(encoder);
    encoder = NULL;
  }
  if (preview)
  {
    mmal_component_destroy(preview);
    preview = NULL;
  }
  //destroy camera component
  if (camera)
  {
    mmal_component_destroy(camera);
    camera = NULL;
  }

  DBG("mmal cleanup done\n");
  pthread_cleanup_pop(1);

  return NULL;
}

/******************************************************************************
  Description.: this functions cleans up allocated resources
  Input Value.: arg is unused
  Return Value: -
 ******************************************************************************/
void worker_cleanup(void *arg)
{
  static unsigned char first_run = 1;

  if (!first_run)
  {
    DBG("already cleaned up resources\n");
    return;
  }

  first_run = 0;
  DBG("cleaning up resources allocated by input thread\n");

  if(pglobal->in[plugin_number].buf != NULL)
    free(pglobal->in[plugin_number].buf);
}
