/* Copyright (c) 2015, 2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*************************************************************************
*
*Application Notes:
*
*Camera selection:
*  Each camera is given a unique function id in the sensor driver.
*   LEFT SENSOR = 0, RIGHT SENSOR = 1, STEREO SENSOR = 2,
*  getNumberOfCameras gives information on number of camera connected on target.
*  getCameraInfo provides information on each camera loop.
*  camid is obtained by looping through available cameras and matching info.func
*  with the requested camera.
*
****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <syslog.h>
#include <inttypes.h>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <sys/stat.h>

#define TAG "LIBCAM-BIN"
#include <stdio.h>
#include <android/log.h>

// System dependencies
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
//#define STAT_H <SYSTEM_HEADER_PREFIX/stat.h>
//#include STAT_H
#include <utils/Errors.h>
#include <thread>


#if 0
#define CAM_ERR(fmt, args...) do { \
  __android_log_print(ANDROID_LOG_ERROR,TAG,fmt, ##args); \
} while (0)
#else
#define CAM_ERR printf
#endif

#include "camera.h"
#include "camera_parameters.h"

#define DEFAULT_EXPOSURE_VALUE 250
#define MIN_EXPOSURE_VALUE 1
#define MAX_EXPOSURE_VALUE 65535
#define DEFAULT_GAIN_VALUE  300
#define MIN_GAIN_VALUE 256
#define MAX_GAIN_VALUE 2048
#define DEFAULT_EXPOSURE_TIME_VALUE "0"

#define QCAMERA_DUMP_LOCATION "data/vendor/camera/"

#define DEFAULT_CAMERA_FPS 30
#define MS_PER_SEC 1000
#define NS_PER_MS 1000000
#define NS_PER_US 1000
#define MAX_BUF_SIZE 128

const int SNAPSHOT_WIDTH_ALIGN = 64;
const int SNAPSHOT_HEIGHT_ALIGN = 64;
const int TAKEPICTURE_TIMEOUT_MS = 5000;

using namespace std;
using namespace camera;

#define MAX_CAM 2

struct CameraCaps
{
    vector<ImageSize> pSizes, vSizes, picSizes,rawPicSizes;
    vector<string> focusModes, wbModes, isoModes;
    Range brightness, sharpness, contrast;
    vector<Range> previewFpsRanges;
    vector<string> previewFormats;
    string rawSize;
    Range64 exposureRange;
    Range gainRange;
};

enum OutputFormatType{
    YUV_FORMAT,
    RAW_FORMAT,
    JPEG_FORMAT
};

enum CamFunction {
    CAM_FUNC_LEFT_SENSOR = 0,       //left ov
    CAM_FUNC_RIGHT_SENSOR = 1,      //right ov
    CAM_FUNC_STEREO = 2,            //stereo
    CAM_FUNC_MAX,
};

enum AppLoglevel {
    CAM_LOG_SILENT = 0,
    CAM_LOG_ERROR = 1,
    CAM_LOG_INFO = 2,
    CAM_LOG_DEBUG = 3,
    CAM_LOG_MAX,
};

/**
*  Helper class to store all parameter settings
*/
struct TestConfig
{
    bool dumpFrames;
    bool infoMode;
    bool testSnapshot;
    int runTime;
    int exposureValue;
    int gainValue;
    string expTimeValue;
    string isoValue;
    CamFunction func;
    OutputFormatType outputFormat;
    OutputFormatType snapshotFormat;
    ImageSize pSize;
    ImageSize vSize;
    ImageSize picSize;
    int fps;
    AppLoglevel logLevel;
    int statsLogMask;
    uint32_t num_images;
    string antibanding;
    bool is_interact;
};

/**
 * CLASS  CameraTest
 *
 * - inherits ICameraListers which provides core functionality
 * - User must define onPreviewFrame (virtual) function. It is
 *    the callback function for every preview frame.
 * - If user is using VideoStream then the user must define
 *    onVideoFrame (virtual) function. It is the callback
 *    function for every video frame.
 * - If any error occurs,  onError() callback function is
 *    called. User must define onError if error handling is
 *    required.
 */
class CameraTest : ICameraListener
{
public:

    CameraTest();
    CameraTest(TestConfig config);
    ~CameraTest();
    int run();

    int initialize(int camId);

    /* listener methods */
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);
    virtual void onPictureFrame(ICameraFrame* frame);

private:
    ICameraDevice* camera_[MAX_CAM];
    CameraParams params_[MAX_CAM];
    ImageSize pSize_[MAX_CAM];
    ImageSize picSize_[MAX_CAM];
    CameraCaps caps_[MAX_CAM];
    TestConfig config_;
    vector<int> camArray;

    uint32_t sFrameCount_[MAX_CAM], pFrameCount_[MAX_CAM];
    float pFpsAvg_[MAX_CAM];

    uint64_t pTimeStampPrev_[MAX_CAM];

    pthread_cond_t cvPicDone[MAX_CAM];
    pthread_mutex_t mutexPicDone[MAX_CAM];
    pthread_mutex_t mutexSeq;
    bool isPicDone[MAX_CAM];

    int printCapabilities(int camId);
    int setParameters(int camId);
    int takePicture(uint32_t num_images = 1);
    int setFPSindex(TestConfig& cfg, int camId, int &pFpsIdx);
};

CameraTest::CameraTest()
{
    for (int i = 0; i < MAX_CAM; i++) {
        camera_[i] = NULL;
        sFrameCount_[i] = 0;
        pFrameCount_[i] = 0;
        pFpsAvg_[i] = 0.0f;
        pTimeStampPrev_[i] = 0;
        pthread_cond_init(&cvPicDone[i], NULL);
        pthread_mutex_init(&mutexPicDone[i], NULL);
    }
    pthread_mutex_init(&mutexSeq, NULL);
}

CameraTest::CameraTest(TestConfig config)
{
    for (int i = 0; i < MAX_CAM; i++) {
        camera_[i] = NULL;
        sFrameCount_[i] = 0;
        pFrameCount_[i] = 0;
        pFpsAvg_[i] = 0.0f;
        pTimeStampPrev_[i] = 0;
        pthread_cond_init(&cvPicDone[i], NULL);
        pthread_mutex_init(&mutexPicDone[i], NULL);
    }
    pthread_mutex_init(&mutexSeq, NULL);
    config_ = config;
    switch    (config_.func) {
        case CAM_FUNC_LEFT_SENSOR:
            camArray.push_back(0);
            break;
        case CAM_FUNC_RIGHT_SENSOR:
            camArray.push_back(1);
            break;
        case CAM_FUNC_STEREO:
            camArray.push_back(0);
            camArray.push_back(1);
            break;
        default:
            camArray.push_back(0);
            break;
    }
}

int CameraTest::initialize(int camId)
{
    int rc;
    rc = ICameraDevice::createInstance(camId, &(camera_[camId])); //open camera here
    if (rc != 0) {
        printf("could not open camera %d, rc %d\n", camId, rc);
        return rc;
    }
    camera_[camId]->addListener(this);

    rc = params_[camId].init(camera_[camId]);
    if (rc != 0) {
        printf("failed to init parameters\n");
        ICameraDevice::deleteInstance(&(camera_[camId]));
        return rc;
    }

    //printf("params = %s\n", params_.toString().c_str());

    /* query capabilities */
    caps_[camId].pSizes = params_[camId].getSupportedPreviewSizes();
    caps_[camId].picSizes = params_[camId].getSupportedPictureSizes(FORMAT_JPEG);
    caps_[camId].rawPicSizes = params_[camId].getSupportedPictureSizes(FORMAT_RAW10);
    caps_[camId].focusModes = params_[camId].getSupportedFocusModes();
    caps_[camId].wbModes = params_[camId].getSupportedWhiteBalance();
    caps_[camId].isoModes = params_[camId].getSupportedISO();
    caps_[camId].brightness = params_[camId].getSupportedBrightness();
    caps_[camId].sharpness = params_[camId].getSupportedSharpness();
    caps_[camId].contrast = params_[camId].getSupportedContrast();
    caps_[camId].previewFpsRanges = params_[camId].getSupportedPreviewFpsRanges();
    caps_[camId].previewFormats = params_[camId].getSupportedPreviewFormats();
    caps_[camId].rawSize = params_[camId].get("raw-size");

    return 0;
}

CameraTest::~CameraTest()
{
    pthread_mutex_destroy(&mutexSeq);
    for (int i = 0; i < MAX_CAM; i++) {
        pthread_cond_destroy(&cvPicDone[i]);
        pthread_mutex_destroy(&mutexPicDone[i]);
    }
}

static int dumpToFile(uint8_t* data, uint32_t size, char* name, uint64_t timestamp)
{
    FILE* fp;
    //fp = fopen(name, "wb");
    int file_fd = open(name, O_RDWR | O_CREAT, 0777);
    fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (file_fd < 0) {
        printf("fopen failed for %s\n", name);
        return -1;
    }
    write(file_fd, data, (size_t)size);
    //fwrite(data, size, 1, fp);
    printf("saved filename %s\n", name);
    //fclose(fp);
    close(file_fd);
    return 0;
}

static inline uint32_t align_size(uint32_t size, uint32_t align)
{
    return ((size + align - 1) & ~(align-1));
}

#if 0
void request_process_thread(ICameraDevice *device, int camId, TestConfig cfg, pthread_mutex_t *mutex)
{
    int rc = 0;
    if (cfg.func == CAM_FUNC_STEREO) {
        if (camId == CAM_FUNC_LEFT_SENSOR) {
            pthread_mutex_unlock(mutex);
        } else {
            pthread_mutex_lock(mutex);
        }
    }
    printf("thread: takePicture for %d start\n", camId);
    rc = device->takePicture();
    if (rc) {
        printf("takePicture failed for %d\n", camId);
        return;
    }
    printf("thread: takePicture for %d finish\n", camId);
}

int CameraTest::takePicture(uint32_t num_images)
{
    int rc;
    int camId;
    std::thread *request_thread[2];

    pthread_mutex_lock(&mutexSeq);
    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        printf("take picture camId %d\n", camId);
        request_thread[camId] = new std::thread(request_process_thread,
          camera_[camId], camId, config_, &mutexSeq);
    }

    sleep(1);

    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        request_thread[camId]->join();
    }

    return 0;
}
#else
#if 1
int CameraTest::takePicture(uint32_t num_images)
{
    int rc;
    int camId;
    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        printf("take picture camId %d\n", camId);
        rc = camera_[camId]->takePicture(num_images);
        if (rc) {
            printf("camId %d takePicture failed\n", camId);
            return rc;
        }
    }
    return 0;
}
#else
int CameraTest::takePicture(uint32_t num_images)
{
    int rc;
    pthread_mutex_lock(&mutexPicDone);
    isPicDone = false;
    printf("take picture\n");
    rc = camera_->takePicture(num_images);
    if (rc) {
        printf("takePicture failed\n");
        pthread_mutex_unlock(&mutexPicDone);
        return rc;
    }

    struct timespec waitTime;
    struct timeval now;

    gettimeofday(&now, NULL);
    waitTime.tv_sec = now.tv_sec + TAKEPICTURE_TIMEOUT_MS/MS_PER_SEC;
    waitTime.tv_nsec = now.tv_usec * NS_PER_US + (TAKEPICTURE_TIMEOUT_MS % MS_PER_SEC) * NS_PER_MS;
    /* wait for picture done */
    while (isPicDone == false) {
        rc = pthread_cond_timedwait(&cvPicDone, &mutexPicDone, &waitTime);
        if (rc == ETIMEDOUT) {
            printf("error: takePicture timed out\n");
            break;
        }
    }
    pthread_mutex_unlock(&mutexPicDone);
    return 0;
}
#endif
#endif

void CameraTest::onError()
{
    printf("camera error!, aborting\n");
    exit(EXIT_FAILURE);
}


string getStringFromEnum(CamFunction e)
{
  switch(e)
  {
  case CAM_FUNC_LEFT_SENSOR: return "left";
  case CAM_FUNC_RIGHT_SENSOR: return "right";
  case CAM_FUNC_STEREO: return "stereo";
  default:
      printf("error: unknown camera type \n");
  }

  return "unknown";
}

/**
 *
 * FUNCTION: onPreviewFrame
 *
 *  - This is called every frame I
 *  - In the test app, we save files only after every 30 frames
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void CameraTest::onPreviewFrame(ICameraFrame* frame)
{
    int camidx = frame->camIdx;
    uint64_t diff = frame->timeStamp - pTimeStampPrev_[camidx];
    pFpsAvg_[camidx] = ((pFpsAvg_[camidx] * pFrameCount_[camidx]) + (1e9 / diff)) / (pFrameCount_[camidx] + 1);
    pFrameCount_[camidx]++;
    pTimeStampPrev_[camidx]  = frame->timeStamp;

    if (pFrameCount_[camidx] > 0 && pFrameCount_[camidx] % 20 == 0) {
        char name[MAX_BUF_SIZE];

        snprintf(name, MAX_BUF_SIZE, QCAMERA_DUMP_LOCATION "preview_%dx%d_%04d_%llu_%s.yuv",
          pSize_[camidx].width, pSize_[camidx].height, pFrameCount_[camidx], frame->timeStamp,
          camidx==0?"L":"R");

        if (config_.dumpFrames == true) {
            dumpToFile(frame->data, frame->size, name, frame->timeStamp);
        }
    }
}

void CameraTest::onPictureFrame(ICameraFrame* frame)
{
    char imageName[MAX_BUF_SIZE];
    int camidx = frame->camIdx;

    snprintf(imageName, MAX_BUF_SIZE, QCAMERA_DUMP_LOCATION "snapshot_%dx%d_%lld_%d_%s.jpg",
      picSize_[camidx].width, picSize_[camidx].height, frame->timeStamp, sFrameCount_[camidx],
      camidx==0?"L":"R");
    sFrameCount_[camidx]++;

    dumpToFile(frame->data, frame->size, imageName, frame->timeStamp);
    /* notify the waiting thread about picture done */
    pthread_mutex_lock(&mutexPicDone[camidx]);
    isPicDone[camidx] = true;
    pthread_cond_signal(&cvPicDone[camidx]);
    pthread_mutex_unlock(&mutexPicDone[camidx]);
    printf("%s:%d\n", __func__, __LINE__);
}

/**
 *
 * FUNCTION: onVideoFrame
 *
 *  - This is called every frame I
 *  - In the test app, we save files only after every 30 frames
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void CameraTest::onVideoFrame(ICameraFrame* frame)
{
    return;
}

int CameraTest::printCapabilities(int camId)
{
    printf("Camera capabilities\n");

    printf("available preview sizes:\n");
    for (size_t i = 0; i < caps_[camId].pSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_[camId].pSizes[i].width, caps_[camId].pSizes[i].height);
    }
    printf("available jpeg picture sizes:\n");
    for (size_t i = 0; i < caps_[camId].picSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_[camId].picSizes[i].width, caps_[camId].picSizes[i].height);
    }
    printf("available preview formats:\n");
    for (size_t i = 0; i < caps_[camId].previewFormats.size(); i++) {
        printf("%d: %s\n", i, caps_[camId].previewFormats[i].c_str());
    }
    printf("available whitebalance modes:\n");
    for (size_t i = 0; i < caps_[camId].wbModes.size(); i++) {
        printf("%d: %s\n", i, caps_[camId].wbModes[i].c_str());
    }
    printf("available ISO modes:\n");
    for (size_t i = 0; i < caps_[camId].isoModes.size(); i++) {
        printf("%d: %s\n", i, caps_[camId].isoModes[i].c_str());
    }
    printf("available brightness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_[camId].brightness.min,
           caps_[camId].brightness.max, caps_[camId].brightness.step);
    printf("available sharpness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_[camId].sharpness.min,
           caps_[camId].sharpness.max, caps_[camId].sharpness.step);
    printf("available contrast values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_[camId].contrast.min,
           caps_[camId].contrast.max, caps_[camId].contrast.step);

    printf("available preview fps ranges:\n");
    for (size_t i = 0; i < caps_[camId].previewFpsRanges.size(); i++) {
        printf("%d: [%d, %d]\n", i, caps_[camId].previewFpsRanges[i].min,
               caps_[camId].previewFpsRanges[i].max);
    }

    return 0;
}

ImageSize i5MSize(2592, 1944);
ImageSize FHDSize(1920,1080);
ImageSize HDSize(1280,720);
ImageSize VGASize(640,480);
ImageSize QVGASize(320,240);


const char usageStr[] =
    "Camera API test application \n"
    "\n"
    "usage: camera-test-stereo [options]\n"
    "\n"
    "  -t <duration>   capture duration in seconds [10]\n"
    "  -d              dump frames\n"
    "  -i              info mode\n"
    "                    - print camera capabilities\n"
    "                    - streaming will not be started\n"
    "  -f <type>       camera type\n"
    "                    - left \n"
    "                    - right \n"
    "                    - stereo \n"
    "  -s <size>       take pickture at set resolution ( disabled by default) \n"
    "                    - 5M            ( max picture resolution supported by sensor"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of tracking camera and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "  -V <level>      syslog level [0]\n"
    "                    0: silent\n"
    "                    1: error\n"
    "                    2: info\n"
    "                    3: debug\n"
    "  -h              print this message\n"
;

static inline void printUsageExit(int code)
{
    printf("%s", usageStr);
    exit(code);
}

enum Commands_e {
    TAKEPICTURE_CMD = 'p',
    EXIT_CMD = 'q',
    INVALID_CMD = '0'
};

/*===========================================================================
 * FUNCTION   : printMenu
 *
 * DESCRIPTION: prints the available camera options
 *
 * PARAMETERS :
 *  @currentCamera : camera context currently being used
 *
 * RETURN     : None
 *==========================================================================*/
void printMenu()
{
    printf("\n\n=========== FUNCTIONAL TEST MENU ===================\n\n");

    printf("   %c. Take picture\n", TAKEPICTURE_CMD);
    printf("   %c. Quit \n", EXIT_CMD);

    printf("\n   Choice: ");
}

/**
 * FUNCTION: setFPSindex
 *
 * scans through the supported fps values and returns index of
 * requested fps in the array of supported fps
 *
 * @param fps      : Required FPS  (Input)
 * @param pFpsIdx  : preview fps index (output)
 *
 *  */
int CameraTest::setFPSindex(TestConfig & cfg, int camId, int &pFpsIdx)
{
    int defaultPrevFPSIndex = -1;
    size_t i;
    int rc = 0;
    int preview_fps = cfg.fps;
    for (i = 0; i < caps_[camId].previewFpsRanges.size(); i++) {
        //printf("caps_[camId].previewFpsRanges[i] %d, preview_fps %d\n", caps_[camId].previewFpsRanges[i].max/1000, preview_fps);
        if (  (caps_[camId].previewFpsRanges[i].max)/1000 == preview_fps )
        {
            pFpsIdx = i;
            break;
        }
        if ( (caps_[camId].previewFpsRanges[i].max)/1000 == DEFAULT_CAMERA_FPS )
        {
            defaultPrevFPSIndex = i;
        }
    }
    if ( i >= caps_[camId].previewFpsRanges.size() )
    {
        if (defaultPrevFPSIndex != -1 )
        {
            pFpsIdx = defaultPrevFPSIndex;
        } else
        {
            pFpsIdx = -1;
            rc = -1;
        }
    }
    return rc;
}
/**
 *  FUNCTION : setParameters
 *
 *  - When camera is opened, it is initialized with default set
 *    of parameters.
 *  - This function sets required parameters based on camera and
 *    usecase
 *  - params_setXXX and params_set  only updates parameter
 *    values in a local object.
 *  - params_.commit() function will update the hardware
 *    settings with the current state of the parameter object
 *  - Some functionality will not be application for all for
 *    sensor modules. for eg. tracking camera sensor does not support
 *    autofocus/focus mode.
 *  - Reference setting for different sensors and format are
 *    provided in this function.
 *
 *  */
int CameraTest::setParameters(int camId)
{
    size_t index;
    int focusModeIdx = 0;
    int pFpsIdx = 3;
    vector<ImageSize> supportedSnapshotSizes;

    pSize_[camId] = config_.pSize;
    picSize_[camId] = config_.picSize;
    printf("camId %d config_.func %d\n", camId, config_.func);
    CAM_ERR("pSize_.wxh %dx%d picSize_.wxh %dx%d\n",
        pSize_[camId].width, pSize_[camId].height, picSize_[camId].width, picSize_[camId].height);
    switch ( camId ){
        case 0:
            params_[camId].set("ae-bracket-hdr", "AE-Bracket");
            params_[camId].set("capture-burst-exposures", "0,-4,+4");
        case 1:
            params_[camId].set("preview-format", "yuv420sp");
            params_[camId].set("zsl", "on");
            params_[camId].set("no-display-mode", "1");
            printf("setting picture size: %dx%d\n",
                    picSize_[camId].width, picSize_[camId].height);
            params_[camId].setPictureFormat(FORMAT_JPEG);
            supportedSnapshotSizes = caps_[camId].picSizes;
            if (config_.func == CAM_FUNC_STEREO) {
                printf("set dual camera mode for camId %d\n", camId);
                if (camId == 0) {
                    params_[camId].set("dual-camera-mode", "ON");
                    params_[camId].set("dual-camera-id", "1");
                    params_[camId].set("dual-camera-main-camera", "true");
                } else {
                    params_[camId].set("dual-camera-mode", "ON");
                    params_[camId].set("dual-camera-id", "0");
                    params_[camId].set("dual-camera-main-camera", "false");
                }
            }

            for ( index = 0 ; index < supportedSnapshotSizes.size() ; index++) {
                if ( config_.picSize.width == supportedSnapshotSizes[index].width && config_.picSize.height == supportedSnapshotSizes[index].height )
                {
                    picSize_[camId] = supportedSnapshotSizes[index];
                    break;
                }
            }
            if ( index >= supportedSnapshotSizes.size() ) {
                printf("Error: Snapshot resolution %d x %d not supported for requested format \n",config_.picSize.width,config_.picSize.height);
                exit(1);
            }

            params_[camId].setPictureSize(picSize_[camId]);
            printf("Setting snapshot size : %d x %d \n", picSize_[camId].width, picSize_[camId].height );

            if (config_.snapshotFormat == JPEG_FORMAT) {
                params_[camId].setPictureThumbNailSize(picSize_[camId]);
            }

            break;
        default:
            printf("invalid sensor function \n");
            break;
    }

    printf("setting preview size: %dx%d\n", pSize_[camId].width, pSize_[camId].height);
    params_[camId].setPreviewSize(pSize_[camId]);

    /* Find index and set FPS  */
    int rc = setFPSindex(config_, camId, pFpsIdx);
    if ( rc == -1) {
        return rc;
    }
    printf("setting preview fps range: %d, %d ( idx = %d ) \n",
        caps_[camId].previewFpsRanges[pFpsIdx].min,
        caps_[camId].previewFpsRanges[pFpsIdx].max, pFpsIdx);
    params_[camId].setPreviewFpsRange(caps_[camId].previewFpsRanges[pFpsIdx]);

    return params_[camId].commit();
}

int CameraTest::run()
{
    int camId = 0;
    int rc = EXIT_SUCCESS;
    /* returns the number of camera-modules connected on the board */
    int n = getNumberOfCameras();

    if (n < 0) {
        printf("getNumberOfCameras() failed, rc=%d\n", n);
        return EXIT_FAILURE;
    }
    printf("num_cameras = %d\n", n);

    /* find camera based on function */
    for (int i=0; i<MAX_CAM; i++) {
        CameraInfo info;
        getCameraInfo(i, info);
    }

    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        printf("start run camera id %d\n", camId);
    }

    if (n < 1) {
        printf("No cameras found.\n");
        return EXIT_FAILURE;
    }

    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        printf("initialize camId %d\n", camId);
        rc = initialize(camId);
        if (0 != rc) {
            return rc;
        }
    }

    if (config_.infoMode) {
        for (int i=0; i<camArray.size(); i++) {
            camId = camArray[i];
            printf("camId %d\n", camId);
            printCapabilities(camId);
        }
        return rc;
    }

    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        /* initialize perf counters */
        sFrameCount_[camId] = 0;
        pFrameCount_[camId] = 0;
        pFpsAvg_[camId] = 0.0f;
        rc = setParameters(camId);
        if (rc) {
            printf("setParameters failed\n");
            printUsageExit(0);
            goto del_camera;
        }
    }

    /* starts the preview stream. At every preview frame onPreviewFrame( ) callback is invoked */
    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        printf("start preview camId %d\n", camId);
        rc = camera_[camId]->startPreview();
        printf("start preview camId %d rc %d\n", camId, rc);
    }

    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        rc = params_[camId].commit();
        if (rc) {
            printf("commit failed\n");
            exit(EXIT_FAILURE);
        }
    }

    if (config_.is_interact) {
        bool is_running = true;
        Commands_e command;
        while (is_running) {
            sleep(1);
            printMenu();
            command = static_cast<Commands_e>(getchar());
            while ((getchar()) != '\n');
            switch (command) {
            case TAKEPICTURE_CMD:
            {
                printf("taking picture\n");
                rc = takePicture(config_.num_images);

                if (rc) {
                    printf("takePicture failed\n");
                    exit(EXIT_FAILURE);
                }
            }
            break;
            case EXIT_CMD:
            {
                printf("exit\n");
                is_running = false;
            }
            default:
            printf("invalid command\n");
            }
        }
    } else if (config_.testSnapshot == true) {
        printf("waiting for 3 seconds for exposure to settle...\n");
        /* sleep required to settle the exposure before taking snapshot.
           This app does not provide interactive feedback to user
           about the exposure */
        sleep(3);
        printf("taking picture\n");
        rc = takePicture(config_.num_images);

        if (rc) {
            printf("takePicture failed\n");
            exit(EXIT_FAILURE);
        }
    }


    /* Put the main/run thread to sleep and process the frames in the callbacks */
    printf("waiting for %d seconds ...\n", config_.runTime);
    sleep(config_.runTime);

    printf("stop preview\n");
    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        camera_[camId]->stopPreview();
    }
    //printf("Average preview FPS = %.2f\n", pFpsAvg_[camId]);

del_camera:
    /* release camera device */
    for (int i=0; i<camArray.size(); i++) {
        camId = camArray[i];
        ICameraDevice::deleteInstance(&camera_[camId]);
    }
    return rc;
}

/**
 *  FUNCTION: setDefaultConfig
 *
 *  set default config based on camera module
 *
 * */
static int setDefaultConfig(TestConfig &cfg) {

    cfg.outputFormat = YUV_FORMAT;
    cfg.dumpFrames = false;
    cfg.runTime = 10;
    cfg.infoMode = false;
    cfg.testSnapshot = false;
    cfg.exposureValue = DEFAULT_EXPOSURE_VALUE;  /* Default exposure value */
    cfg.gainValue = DEFAULT_GAIN_VALUE;  /* Default gain value */
    cfg.expTimeValue = DEFAULT_EXPOSURE_TIME_VALUE;
    cfg.isoValue = "auto";
    cfg.fps = DEFAULT_CAMERA_FPS;
    cfg.logLevel = CAM_LOG_DEBUG;
    cfg.snapshotFormat = JPEG_FORMAT;
    cfg.statsLogMask = STATS_NO_LOG;
    cfg.num_images = 1;
    cfg.antibanding = "off";

    switch (cfg.func) {
    case CAM_FUNC_LEFT_SENSOR:
        cfg.pSize   = VGASize;
        cfg.picSize   = VGASize;
        break;
    case CAM_FUNC_RIGHT_SENSOR:
        cfg.pSize   = VGASize;
        cfg.picSize   = VGASize;
        break;
    case CAM_FUNC_STEREO:
        cfg.pSize = VGASize;
        cfg.picSize  = VGASize;
        break;
    default:
        printf("invalid sensor function \n");
        break;
    }
    return 0;
}

/**
 *  FUNCTION: parseCommandline
 *
 *  parses commandline options and populates the config
 *  data structure
 *
 *  */
static TestConfig parseCommandline(int argc, char* argv[])
{
    TestConfig cfg;
    cfg.func = CAM_FUNC_LEFT_SENSOR;

    int c;
    int outputFormat;

    while ((c = getopt(argc, argv, "hdt:i:s:f:V:")) != -1) {
        switch (c) {
        case 'f':
            {
                string str(optarg);
                if  (str == "left") {
                    cfg.func = CAM_FUNC_LEFT_SENSOR;
                } else if (str == "right") {
                    cfg.func = CAM_FUNC_RIGHT_SENSOR;
                } else if (str == "stereo") {
                    cfg.func = CAM_FUNC_STEREO;
                }
                break;
            }
        case '?':
            break;
        default:
            break;
        }
    }
    setDefaultConfig(cfg);

    optind = 1;
    while ((c = getopt(argc, argv, "hdt:i:s:f:V:")) != -1) {
        switch (c) {
        case 't':
            cfg.runTime = atoi(optarg);
            break;
       case 's':
            {
                string str(optarg);
                if (str == "5M") {
                    cfg.picSize = i5MSize;
                } else if (str == "1080p") {
                    cfg.picSize = FHDSize;
                } else if (str == "720p") {
                    cfg.picSize = HDSize;
                } else if (str == "VGA") {
                    cfg.picSize = VGASize;
                } else if (str == "QVGA") {
                    cfg.picSize = QVGASize;
                }
                cfg.testSnapshot = true;
                break;
            }
        case 'd':
            cfg.dumpFrames = true;
            break;
        case 'i':
            cfg.infoMode = true;
            break;
        case 'V':
            cfg.logLevel = (AppLoglevel)atoi(optarg);
            break;
        case 'f':
            break;
        case 'h':
        case '?':
            printUsageExit(0);
        default:
            abort();
        }
    }
    return cfg;
}

int main(int argc, char* argv[])
{
    TestConfig config;

    if (argc > 1) {
        config = parseCommandline(argc, argv);
        config.is_interact = false;
    } else {
        config.func = CAM_FUNC_STEREO;
        setDefaultConfig(config);
        config.picSize = i5MSize;
        config.dumpFrames = false;
        config.testSnapshot = true;
        config.is_interact = true;
    }

    /* setup syslog level */
    if (config.logLevel == CAM_LOG_SILENT) {
        setlogmask(LOG_UPTO(LOG_EMERG));
    } else if (config.logLevel == CAM_LOG_DEBUG) {
        setlogmask(LOG_UPTO(LOG_DEBUG));
    } else if (config.logLevel == CAM_LOG_INFO) {
        setlogmask(LOG_UPTO(LOG_INFO));
    } else if (config.logLevel == CAM_LOG_ERROR) {
        setlogmask(LOG_UPTO(LOG_ERR));
    }
    openlog(NULL, LOG_NDELAY, LOG_DAEMON);

    CameraTest test(config);
    test.run();

    return EXIT_SUCCESS;
}

