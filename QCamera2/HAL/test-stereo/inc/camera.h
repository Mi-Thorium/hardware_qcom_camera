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
#ifndef __CAMERA_H__
#define __CAMERA_H__


#include <stdint.h>
#include <string>
#include <vector>
#include <ostream>

#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)
#endif

namespace camera
{


/**
 * Type of control event received from camera
 */
enum CameraEventType
{
    CAMERA_EVT_NONE = 0x0000,
    CAMERA_EVT_FOCUS = 0x0001,
    CAMERA_EVT_ALL = 0xFFFF,
};


/**
 * data structure to represent a single control event
 **/
struct ControlEvent
{
    int ext1;
    int ext2;
    CameraEventType type;
};

#define MAX_FACES 10

struct FaceData
{
    int32_t rect[4];
    int32_t score;
    int32_t id;
    int32_t left_eye[2];
    int32_t right_eye[2];
    int32_t mouth[2];
    int32_t smile_degree;
    int32_t smile_score;
    int32_t blink_detected;
    int32_t face_recognised;
    int32_t gaze_angle;
    int32_t updown_dir;
    int32_t leftright_dir;
    int32_t roll_dir;
    int32_t left_right_gaze;
    int32_t top_bottom_gaze;
    int32_t leye_blink;
    int32_t reye_blink;
};

typedef struct _FaceRoi
{
  int32_t number_of_faces;
  FaceData  faces[MAX_FACES];
} FaceRoi;
/**
 * Interface to an object that represents single image/metadata
 * frame
 */
class ICameraFrame
{
    DISALLOW_COPY_AND_ASSIGN(ICameraFrame);
protected:
    uint32_t refs_;

    ICameraFrame() :
        camIdx(0),
        refs_(0),
        timeStamp(0),
        data(NULL),
        fd(0)
    {}

    virtual ~ICameraFrame() {}

public:

    /**
     * aquire a reference to the frame, this is required if client
     * wants to hold the frame for further processing after camera
     * callback returns
     *
     * @return uint32_t : number of refs to the frame.
     */
    virtual uint32_t acquireRef() = 0;

    /**
     * release reference to the frame object. This will release
     * memory associated with image data as well.
     *
     * @return uint32_t : number of refs to the frame
     */
    virtual uint32_t releaseRef() = 0;

    int camIdx;

    /**
     * frame timestamp
     */
    uint64_t timeStamp;

    /**
     * frame data size in bytes
     */
    uint32_t size;

    /**
     * pointer to start of image data
     */
    uint8_t* data;

    /**
     * file descriptor for the frame memory
     * If value is -1, it indicates the memory is not shared
     */
    int fd;

    /**
     * opaque metadata about the frame memory. This is used for
     * sharing memory between camera and video encoder
     */
    void *metadata;

    /**
     * pointer face data
     */
    FaceRoi *facedata;
};

/**
 * Interface to a parameters object.
 */
class ICameraParameters
{
public:
    virtual ~ICameraParameters() {}
    /**
     * Get serialized representation of the parameter object in a
     * ostream
     *
     * @param ps : reference to an ostream object
     *
     * @return int : 0 on success
     */
    virtual int writeObject(std::ostream& ps) const = 0;
};

/**
 * Interface for camera listener object. Client needs to
 * implement this interface to get access to camera data and
 * control events. The methods in listener can be invoked from
 * multiple different threads. Client needs to make sure that
 * implementation is thread-safe.
 */
class ICameraListener
{
public:

    virtual ~ICameraListener() {}

    /**
     * This function is called when an error is generated by camera
     * driver
     */
    virtual void onError() {}

    /**
     * This function is called when a control event needs to be
     * delivered to a client.
     *
     * @param control [in]: control event object
     */
    virtual void onControl(const ControlEvent& control) {}

    /**
     * This function is called when a preview frame is generated by
     * camera.
     *
     * @param frame [in]: pointer to an existing ICameraFrame
     *              generated by camera
     */
    virtual void onPreviewFrame(ICameraFrame *frame) {}

    /**
     * This function is called when a preview frame is generated by
     * camera.
     *
     * @param frame [in]: pointer to an existing ICameraFrame
     *              generated by camera
     */
    virtual void onVideoFrame(ICameraFrame *frame) {}

    /**
     * This function is called when a snapshot frame is generated by
     * camera.
     *
     * @param frame [in]: pointer to an existing ICameraFrame
     *              generated by camera
     */
    virtual void onPictureFrame(ICameraFrame *frame) {}

    /**
     * This function is called when a metadata frame is generated by
     * camera.
     *
     * @param frame [in]: pointer to an existing ICameraFrame
     *              generated by camera
     */
    virtual void onMetadataFrame(ICameraFrame *frame) {}
};

/**
 * Interface to a camera device. Client uses the API provided by
 * this interface to interact with camera device.
 */
class ICameraDevice
{
    DISALLOW_COPY_AND_ASSIGN(ICameraDevice);
protected:
    ICameraDevice() {}
    virtual ~ICameraDevice() {}

public:

    /**
     * Factory method to create an instance of ICameraDevice
     *
     * @param index [in]: camera ID
     * @param device [out]: pointer to to a ICameraDevice* to be
     *               created
     *
     * @return int : 0 on Success
     */
    static int createInstance(int index, ICameraDevice** device);

    /**
     * delete instance of an ICameraDevice
     *
     * @param device [out]: pointer to an ICameraDevice* to be
     *               deleted
     */
    static void deleteInstance(ICameraDevice** device);

    /**
     * Add listener to handle various notifications and data frames from the
     * camera device.
     * @param listener [in]
     **/
    virtual void addListener(ICameraListener *listener) = 0;

    /**
     * Removes a previously added listener from camera device
     *
     * @param listener [in]
     */
    virtual void removeListener(ICameraListener *listener) = 0;

    /**
     * Subscribe for camera control events
     *
     * @param eventMask : is a bitmask of values in enum EventType
     * @return int : 0 on success
     **/
    virtual void subscribe(uint32_t eventMask) = 0;

    /**
     * Unsubscribe to previously subscribed control events.
     *
     * @param eventMask : is a bitmask of values in enum EventType
     * @return int : 0 on success
     **/
    virtual void unsubscribe(uint32_t eventMask) = 0;

    /**
     * Set parameters in camera device. Camera device state will be
     * updated with new settings when this call returns.
     *
     * @param params [in]: populated parameters object with new
     *               camera parameters to be set
     *
     * @return int : 0 on success
     */
    virtual int setParameters(const ICameraParameters& params) = 0;

    /**
     * Retrieve the octet stream of parameters as name,value pairs. Note the
     * parameters fetched in to be buffer may be partial to the extent of bufSize.
     * Use the bufSizeRequired to determine the total length of buffer needed to
     * get all the parameters.
     *
     * @param buf [out]: buffer will be populated with the octet stream of
     *            parameters.
     * @param bufSize [in]: sizeof memory at buf.
     * @param bufSizeRequired [out]: optional if provided will be populated
     * with total size of buffer needed to fetch all the parameters.
     *
     * @return int : 0 on success
     **/
    virtual int getParameters(uint8_t* buf, uint32_t bufSize,
                              int* bufSizeRequired = NULL) = 0;

     /**
     * Gets the pointer to the the class object that define the
     * parameters of the camera sensor. (HAL3 only)
     *
     * @return pointer to parameters object typecasted to void
     **/
    virtual void* getParameters() = 0;


    /**
     * Start preview stream on camera
     *
     * @return int : 0 on success
     */
    virtual int startPreview() = 0;

    /**
     * Stop preview streaming
     */
    virtual void stopPreview() = 0;

    /**
     * Start video recording stream
     *
     * @return int : 0 on success
     */
    virtual int startRecording() = 0;

    /**
     * Stop video recording
     */
    virtual void stopRecording() = 0;

    /**
     * @brief Take a picture
     *
     * Preview needs to be running when this method is called. Video
     * recording may or may not be running. If successful,
     * onPictureFrame() callback is called with image data for
     * picture. Client needs to wait for this callback before taking
     * any more pictures.
     *
     * @return int
     */
    virtual int takePicture(uint32_t num_images = 1) = 0;


    virtual void sendFaceDetectCommand(bool turn_on) {};

};

/**
 * Structure to hold information about a single camera
 * device
 */
struct CameraInfo {
    int func;
};

/**
 * Get the number of camera (sensors) detected on the device.
 *
 * @return int
 **/
int getNumberOfCameras();

/**
 * Get the additional information about camera by index. index 0 identifies
 * the first camera.
 *
 * @param idx : index of the camera device. index 0 identifies first
 *            camera.
 * @param info
 * @return int : 0 is success
 **/
int getCameraInfo(int idx, struct CameraInfo& info);

} /* namespace camera */

#endif  /* !__CAMERA_H__ */
