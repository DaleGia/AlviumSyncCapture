#ifndef AlviumSyncDetect_H_
#define AlviumSyncDetect_H_

/**
 * @file AlviumSyncDetect.hpp
 * Copyright (c) 2024 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include <iostream>

#include "GUIDetect.hpp"
#include "AlliedVisionAlvium/AlliedVisionAlviumPPSSync.hpp"
#include "OpenCVFITS/OpenCVFITS.hpp"
#include <opencv2/opencv.hpp>

#include "ImageTransientDetection/ImageTransientDetection.hpp"
#include "ImageTransientDetection/ImagePreviewWindow.hpp"
#include "ImageTransientDetection/StackedImage.hpp"
#include "ImageTransientDetection/BrightObjectMasking.hpp"

#include "AlliedVisionAlvium/PPSSync.hpp"
#include "RingBuffer/RingBuffer.hpp"

/*****************************************************************************/
/*MACROS                                                             */
/*****************************************************************************/

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
class AlviumSyncDetect
{
public:
    AlviumSyncDetect();
    AlviumSyncDetect(std::string cameraName) : cameraName(cameraName) {};
    ~AlviumSyncDetect()
    {
        this->camera.disconnect();
    };
    /**
     * @brief
     *
     */
    void run(std::string name, bool polarimetricFlag);

private:
    GUIDetect screen;
    AlliedVisionAlviumPPSSync camera;
    std::string cameraName;

    ImageTransientDetection imageTransientDetection;

    std::atomic<bool>
        exitFlag = false;

    bool polariserMode = false;

    std::atomic<bool> isImageReceptionEnabled = false;
    std::atomic<bool> isSavingEnabled = false;

    uint64_t receivedFramesCount = 0;
    uint64_t savedFramesCount = 0;
    std::string currentRootSavePath;
    std::string currentSavePath;

    static const uint32_t IMAGEBUFFERSIZE = 100;
    /* 12 seconds is 10 second exposure max plus 2*/
    static const uint32_t ACQUIRESINGLEIMAGETIMEOUT_MS = 12000U;
    GNSS gnss;

    uint32_t prePostDetectionBufferSize = 20;
    uint32_t detectionDownsampleFactor = 1;
    std::mutex detectionImagesLock;
    bool detectionActiveFlag = false;
    cv::Rect detectionBoundingBox;
    cv::Rect convertedDetectionBoundingBox;

    uint64_t detectionTailFrameCount = 0;
    time_t detectionStartTime;
    time_t detectionEndTime;
    StackedImage detectionInputStack;
    cv::Mat previousInputStack;
    cv::Mat detectionStack;
    uint64_t detectionCount = 0;

    bool maskBrightObjects = false;
    BrightObjectMasking brightObjectMasking;
    StackedImage brightObjectInputStack;
    cv::Mat brightObjectMask;

    std::vector<AlliedVisionAlviumPPSSynchronisedFrameData> detectionImages;
    std::unique_ptr<RingBuffer<AlliedVisionAlviumPPSSynchronisedFrameData>>
        predetectionImages;

    void
    updateCameraCapture(void);

    bool startDetection();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool startImageAcquisition(void);

    /**
     * @brief
     *
     */
    bool stopImageAcquisition(void);

    /**
     * @brief
     *
     */
    bool startImageSaving(void);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool stopImageSaving(void);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool enableExternalTriggering(void);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool disableExternalTriggering(void);

    void getCameraTemperature(double sensor, double mainboard);

    static void writeDetectionToFile(AlviumSyncDetect *app);

    /**
     * @brief Create a New Save Directory object
     *
     * @param directoryName
     * @return true
     * @return false
     */
    bool createNewSaveDirectory(std::string directoryName);

    /**
     * @brief
     *
     * @param now
     * @return uint64_t
     */
    static uint64_t systemClockToUTC(std::chrono::system_clock::time_point now);

    /**
     * @brief
     *
     * @param frameData
     * @param arg
     */
    static void frameReceviedFunction(
        AlliedVisionAlviumPPSSynchronisedFrameData &frameData,
        void *arg);

    /**
     * @brief
     *
     * @param eventName
     * @param value
     * @param arg
     */
    static void cameraPPSCallback(
        int64_t cameraPPSTimestamp,
        int64_t systemPPSTimestamp,
        int64_t cameraPPSJitter,
        int64_t systemPPSJitter,
        void *arg);

    static void previewFunction();

    static void polPreviewFunction();

    void newInputStackCallback(
        cv::Mat &stack,
        double brightnessFactor);

    void newBrightObjectStackCallback(
        cv::Mat &stack,
        double brightnessFactor);
};

#endif // AlviumSyncDetect_H_
