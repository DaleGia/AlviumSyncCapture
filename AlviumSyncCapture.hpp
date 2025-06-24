#ifndef ALVIUMSYNCCAPTURE_H_
#define ALVIUMSYNCCAPTURE_H_

/**
 * @file AlviumSyncCapture.hpp
 * Copyright (c) 2024 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include <iostream>

#include "GUIControl.hpp"
#include "AlliedVisionAlvium/GNSS.hpp"
#include "AlliedVisionAlvium/AlliedVisionAlvium.hpp"
#include "ImageTransientDetection/ImagePreviewWindow.hpp"
#include "ImageTransientDetection/StackedImage.hpp"
#include "OpenCVFITS/OpenCVFITS.hpp"
#include <opencv2/opencv.hpp>
#include "AlliedVisionAlvium/PPSSync.hpp"

/*****************************************************************************/
/*MACROS                                                             */
/*****************************************************************************/

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
class AlviumSyncCapture
{
public:
    AlviumSyncCapture();
    AlviumSyncCapture(std::string cameraName) : cameraName(cameraName) {};
    ~AlviumSyncCapture()
    {
        this->camera.disconnect();
    };
    /**
     * @brief
     *
     */
    void run(std::string name, bool polarimetricFlag);

private:
    GUIControl controlScreen;
    GNSS gnss;
    AlliedVisionAlvium camera;
    std::string cameraName;

    std::mutex lastRecievedImageMutex;
    cv::Mat lastRecievedImage;
    std::atomic<bool> exitFlag = false;

    std::atomic<bool> isImageReceptionEnabled = false;
    std::atomic<bool> isSavingEnabled = false;
    std::atomic<bool> isGNSSTriggeringEnabled = false;

    bool stackEnabled = false;
    StackedImage stack;
    cv::Mat oldStack;
    double oldBrightnessFactor;
    uint64_t receivedFramesCount = 0;
    uint64_t savedFramesCount = 0;
    std::string currentRootSavePath;
    std::string currentSavePath;

    std::atomic<int64_t> triggerFrequency = 0;

    int64_t lastGNSSTime = 0;
    double lastLatitude = 0;
    double lastLongitude = 0;
    double lastAltitudeMSL = 0;
    std::mutex gnssMutex;

    double lastSensorTemperature = 0;
    double lastMainboardTemperature = 0;
    std::mutex temperatureMutex;

    static const uint32_t IMAGEBUFFERSIZE = 100;
    /* 12 seconds is 10 second exposure max plus 2*/
    static const uint32_t ACQUIRESINGLEIMAGETIMEOUT_MS = 12000U;
    OpenCVFITS fits;

    PPSSync pps;

    void updateCameraCapture(void);

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
     * @brief Create a New F I T S object
     *
     * @return true
     * @return false
     */
    bool createNewFITS(void);

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

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool enablePPSEvent(void);

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
     * @param frequency
     * @return true
     * @return false
     */
    bool setGNSSTriggerFrequency(int32_t frequency);

    /**
     * @brief
     *
     * @param frequency
     * @return true
     * @return false
     */
    bool getGNSSTriggerFrequency(int32_t &frequency);

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
        AlliedVisionAlviumFrameData &frameData,
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
};

#endif // ALVIUMSYNCCAPTURE_H_
