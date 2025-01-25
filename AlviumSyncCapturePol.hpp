#ifndef ALVIUMSYNCCAPTUREPOL_H_
#define ALVIUMSYNCCAPTUREPOL_H_

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
#include "GNSS.hpp"
#include "AlliedVisionAlvium/AlliedVisionAlvium.hpp"
#include "ImagePreviewWindow.hpp"
#include "OpenCVFITS/OpenCVFITS.hpp"
#include <opencv2/opencv.hpp>

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

    /**
     * @brief
     *
     */
    void run(void);

private:
    GUIControl controlScreen;
    GNSS gnss;
    AlliedVisionAlvium camera;

    std::mutex lastRecievedImageMutex;
    cv::Mat lastRecievedImage;
    std::atomic<bool> exitDisplayThreadFlag = false;

    std::atomic<bool> isImageReceptionEnabled = false;
    std::atomic<bool> isSavingEnable = false;
    std::atomic<bool> isGNSSTriggeringEnabled = false;
    std::atomic<uint64_t>
        receivedFramesCount = 0;
    std::atomic<uint64_t> savedFramesCount = 0;
    std::string currentRootSavePath;
    std::string currentSavePath;

    std::atomic<double> currentExposureUs = 0;
    std::atomic<double> currentGainDb = 0;
    std::atomic<double> calculatedCameraFPS = 0;
    std::atomic<int64_t> triggerFrequency = 0;
    std::atomic<int64_t> lastSystemTimeAtCameraPPS = 0;
    std::atomic<int64_t> lastSystemTimeJitter = 0;
    std::atomic<int64_t> lastCameraPPSTimestamp = 0;
    std::atomic<int64_t> lastCameraTimeJitter = 0;
    std::atomic<int64_t> lastGNSSTime = 0;

    std::atomic<double> lastLatitude = 0;
    std::atomic<double> lastLongitude = 0;
    std::atomic<double> lastAltitudeMSL = 0;

    static const uint32_t IMAGEBUFFERSIZE = 10;
    /* 12 seconds is 10 second exposure max plus 2*/
    static const uint32_t ACQUIRESINGLEIMAGETIMEOUT_MS = 12000U;
    OpenCVFITS fits;

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
        std::string eventName,
        int64_t value,
        time_t systemTimestampSeconds,
        time_t systemTimestampNanoseconds,
        void *arg);
};

#endif // ALVIUMSYNCCAPTURE_H_
