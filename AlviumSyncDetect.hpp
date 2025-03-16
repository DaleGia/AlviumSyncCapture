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
#include "ImageTransientDetection/ImagePreviewWindow.hpp"
#include "OpenCVFITS/OpenCVFITS.hpp"
#include <opencv2/opencv.hpp>
#include "AlliedVisionAlvium/PPSSync.hpp"

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

    std::atomic<bool> exitFlag = false;

    std::atomic<bool> isImageReceptionEnabled = false;
    std::atomic<bool> isSavingEnabled = false;
    std::atomic<bool> isGNSSTriggeringEnabled = false;

    uint64_t receivedFramesCount = 0;
    uint64_t savedFramesCount = 0;
    std::string currentRootSavePath;
    std::string currentSavePath;

    std::atomic<int64_t> triggerFrequency = 0;

    double lastSensorTemperature = 0;
    double lastMainboardTemperature = 0;
    std::mutex temperatureMutex;

    static const uint32_t IMAGEBUFFERSIZE = 100;
    /* 12 seconds is 10 second exposure max plus 2*/
    static const uint32_t ACQUIRESINGLEIMAGETIMEOUT_MS = 12000U;
    OpenCVFITS fits;

    GNSS gnss;

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
};

#endif // AlviumSyncDetect_H_
