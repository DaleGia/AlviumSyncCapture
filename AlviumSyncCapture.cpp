/**
 * @file AlviumSyncCapture.c
 * Copyright (c) 2023 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include "unistd.h"
#include <iostream>

#include "GUIScreen.hpp"
#include "GNSS.hpp"
#include "AlliedVisionAlvium/AlliedVisionAlvium.hpp"
#include "OpenCVFITS/OpenCVFITS.hpp"

static const uint32_t IMAGEBUFFERSIZE = 10;

std::atomic<bool> isImageReceptionEnabled = false;
std::atomic<bool> isRecordingEnabled = false;
std::atomic<uint64_t> receivedFramesCount = 0;
std::atomic<uint64_t> savedFramesCount = 0;
std::string currentSavePath;

std::atomic<double> currentExposureUs = 0;
std::atomic<double> currentGainDb = 0;

std::atomic<uint64_t> lastGNSSTimestamp = 0;
std::atomic<uint64_t> lastCameraPPSTimestamp = 0;

OpenCVFITS fits;

void updateCameraConfig(GUIScreen &screen, AlliedVisionAlvium &camera);
void frameReceviedFunction(
    cv::Mat frame,
    uint64_t cameraTimestamp,
    uint64_t cameraFrameId,
    void *arg);
/*****************************************************************************/
/* MAIN                                                                      */
/*****************************************************************************/
int main(int argc, const char **argv)
{
    GUIScreen screen;
    GNSS gnss;
    AlliedVisionAlvium camera;

    /* Try to connect to a camera. Fail if you cannot */
    if (false == camera.connect())
    {
        return -1;
    }

    /* Start the GNSS stuff */
    gnss.start();

    std::string cameraName = camera.getName();
    std::cout << cameraName << std::endl;
    screen.capture->connectedCamera->setValue(cameraName);

    /* Create the screen and update the connected camera */
    screen.start();

    // /* Set what happens when the capture button is pushed */
    // screen.capture->acquireButton->setCallback(
    //     [&screen, &camera]
    //     {
    //         if (false == isImageReceptionEnabled)
    //         {
    //             /* We are not currently capturing... Lets start capturing*/
    //             /* update the currently configured camera values */
    //             updateCameraConfig(screen, camera);

    //             /* start the camera acquisition */
    //             if (true == camera.startAcquisition(IMAGEBUFFERSIZE, frameReceviedFunction, NULL))
    //             {
    //                 isImageReceptionEnabled = true;
    //                 screen.capture->acquireButton->setCaption("Stop image acquisition");
    //                 screen.capture->acquireButton->setBackgroundColor(
    //                     screen.capture->RED);
    //                 std::cout << "Started image acquisition... " << std::endl;
    //             }
    //         }
    //         else
    //         {
    //             /* We are currently capturing... Lets stop capturing*/
    //             /* Stop the camera acquisition */
    //             if (true == camera.stopAcquisition())
    //             {
    //                 isImageReceptionEnabled = false;
    //                 screen.capture->acquireButton->setCaption("Start image acquisition");
    //                 screen.capture->acquireButton->setBackgroundColor(
    //                     screen.capture->GREEN);
    //                 std::cout << "Stopped image acquisition... " << std::endl;
    //             }
    //             /* update the currently configured camera values */
    //             updateCameraConfig(screen, camera);
    //         }
    //     });
    /* If the screen is exited, it ends up here */
    return 1;
}

void frameReceviedFunction(
    cv::Mat frame,
    uint64_t cameraTimestamp,
    uint64_t cameraFrameId,
    void *arg)
{
    // Get current time with milliseconds
    auto now = std::chrono::system_clock::now();

    receivedFramesCount++;

    if (isRecordingEnabled)
    {
        // Convert to time_since_epoch in microseconds
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
        // Extract the number of microseconds
        auto timestamp = microseconds.count();

        if (false == fits.addMat2FITS(frame))
        {
            std::cerr << "Could not add image to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("ExposureTime", currentExposureUs))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("Gain", currentGainDb))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("FrameID", cameraFrameId))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("cameraImageTimestamp", cameraTimestamp))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("systemImageReceptionTimestampUTC", cameraTimestamp))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("cameraLastPPSTimestamp", lastCameraPPSTimestamp))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else if (false == fits.addKey2FITS("GNSSLastPPSTimestampUTC", lastGNSSTimestamp))
        {
            std::cerr << "Could not add exposure to fits file " << std::endl;
        }
        else
        {
            savedFramesCount++;
        }
    }
}

void updateCameraConfig(GUIScreen &screen, AlliedVisionAlvium &camera)
{
    std::string value;
    if (false == camera.getFeature("ExposureTime", value))
    {
        std::cerr << "Could not get camera exposure" << std::endl;
    }
    else
    {
        screen.cameraWindow->exposure->setValue(std::stod(value));
    }

    if (false == camera.getFeature("Gain", value))
    {
        std::cerr << "Could not get camera gain" << std::endl;
    }
    else
    {
        screen.cameraWindow->gain->setValue(std::stod(value));
    }
}
