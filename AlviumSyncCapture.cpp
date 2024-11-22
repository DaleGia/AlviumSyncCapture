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
#include <opencv2/opencv.hpp>

/*****************************************************************************/
/* Class                                                                      */
/*****************************************************************************/
class AlviumSyncCapture
{
public:
    AlviumSyncCapture() {};

    void run(void)
    {
        /* Try to connect to a camera. Fail if you cannot */
        if (false == camera.connect())
        {
            return;
        }

        std::string cameraName = camera.getName();
        std::cout << cameraName << std::endl;
        this->screen.capture->connectedCamera->setValue(cameraName);

        /* Set what happens when the acquise image button is pushed */
        this->screen.capture->acquireButton->setCallback(
            [this]
            {
                if (false == isImageReceptionEnabled)
                {
                    /* We are not currently capturing... Lets start capturing*/
                    /* update the currently captureured camera values */
                    this->updateCameracapture();

                    /* start the camera acquisition */
                    if (true == this->camera.startAcquisition(IMAGEBUFFERSIZE, AlviumSyncCapture::frameReceviedFunction, this))
                    {
                        this->isImageReceptionEnabled = true;
                        this->screen.capture->acquireButton->setCaption("Stop image acquisition");
                        this->screen.capture->acquireButton->setBackgroundColor(
                            this->screen.capture->RED);
                        std::cout << "Started image acquisition... " << std::endl;
                    }
                    else
                    {
                        std::cerr << "Unable to start image acquisition... " << std::endl;
                    }
                }
                else
                {
                    /* We are currently capturing... Lets stop capturing*/
                    /* Stop the camera acquisition */
                    if (true == camera.stopAcquisition())
                    {
                        this->isImageReceptionEnabled = false;
                        this->screen.capture->acquireButton->setCaption("Start image acquisition");
                        this->screen.capture->acquireButton->setBackgroundColor(
                            this->screen.capture->GREEN);
                        std::cout << "Stopped image acquisition... " << std::endl;
                    }

                    /* update the currently captureured camera values */
                    this->updateCameracapture();
                }
            });

        /* Set what happens when the acquise image button is pushed */
        this->screen.capture->acquireSingleButton->setCallback(
            [this]
            {
                if (false == this->isImageReceptionEnabled)
                {
                    cv::Mat image;
                    uint64_t cameraTimestamp;
                    uint64_t cameraFrameId;
                    /* We are not currently capturing... Lets start capturing*/
                    /* update the currently captureured camera values */
                    this->updateCameracapture();

                    this->screen.capture->acquireSingleButton->setCaption("Getting Image...");
                    this->screen.capture->acquireSingleButton->setBackgroundColor(
                        this->screen.capture->RED);
                    /* start the camera acquisition */
                    if (true == this->camera.getSingleFrame(image, cameraFrameId, cameraTimestamp, ACQUIRESINGLEIMAGETIMEOUT_MS))
                    {
                        // Get current time with milliseconds
                        auto now = std::chrono::system_clock::now();
                        // Convert to time_since_epoch in microseconds
                        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
                        // Extract the number of microseconds
                        uint64_t timestamp = systemClockToUTC(now);

                        this->receivedFramesCount++;

                        if (this->isSavingEnable)
                        {
                            std::string directory;
                            directory = screen.capture->imageSaveLocation->value();
                            this->currentSavePath = directory;

                            auto milliseconds =
                                std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                            auto now_c = std::chrono::system_clock::to_time_t(now);

                            // Format date and time string (YYYYMMDD_HHMMSS_mmm)
                            std::stringstream filename_stream;
                            filename_stream << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S_")
                                            << std::setfill('0') << std::setw(3) << milliseconds.count();
                            if (false == this->createNewSaveDirectory(filename_stream.str()))
                            {
                                return;
                            }

                            OpenCVFITS singleFits;
                            if (false == singleFits.createFITS(this->currentSavePath + "/" + filename_stream.str() + ".fit"))
                            {
                                std::cerr << "Could not create fits file" << std::endl;
                            }
                            else if (false == singleFits.addMat2FITS(image))
                            {
                                std::cerr << "Could not add image to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("ExposureTime", this->currentExposureUs))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("Gain", this->currentGainDb))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("FrameID", cameraFrameId))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("cameraImageTimestamp", cameraTimestamp))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("systemImageReceptionTimestampUTC", timestamp))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("cameraLastPPSTimestamp", this->lastCameraPPSTimestamp))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (false == singleFits.addKey2FITS("GNSSLastPPSTimestampUTC", this->lastGNSSTimestamp))
                            {
                                std::cerr << "Could not add exposure to fits file " << std::endl;
                            }
                            else if (true == singleFits.closeFITS())
                            {
                                this->savedFramesCount++;
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "Unable to get single image... " << std::endl;
                    }

                    this->screen.capture->acquireSingleButton->setCaption("Acquire single image");
                    this->screen.capture->acquireSingleButton->setBackgroundColor(
                        this->screen.capture->GREEN);
                }
            });

        /* Set what happens when the recording button is pushed */
        screen.capture->recordingButton->setCallback(
            [this]
            {
                if (false == this->isSavingEnable)
                {
                    if (screen.capture->imageSaveLocation->value() == "")
                    {
                        std::cerr << "No save location selected... please select one in the captureuration menu..." << std::endl;
                    }
                    else
                    {
                        /* Get the current save location */
                        std::string directory;
                        directory = screen.capture->imageSaveLocation->value();
                        this->currentSavePath = directory;

                        // Get current time with milliseconds
                        auto now = std::chrono::system_clock::now();
                        auto milliseconds =
                            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                        auto now_c = std::chrono::system_clock::to_time_t(now);

                        // Format date and time string (YYYYMMDD_HHMMSS_mmm)
                        std::stringstream filename_stream;
                        filename_stream << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S_")
                                        << std::setfill('0') << std::setw(3) << milliseconds.count();
                        if (false == this->createNewSaveDirectory(filename_stream.str()))
                        {
                            return;
                        }

                        if (false == this->fits.createFITS(this->currentSavePath + "/" + filename_stream.str() + ".fit"))
                        {
                            std::cerr << "Could not create fits file" << std::endl;
                            return;
                        }

                        this->isSavingEnable = true;
                        screen.capture->recordingButton->setCaption("Stop Recording");
                        screen.capture->recordingButton->setBackgroundColor(
                            screen.capture->RED);
                        std::cout << "Started to record... " << std::endl;
                    }
                }
                else
                {
                    std::string string;
                    this->isSavingEnable = false;

                    this->fits.closeFITS();

                    screen.capture->recordingButton->setCaption("Start Recording");
                    screen.capture->recordingButton->setBackgroundColor(
                        screen.capture->GREEN);
                    std::cout << "Stopped recording... " << std::endl;
                }
            });

        /* Set what happens when the pixel format is edited */
        screen.cameraWindow->pixelFormat->setCallback(
            [this](std::string value)
            {
                if (false == this->camera.setFeature("PixelFormat", value))
                {
                    std::cerr << "Could not set bitDepth to " << value << std::endl;
                }
                else
                {
                    std::cout << "Set PixelFormat to " << value << std::endl;
                }

                this->updateCameracapture();
                return true;
            });
        /* Set what happens when Gain is edited */
        screen.cameraWindow->gain->setCallback(
            [this](double value)
            {
                if (false == this->camera.setFeature("Gain", std::to_string(value)))
                {
                    std::cerr << "Could not set gain..." << std::endl;
                }
                else
                {
                    std::cout << "Set gain to " << value << std::endl;
                }

                this->updateCameracapture();
            });

        /* Set what happens when exposure is edited */
        screen.cameraWindow->exposure->setCallback(
            [this](double value)
            {
                if (false == camera.setFeature("ExposureTime", std::to_string(value)))
                {
                    std::cerr << "Could not set exposure..." << std::endl;
                }
                else
                {
                    std::cout << "Set exposure to " << value << std::endl;
                }

                this->updateCameracapture();
            });

        /* Set what happens when exposure is edited */
        screen.cameraWindow->pixelFormat->setCallback(
            [this](std::string value)
            {
                if (false == this->camera.setFeature("PixelFormat", value))
                {
                    std::cerr << "Could not set PixelFormat to " << value << " ... " << std::endl;
                }
                else
                {
                    std::cout << "Set PixelFormat to " << value << std::endl;
                }

                this->updateCameracapture();
                return true;
            });

        /* Create the screen and update the connected camera */
        this->updateCameracapture();
        this->screen.start();
    }

private:
    GUIScreen screen;
    GNSS gnss;
    AlliedVisionAlvium camera;

    std::atomic<bool> isImageReceptionEnabled = false;
    std::atomic<bool> isSavingEnable = false;
    std::atomic<uint64_t> receivedFramesCount = 0;
    std::atomic<uint64_t> savedFramesCount = 0;
    std::string currentSavePath;

    std::atomic<double> currentExposureUs = 0;
    std::atomic<double> currentGainDb = 0;

    std::atomic<uint64_t> lastGNSSTimestamp = 0;
    std::atomic<uint64_t> lastCameraPPSTimestamp = 0;

    static const uint32_t IMAGEBUFFERSIZE = 10;
    /* 12 seconds is 10 second exposure max plus 2*/
    static const uint32_t ACQUIRESINGLEIMAGETIMEOUT_MS = 12000U;
    OpenCVFITS fits;

    static void frameReceviedFunction(
        cv::Mat frame,
        uint64_t cameraTimestamp,
        uint64_t cameraFrameId,
        void *arg)
    {
        // Get current time with milliseconds
        auto now = std::chrono::system_clock::now();
        AlviumSyncCapture *self = (AlviumSyncCapture *)arg;
        self->receivedFramesCount++;

        if (self->isSavingEnable)
        {
            // Convert to time_since_epoch in microseconds
            auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
            // Extract the number of microseconds
            auto timestamp = microseconds.count();

            if (false == self->fits.addMat2FITS(frame))
            {
                std::cerr << "Could not add image to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("ExposureTime", self->currentExposureUs))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("Gain", self->currentGainDb))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("FrameID", cameraFrameId))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("cameraImageTimestamp", cameraTimestamp))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("systemImageReceptionTimestampUTC", cameraTimestamp))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("cameraLastPPSTimestamp", self->lastCameraPPSTimestamp))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else if (false == self->fits.addKey2FITS("GNSSLastPPSTimestampUTC", self->lastGNSSTimestamp))
            {
                std::cerr << "Could not add exposure to fits file " << std::endl;
            }
            else
            {
                self->savedFramesCount++;
            }
        }
    }

    void updateCameracapture(void)
    {
        std::string value;
        if (false == this->camera.getFeature("ExposureTime", value))
        {
            std::cerr << "Could not get camera exposure" << std::endl;
        }
        else
        {
            this->screen.cameraWindow->exposure->setValue(std::stod(value));
        }

        if (false == this->camera.getFeature("Gain", value))
        {
            std::cerr << "Could not get camera gain" << std::endl;
        }
        else
        {
            this->screen.cameraWindow->gain->setValue(std::stod(value));
        }

        if (false == this->camera.getFeature("PixelFormat", value))
        {
            std::cerr << "Could not get camera gain" << std::endl;
        }
        else
        {
            this->screen.cameraWindow->pixelFormat->setValue(value);
        }
    }

    bool createNewSaveDirectory(std::string directoryName)
    {
        /* We need to create a new directory for the file
         * save location. Then if we are saving average frames
         * we save the frame
         */

        std::string directorypath = this->currentSavePath + directoryName;

        /* Create the images and detections subfolders */
        if (true == std::filesystem::exists(directorypath))
        {
            // Directory already exists...
        }
        else if (false == std::filesystem::create_directories(directorypath))
        {
            std::cerr << "Could not create directory" << directorypath << std::endl;
            return false;
        }

        return true;
    }

    static uint64_t systemClockToUTC(std::chrono::system_clock::time_point now)
    {
        // Convert to time_point in UTC
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm *now_tm = std::gmtime(&now_time_t); // Convert to UTC time struct

        // Set the time zone to UTC
        now_tm->tm_isdst = 0; // Not Daylight Saving Time

        // Convert back to time_point
        std::chrono::system_clock::time_point utc_time_point = std::chrono::system_clock::from_time_t(std::mktime(now_tm));

        // Convert to microseconds since the Unix epoch
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(utc_time_point.time_since_epoch());
        uint64_t timestamp = microseconds.count();

        return timestamp;
    }
};

/*****************************************************************************/
/* MAIN                                                                      */
/*****************************************************************************/
int main(int argc, const char **argv)
{
    AlviumSyncCapture alviumSyncCapture;
    alviumSyncCapture.run();
    return 1;
}