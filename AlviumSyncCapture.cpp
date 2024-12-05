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
#include "AlviumSyncCapture.hpp"
#include "unistd.h"
#include <cstdlib>

/*****************************************************************************/
/* Class                                                                      */
/*****************************************************************************/
AlviumSyncCapture::AlviumSyncCapture() {
};

void AlviumSyncCapture::run(void)
{
    gnss.start(
        [this](
            double latitude,
            double longitude,
            double altitude,
            double timeSeconds,
            double timeNanoseconds,
            void *arg)
        {
            std::string location;
            location = std::to_string(latitude);
            location += " ";
            location += std::to_string(longitude);

            this->screen.gnssWindow->LocationBox->setValue(location);
            this->screen.gnssWindow->altitudeMSLBox->setValue(std::to_string(altitude));
            double timeTotal = timeSeconds + timeNanoseconds;

            this->lastGNSSTime = timeTotal;
            this->lastLatitude = latitude;
            this->lastLongitude = longitude;
            this->lastAltitudeMSL = altitude;

            // Convert to a time_t object
            time_t gnss_unix_timestamp = timeTotal;
            tm *tm_obj = gmtime(&gnss_unix_timestamp);
            // Format the time using stringstream
            std::stringstream ss;
            ss << std::put_time(tm_obj, "%Y-%m-%d %H:%M:%S");
            this->screen.gnssWindow->TimeUTCBox->setValue(ss.str());
        });

    int frequency;
    if (true == getGNSSTriggerFrequency(frequency))
    {
        this->triggerFrequency = frequency;
        this->screen.gnssWindow->triggerFrequencyBox->setValue(frequency);
    }
    else
    {
        this->screen.gnssWindow->triggerFrequencyBox->setValue(-1);
    }

    /* Try to connect to a camera. Fail if you cannot */
    if (false == camera.connect())
    {
        return;
    }

    if (false == camera.setFeature("DeviceLinkThroughputLimitMode", "Off"))
    {
        std::cerr << "Unable to disable DeviceLinkThroughputLimit... Maximum framerate may be affected..." << std::endl;
    }

    if (false == this->enablePPSEvent())
    {
        std::cerr << "Unable to enable PPS on camera... Something is wrong..." << std::endl;
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
                if (true == this->isSavingEnable)
                {
                    this->createNewFITS();
                }

                this->startImageAcquisition();
            }
            else
            {
                this->stopImageAcquisition();
            }
        });

    /* Set what happens when the acquise image button is pushed */
    this->screen.capture->acquireSingleButton->setCallback(
        [this]
        {
            std::cout << "Acuqiring single image" << std::endl;
            if (false == this->isImageReceptionEnabled)
            {
                cv::Mat image;
                uint64_t cameraTimestamp;
                uint64_t cameraFrameId;
                /* We are not currently capturing... Lets start capturing*/
                /* update the currently captureured camera values */
                this->updateCameraCapture();
                std::cout << "Camera Capture updated" << std::endl;

                this->screen.capture->acquireSingleButton->setCaption("Getting Image...");
                this->screen.capture->acquireSingleButton->setBackgroundColor(
                    this->screen.capture->RED);
                /* start the camera acquisition */
                if (true == this->camera.getSingleFrame(image, cameraFrameId, cameraTimestamp, ACQUIRESINGLEIMAGETIMEOUT_MS))
                {
                    std::cout << "Got Image" << std::endl;

                    // Get current time with milliseconds
                    auto now = std::chrono::system_clock::now();
                    // Extract the number of microseconds
                    uint64_t timestamp = systemClockToUTC(now);

                    this->receivedFramesCount++;
                    this->lastRecievedImageMutex.lock();
                    this->lastRecievedImage = image.clone();
                    this->lastRecievedImageMutex.unlock();
                    if (this->isSavingEnable)
                    {
                        std::string directory;
                        directory = screen.capture->imageSaveLocation->value();
                        this->currentRootSavePath = directory;

                        auto milliseconds =
                            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                        auto now_c = std::chrono::system_clock::to_time_t(now);

                        // Format date and time string (YYYYMMDD_HHMMSS_mmm)
                        std::stringstream filename_stream;
                        filename_stream << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S_")
                                        << std::setfill('0') << std::setw(3) << milliseconds.count();

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
                            std::cerr << "Could not add ExposureTime to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("Gain", this->currentGainDb))
                        {
                            std::cerr << "Could not add Gain to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("FrameID", cameraFrameId))
                        {
                            std::cerr << "Could not add FrameID to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("cameraFrameStartTimestamp", cameraTimestamp))
                        {
                            std::cerr << "Could not add cameraImageTimestamp to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("systemImageReceptionTimestampUTC", timestamp))
                        {
                            std::cerr << "Could not add systemImageReceptionTimestampUTC to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastSystemTimeAtCameraPPSTimestamp", this->lastSystemTimeAtCameraPPS))
                        {
                            std::cerr << "Could not add lastSystemTimeAtCameraPPSTimestamp to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastSystemTimeJitterAtCameraPPSuS", this->lastSystemTimeJitter))
                        {
                            std::cerr << "Could not add lastSystemTimeJitterAtCameraPPSuS to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastCameraPPSTimestamp", this->lastCameraPPSTimestamp))
                        {
                            std::cerr << "Could not add lastCameraPPSTimestamp to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastCameraTimeJitterAtCameraPPSuS", this->lastCameraTimeJitter))
                        {
                            std::cerr << "Could not add lastCameraTimeJitterAtCameraPPSuS to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastGNSSPPSTimestamp", this->lastGNSSTime))
                        {
                            std::cerr << "Could not add lastGNSSPPSTimestamp to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastLatitude", this->lastLatitude))
                        {
                            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastLongitude", this->lastLongitude))
                        {
                            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("lastAltitudeMSL", this->lastAltitudeMSL))
                        {
                            std::cerr << "Could not add lastAltitudeMSL to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("externalTriggerFrequency", this->triggerFrequency))
                        {
                            std::cerr << "Could not add externalTriggerFrequency to fits file " << std::endl;
                        }
                        else if (false == singleFits.addKey2FITS("externalTriggerEnabled", (int64_t)this->isGNSSTriggeringEnabled))
                        {
                            std::cerr << "Could not add externalTriggerEnabled to fits file " << std::endl;
                        }
                        else if (true == singleFits.closeFITS())
                        {
                            this->savedFramesCount++;
                        }
                    }
                    else
                    {
                        std::cout << "Saving not enabled." << std::endl;
                        this->lastRecievedImageMutex.lock();
                        this->lastRecievedImage = image.clone();
                        this->lastRecievedImageMutex.unlock();
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
            else
            {
                std::cerr << "Image reception is already enabled... " << std::endl;
            }
        });

    /* Set what happens when the recording button is pushed */
    screen.capture->recordingButton->setCallback(
        [this]
        {
            if (false == this->isSavingEnable)
            {

                if (true == isImageReceptionEnabled)
                {
                    this->createNewFITS();
                }

                this->startImageSaving();
            }
            else
            {
                this->stopImageSaving();
            }
        });

    /* Set what happens when the recording button is pushed */
    screen.cameraWindow->externalButton->setCallback(
        [this]
        {
            if (false == this->isGNSSTriggeringEnabled)
            {
                this->enableExternalTriggering();
            }
            else
            {
                this->disableExternalTriggering();
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

            this->updateCameraCapture();
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

            this->updateCameraCapture();
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

            this->updateCameraCapture();
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

            this->updateCameraCapture();
            return true;
        });

    screen.gnssWindow->triggerFrequencyBox->setCallback(
        [this](uint32_t value)
        {
            int retFrequency;
            this->setGNSSTriggerFrequency(value);

            if (true == this->getGNSSTriggerFrequency(retFrequency))
            {
                this->triggerFrequency = retFrequency;
                this->screen.gnssWindow->triggerFrequencyBox->setValue(retFrequency);
            }

            return true;
        });

    std::thread imagePreviewThread(
        [this]
        {
            ImagePreviewWindow imagePreview("Image Preview");
            imagePreview.setSize(800, 600);

            while (!exitDisplayThreadFlag)
            {
                cv::Mat frame;
                this->lastRecievedImageMutex.lock();
                frame = this->lastRecievedImage.clone();
                this->lastRecievedImageMutex.unlock();
                if (false == frame.empty())
                {
                    imagePreview.setImageStreched(frame, this->screen.cameraWindow->previewStretchSlider->value());
                    this->screen.cameraWindow->framesReceivedValue->setValue(receivedFramesCount);
                    this->screen.cameraWindow->framesSavedValue->setValue(savedFramesCount);
                }
                usleep(33000);
            }
        });
    /* Create the screen and update the connected camera */
    this->disableExternalTriggering();
    this->updateCameraCapture();
    this->screen.start();
}

void AlviumSyncCapture::frameReceviedFunction(
    cv::Mat frame,
    time_t systemTimestampSeconds,
    time_t systemTimestampNanoseconds,
    uint64_t cameraTimestamp,
    uint64_t cameraFrameId,
    void *arg)
{
    // Get current time with milliseconds
    AlviumSyncCapture *self = (AlviumSyncCapture *)arg;
    self->receivedFramesCount++;

    self->lastRecievedImageMutex.lock();
    self->lastRecievedImage = frame.clone();
    self->lastRecievedImageMutex.unlock();

    if (self->isSavingEnable)
    {
        std::tm *tmutc = std::gmtime(&systemTimestampSeconds);
        time_t systemTimestamp = std::mktime(tmutc) + systemTimestampNanoseconds;

        if (false == self->fits.addMat2FITS(frame))
        {
            std::cerr << "Could not add image to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("ExposureTime", self->currentExposureUs))
        {
            std::cerr << "Could not add ExposureTime to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("Gain", self->currentGainDb))
        {
            std::cerr << "Could not add Gain to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("FrameID", cameraFrameId))
        {
            std::cerr << "Could not add FrameID to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("cameraFrameStartTimestamp", cameraTimestamp))
        {
            std::cerr << "Could not add cameraImageTimestamp to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("systemImageReceptionTimestampUTC", systemTimestamp))
        {
            std::cerr << "Could not add systemImageReceptionTimestampUTC to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastSystemTimeAtCameraPPSTimestamp", self->lastSystemTimeAtCameraPPS))
        {
            std::cerr << "Could not add lastSystemTimeAtCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastSystemTimeJitterAtCameraPPSuS", self->lastSystemTimeJitter))
        {
            std::cerr << "Could not add lastSystemTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastCameraPPSTimestamp", self->lastCameraPPSTimestamp))
        {
            std::cerr << "Could not add lastCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastCameraTimeJitterAtCameraPPSuS", self->lastCameraTimeJitter))
        {
            std::cerr << "Could not add lastCameraTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastGNSSPPSTimestamp", self->lastGNSSTime))
        {
            std::cerr << "Could not add lastGNSSPPSTimestamp to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastLatitude", self->lastLatitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastLongitude", self->lastLongitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("lastAltitudeMSL", self->lastAltitudeMSL))
        {
            std::cerr << "Could not add lastAltitudeMSL to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("externalTriggerFrequency", self->triggerFrequency))
        {
            std::cerr << "Could not add externalTriggerFrequency to fits file " << std::endl;
        }
        else if (false == self->fits.addKey2FITS("externalTriggerEnabled", (int64_t)self->isGNSSTriggeringEnabled))
        {
            std::cerr << "Could not add externalTriggerEnabled to fits file " << std::endl;
        }
        else
        {
            self->savedFramesCount++;
        }
    }
}

void AlviumSyncCapture::updateCameraCapture(void)
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

bool AlviumSyncCapture::startImageAcquisition(void)
{
    /* We are not currently capturing... Lets start capturing*/
    /* update the currently captureured camera values */
    this->updateCameraCapture();

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
        return false;
    }

    return true;
}
bool AlviumSyncCapture::stopImageAcquisition(void)
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
    else
    {
        return false;
    }
    /* update the currently captureured camera values */
    this->updateCameraCapture();
    return true;
}

bool AlviumSyncCapture::createNewFITS(void)
{
    // Get current time with milliseconds
    auto now = std::chrono::system_clock::now();
    auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto now_c = std::chrono::system_clock::to_time_t(now);

    // Format date and time string (YYYYMMDD_HHMMSS_mmm)
    std::stringstream filename_stream;
    filename_stream << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S_")
                    << std::setfill('0') << std::setw(3) << milliseconds.count();

    if (false == this->fits.createFITS(this->currentSavePath + "/" + filename_stream.str() + ".fit"))
    {
        std::cerr << "Could not create fits file" << std::endl;
        return false;
    }

    return true;
}

bool AlviumSyncCapture::startImageSaving(void)
{

    if (this->screen.capture->imageSaveLocation->value() == "")
    {
        std::cerr
            << "No save location selected... please select one in the capture window..." << std::endl;
        return false;
    }
    else
    {
        /* Get the current save location */
        std::string directory;
        directory = this->screen.capture->imageSaveLocation->value();
        this->currentRootSavePath = directory;

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
            return false;
        }

        this->isSavingEnable = true;
        this->screen.capture->recordingButton->setCaption("Stop Recording");
        this->screen.capture->recordingButton->setBackgroundColor(
            screen.capture->RED);
        std::cout << "Started to record... " << std::endl;

        return true;
    }
}

bool AlviumSyncCapture::stopImageSaving(void)
{

    this->isSavingEnable = false;

    this->fits.closeFITS();

    screen.capture->recordingButton->setCaption("Start Recording");
    screen.capture->recordingButton->setBackgroundColor(
        screen.capture->GREEN);
    std::cout << "Stopped recording... " << std::endl;
    return true;
}

bool AlviumSyncCapture::enableExternalTriggering(void)
{
    if (false == this->isGNSSTriggeringEnabled)
    {
        /* Disable image reception if it is going */
        if (true == this->isImageReceptionEnabled)
        {
            this->stopImageAcquisition();
        }

        /* Configure the camera for external triggering */

        if (false == this->camera.setFeature("LineSelector", "Line0"))
        {
            std::cerr << "Could not set LineSelector" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("LineMode", "Input"))
        {
            std::cerr << "Could not set LineMode" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("LineDebounceMode", "Off"))
        {
            std::cerr << "Could not set LineDebounceMode" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("TriggerSource", "Line0"))
        {
            std::cerr << "Could not set TriggerSource" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("TriggerActivation", "RisingEdge"))
        {
            std::cerr << "Could not set TriggerActivation" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("TriggerDelay", "0"))
        {
            std::cerr << "Could not set TriggerDelay" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("TriggerSelector", "FrameStart"))
        {
            std::cerr << "Could not set TriggerSelector" << std::endl;
            return false;
        }
        else if (false == this->camera.setFeature("TriggerMode", "On"))
        {
            std::cerr << "Could not set TriggerMode" << std::endl;
            return false;
        }
        /* If we are here the camera is configured.*/

        this->isGNSSTriggeringEnabled = true;
        screen.cameraWindow->externalButton->setCaption("Disable Triggering");
        screen.cameraWindow->externalButton->setBackgroundColor(
            screen.cameraWindow->RED);

        this->screen.screen->performLayout();
        std::cout << "Triggering enabled... " << std::endl;
    }

    int frequency;
    if (true == this->getGNSSTriggerFrequency(frequency))
    {
        this->triggerFrequency = frequency;
        this->screen.gnssWindow->triggerFrequencyBox->setValue(frequency);
    }

    return true;
}

bool AlviumSyncCapture::disableExternalTriggering(void)
{
    std::string value;
    if (true == this->isGNSSTriggeringEnabled)
    {
        /* Disable image reception if it is going */
        if (true == this->isImageReceptionEnabled)
        {
            this->stopImageAcquisition();
        }

        /* Configure the camera for external triggering */

        if (false == this->camera.setFeature("TriggerMode", "Off"))
        {
            std::cerr << "Could not set TriggerMode" << std::endl;
            return false;
        }

        if (false == this->camera.getFeature("TriggerMode", value))
        {
            std::cerr << "Could not get TriggerMode" << std::endl;
            return false;
        }
        else if ("Off" != value)
        {
            std::cerr << "TriggerMode is not Off. It is " << value << std::endl;
            return false;
        }

        /* If we are here the camera is configured.*/
    }
    this->isGNSSTriggeringEnabled = false;

    screen.cameraWindow->externalButton->setCaption("Enable Triggering");
    screen.cameraWindow->externalButton->setBackgroundColor(
        screen.cameraWindow->GREEN);
    this->screen.screen->performLayout();
    std::cout << "Triggering Disabled... " << std::endl;

    return true;
}

bool AlviumSyncCapture::enablePPSEvent(void)
{
    if (false == this->camera.activateEvent(
                     "Line1RisingEdge",
                     "EventLine1RisingEdgeTimestamp",
                     cameraPPSCallback,
                     this))
    {
        std::cerr << "Could not activate Line1RisingEdge for PPS event" << std::endl;

        return false;
    }

    return true;
}

void AlviumSyncCapture::cameraPPSCallback(
    std::string eventName,
    int64_t value,
    time_t systemTimestampSeconds,
    time_t systemTimestampNanoseconds,
    void *arg)
{
    AlviumSyncCapture *capture = (AlviumSyncCapture *)arg;

    int64_t oldCameraPPS = capture->lastCameraPPSTimestamp;
    int64_t oldSystemPPS = capture->lastSystemTimeAtCameraPPS;

    capture->lastCameraPPSTimestamp = value;
    capture->lastSystemTimeAtCameraPPS = systemTimestampSeconds + systemTimestampNanoseconds;

    int64_t cameraJitter = (value - oldCameraPPS - 1000000000) / 1000;
    capture->lastCameraTimeJitter = cameraJitter;

    /* Update the camera PPS jtiter */
    capture->screen.gnssWindow->CameraPPSJitter->setValue(
        std::to_string(cameraJitter));

    time_t seconds = systemTimestampSeconds;
    time_t milliseconds = systemTimestampNanoseconds / 1000000;
    if (milliseconds > 700)
    {
        seconds += 1;
    }

    std::tm *tm = std::localtime(&seconds);
    // Format the datetime string
    std::stringstream ss;
    ss << std::put_time(tm, "%Y/%m/%d %H:%M:%S");
    /* Plot of the gui*/
    capture->screen.gnssWindow->CameraSystemLocalPPS->setValue(ss.str());

    /* Convert the time point to UTC time string for display */
    std::tm *tmutc = std::gmtime(&seconds);
    std::stringstream ssutc;
    ssutc << std::put_time(tmutc, "%Y/%m/%d %H:%M:%S");
    /* Plot of the gui*/
    capture->screen.gnssWindow->CameraSystemUTCPPS->setValue(ssutc.str());

    int64_t systemJitter = (capture->lastSystemTimeAtCameraPPS - oldSystemPPS) / 1000;
    capture->lastSystemTimeJitter = systemJitter;
    /* Update the camera PPS jtiter */
    capture->screen.gnssWindow->CameraSystemPPSJitter->setValue(std::to_string(systemJitter));
}

bool AlviumSyncCapture::createNewSaveDirectory(std::string directoryName)
{
    /* We need to create a new directory for the file
     * save location. Then if we are saving average frames
     * we save the frame
     */

    std::string directorypath = this->currentRootSavePath + "/" + directoryName;

    /* Create the images and detections subfolders */
    if (true == std::filesystem::exists(directorypath))
    {
        // Directory already exists...
        std::cerr << "Directory already exists: " << directorypath << std::endl;
    }
    else if (false == std::filesystem::create_directories(directorypath))
    {
        std::cerr << "Could not create directory" << directorypath << std::endl;
        return false;
    }

    this->currentSavePath = directorypath;

    return true;
}

uint64_t AlviumSyncCapture::systemClockToUTC(std::chrono::system_clock::time_point now)
{
    // Convert to time_point in UTC
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::gmtime(&now_time_t); // Convert to UTC time struct

    // Set the time zone to UTC
    now_tm->tm_isdst = 0; // Not Daylight Saving Time

    // Convert back to time_point
    std::chrono::system_clock::time_point utc_time_point = std::chrono::system_clock::from_time_t(std::mktime(now_tm));

    // Convert to microseconds since the Unix epoch
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    uint64_t timestamp = nanoseconds.count();

    return timestamp;
};

bool AlviumSyncCapture::setGNSSTriggerFrequency(int frequency)
{
    std::string command;
    command = "ubxtool -z CFG-TP-FREQ_LOCK_TP2,";
    command += std::to_string(frequency);
    FILE *pipe = popen(command.c_str(), "r");

    if (pipe == nullptr)
    {
        std::cerr << "Failed to open pipe" << std::endl;
        return false;
    }

    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        // Process the output here, e.g., print it to a file or perform other operations
        // You can use string manipulation techniques to extract specific information
    }

    pclose(pipe);

    return true;
}

bool AlviumSyncCapture::getGNSSTriggerFrequency(int &frequency)
{
    FILE *fp = popen("ubxtool -g CFG-TP-FREQ_LOCK_TP2", "r");
    if (fp == nullptr)
    {
        std::cerr << "popen failed\n";
        return false;
    }

    char buffer[1024];
    std::string output;
    while (fgets(buffer, sizeof(buffer), fp) != nullptr)
    {
        output += buffer;
    }

    pclose(fp);

    // Find the position of the target substring
    size_t pos = output.find("CFG-TP-FREQ_LOCK_TP2");
    std::string target_line;
    // If the substring is found
    if (pos != std::string::npos)
    {
        // Find the beginning of the line (assuming lines are separated by '\n')
        size_t line_start = output.rfind('\n', pos);
        // Find the end of the line
        size_t line_end = output.find('\n', pos);

        // Extract the line
        target_line = output.substr(line_start + 1, line_end - line_start - 1);
    }
    else
    {
        std::cout << "CFG-TP-FREQ_LOCK_TP2 not found." << std::endl;
        return false;
    }

    // Find the last word (which contains the integer)
    std::istringstream iss(target_line);
    std::string word;
    while (iss >> word)
    {
    }

    // Convert the last word to an integer
    int newValue;
    std::istringstream(word) >> frequency;

    return true;
}