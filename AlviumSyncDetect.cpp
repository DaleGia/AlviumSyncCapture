/**
 * @file AlviumSyncDetect.c
 * Copyright (c) 2023 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include "AlviumSyncDetect.hpp"
#include "argparse/include/argparse/argparse.hpp"
#include "PolCam.hpp"
#include "unistd.h"
#include <cstdlib>

/*****************************************************************************/
/* Class                                                                      */
/*****************************************************************************/
AlviumSyncDetect::AlviumSyncDetect() {
};

void AlviumSyncDetect::run(
    std::string name = "",
    bool polarimetricFlag = false)
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

            // Convert to a time_t object
            time_t gnss_unix_timestamp = timeTotal;
            tm *tm_obj = gmtime(&gnss_unix_timestamp);
            // Format the time using stringstream
            std::stringstream ss;
            ss << std::put_time(tm_obj, "%Y-%m-%d %H:%M:%S");
            this->screen.gnssWindow->TimeUTCBox->setValue(ss.str());
        });

    /* Try to connect to a camera. Fail if you cannot */
    if ("" == name)
    {
        /* No camera name specified. Just try and connect to any camera */
        if (false == camera.connect())
        {
            std::cerr << "Unable to connect to camera..." << std::endl;
        }
    }
    /* If this fails, could not connect to camera. Get OUT. */
    else if (false == camera.connectByUserId(name))
    {
        std::cerr << "Unable to connect to " << name << "..." << std::endl;
    }

    if (true == camera.isCameraOpen())
    {
        /* Disable this to enable full framerate possible */
        if (false == camera.setFeature("DeviceLinkThroughputLimitMode", "Off"))
        {
            std::cerr << "Unable to disable DeviceLinkThroughputLimit... Maximum framerate may be affected..." << std::endl;
        }

        std::string cameraName = camera.getName();
        std::cout << cameraName << std::endl;
        this->screen.capture->connectedCamera->setValue(cameraName);

        /* Create the screen and update the connected camera */
        this->enableExternalTriggering();
        this->updateCameraCapture();
    }

    /* Set what happens when the acquise image button is pushed */
    this->screen.capture->acquireButton->setCallback(
        [this]
        {
            if (false == isImageReceptionEnabled.load())
            {
                if (true == this->isSavingEnabled.load())
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
            if (false == this->isImageReceptionEnabled.load())
            {

                AlliedVisionAlviumPPSSynchronisedFrameData frame;

                /* We are not currently capturing... Lets start capturing*/
                /* update the currently captureured camera values */
                this->updateCameraCapture();
                std::cout << "Camera Capture updated" << std::endl;

                this->screen.capture->acquireSingleButton->setCaption("Getting Image...");
                this->screen.capture->acquireSingleButton->setBackgroundColor(
                    this->screen.capture->RED);

                /* start the camera acquisition */
                if (true == this->camera.getSingleFrame(
                                frame,
                                ACQUIRESINGLEIMAGETIMEOUT_MS))
                    std::cout << "Got Image" << std::endl;

                // Get current time with milliseconds
                auto now = std::chrono::system_clock::now();
                // Extract the number of microseconds
                uint64_t timestamp = systemClockToUTC(now);

                this->receivedFramesCount++;

                /* Get the temperature to write to file. */
                this->temperatureMutex.lock();
                double sensorTemeprature = this->lastSensorTemperature;
                double mainboardTemperature = this->lastMainboardTemperature;
                this->temperatureMutex.unlock();

                if (this->isSavingEnabled.load())
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
                    else if (false == singleFits.addMat2FITS(frame.image))
                    {
                        std::cerr << "Could not add image to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("ExposureTime", frame.exposureTimeUs))
                    {
                        std::cerr << "Could not add ExposureTime to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("Gain", frame.gainDb))
                    {
                        std::cerr << "Could not add FrameID to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("sensorTemperature", sensorTemeprature))
                    {
                        std::cerr << "Could not add sensorTemperature to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("mainboardTemperature", mainboardTemperature))
                    {
                        std::cerr << "Could not add mainboardTemperature to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("cameraFrameStartTimestamp", frame.cameraFrameStartTimestamp))
                    {
                        std::cerr << "Could not add cameraFrameStartTimestamp to fits file " << std::endl;
                    }
                    else if (false ==
                             singleFits.addKey2FITS(
                                 "systemImageReceptionTimestampUTC",
                                 (frame.systemImageReceivedTimestamp)))
                    {
                        std::cerr << "Could not add systemImageReceptionTimestampUTC to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastSystemTimeAtCameraPPSTimestamp", frame.systemTimestampAtLastPPS))
                    {
                        std::cerr << "Could not add lastSystemTimeAtCameraPPSTimestamp to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastSystemTimeJitterAtCameraPPSuS", frame.systemJitterAtLastPPS))
                    {
                        std::cerr << "Could not add lastSystemTimeJitterAtCameraPPSuS to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastCameraPPSTimestamp", frame.cameraTimestampAtLastPPS))
                    {
                        std::cerr << "Could not add lastCameraPPSTimestamp to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastCameraTimeJitterAtCameraPPSuS", frame.cameraJitterAtLastCameraPPS))
                    {
                        std::cerr << "Could not add lastCameraTimeJitterAtCameraPPSuS to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastGNSSPPSTimestamp", frame.lastGNSStimestamp))
                    {
                        std::cerr << "Could not add lastGNSSPPSTimestamp to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastLatitude", frame.lastGNSSLatitude))
                    {
                        std::cerr << "Could not add lastLatitude to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastLongitude", frame.lastGNSSLongitude))
                    {
                        std::cerr << "Could not add lastLatitude to fits file " << std::endl;
                    }
                    else if (false == singleFits.addKey2FITS("lastAltitudeMSL", frame.lastGNSSAltitudeMSL))
                    {
                        std::cerr << "Could not add lastAltitudeMSL to fits file " << std::endl;
                    }
                    else if (true == singleFits.closeFITS())
                    {
                        this->savedFramesCount++;
                    }
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
            if (false == this->isSavingEnabled.load())
            {

                if (true == isImageReceptionEnabled.load())
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

    /* Set what happens when binning is edited */
    screen.cameraWindow->binning->setCallback(
        [this](int value)
        {
            if (false == camera.setFeature("BinningSelector", "Sensor"))
            {
                std::cerr << "Could not set BinningSelector to Sensor. Trying digital..." << std::endl;
            }
            else if (false == camera.setFeature("BinningSelector", "Digital"))
            {
                std::cerr << "Could not set BinningSelector to Digital. Something wrong..." << std::endl;
            }
            else
            {
                std::cout << "Set BinningSelector to Digital" << std::endl;
            }

            if (false == camera.setFeature("BinningHorizontalMode", "Average"))
            {
                std::cerr << "Could not set BinningHorizontalMode..." << std::endl;
            }
            else
            {
                std::cout << "Set BinningHorizontalMode to Average" << std::endl;
            }

            if (false == camera.setFeature("BinningVerticalMode", "Average"))
            {
                std::cerr << "Could not set BinningVerticalMode..." << std::endl;
            }
            else
            {
                std::cout << "Set BinningVerticalMode to Average" << std::endl;
            }

            if (false == camera.setFeature("BinningHorizontal", std::to_string(value)))
            {
                std::cerr << "Could not set BinningHorizontal to " << std::to_string(value) << std::endl;
            }
            else
            {
                std::cout << "Set BinningHorizontal to " << value << std::endl;
            }

            if (false == camera.setFeature("BinningVertical", std::to_string(value)))
            {
                std::cerr << "Could not set BinningVertical to " << std::to_string(value) << std::endl;
            }
            else
            {
                std::cout << "Set BinningVertical to " << value << std::endl;
            }

            std::string val;
            if (false == camera.getFeature("HeightMax", val))
            {
                std::cerr << "Could not get HeightMax" << std::endl;
            }
            else if (false == camera.setFeature("Height", val))
            {
                std::cerr << "Could not set Height to " << val << std::endl;
            }
            else if (false == camera.getFeature("WidthMax", val))
            {
                std::cerr << "Could not get WidthMax" << std::endl;
            }
            else if (false == camera.setFeature("Width", val))
            {
                std::cerr << "Could not set Width to " << val << std::endl;
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

    std::thread temperatureThread(
        [this]
        {
            while (!exitFlag.load())
            {
                if (true == this->camera.isCameraOpen())
                {
                    std::string sensorTemperature = "0";
                    std::string mainBoardTemperature = "0";
                    if (false == this->camera.setFeature("DeviceTemperatureSelector", "Sensor"))
                    {
                        std::cerr << "Could not set DeviceTemperatureSelector to Sensor" << std::endl;
                    }
                    else if (false == this->camera.getFeature("DeviceTemperature", sensorTemperature))
                    {
                        std::cerr << "Could not get sensor temperature" << std::endl;
                    }
                    else if (false == this->camera.setFeature("DeviceTemperatureSelector", "Mainboard"))
                    {
                        std::cerr << "Could not set DeviceTemperatureSelector to Mainboard" << std::endl;
                    }
                    else if (false == this->camera.getFeature("DeviceTemperature", mainBoardTemperature))
                    {
                        std::cerr << "Could not get mainboard temperature" << std::endl;
                    }
                    else
                    {
                        this->temperatureMutex.lock();
                        this->lastSensorTemperature = std::stod(sensorTemperature);
                        this->lastMainboardTemperature = std::stod(mainBoardTemperature);
                        this->temperatureMutex.unlock();
                        this->screen.cameraWindow->temperatureValue->setValue(std::stod(mainBoardTemperature));
                    }
                }

                sleep(60);
            }
        });

    std::thread imagePreviewThread(
        [this, polarimetricFlag]
        {
            if (false == polarimetricFlag)
            {
                ImagePreviewWindow imagePreview("Image Preview");
                imagePreview.setSize(800, 600);

                while (!exitFlag.load())
                {
                    cv::Mat frame;
                    if (false == frame.empty())
                    {
                        imagePreview.setImageStreched(frame, this->screen.cameraWindow->previewStretchSlider->value());
                        this->screen.cameraWindow->framesReceivedValue->setValue(this->receivedFramesCount);
                        this->screen.cameraWindow->framesSavedValue->setValue(this->savedFramesCount);
                    }
                    usleep(100000);
                }
            }
            else
            {
                ImagePreviewWindow imagePreview("Image");
                ImagePreviewWindow pol0Preview("Pol0");
                ImagePreviewWindow pol45Preview("Pol45");
                ImagePreviewWindow pol90Preview("Pol90");
                ImagePreviewWindow pol135Preview("Pol135");

                ImagePreviewWindow PolarisationDegreePreview("Polarisation Degree");
                ImagePreviewWindow PolarisationAnglePreview("Polarisation Angle");

                imagePreview.setSize(600, 400);
                PolarisationDegreePreview.setSize(600, 400);
                PolarisationAnglePreview.setSize(600, 400);
                PolCam polCam;

                while (!exitFlag.load())
                {
                    cv::Mat frame;

                    if (false == frame.empty())
                    {
                        cv::Mat pol0;
                        cv::Mat pol45;
                        cv::Mat pol90;
                        cv::Mat pol135;
                        cv::Mat intensity;
                        cv::Mat polarisationDegree;
                        cv::Mat polarisationAngle;

                        polCam.getPolarisation(
                            frame,
                            pol0,
                            pol45,
                            pol90,
                            pol135,
                            intensity,
                            polarisationDegree,
                            polarisationAngle);

                        imagePreview.setImageStreched(frame, this->screen.cameraWindow->previewStretchSlider->value());
                        pol0Preview.setImageStreched(pol0, this->screen.cameraWindow->previewStretchSlider->value());
                        pol45Preview.setImageStreched(pol45, this->screen.cameraWindow->previewStretchSlider->value());
                        pol90Preview.setImageStreched(pol90, this->screen.cameraWindow->previewStretchSlider->value());
                        pol135Preview.setImageStreched(pol135, this->screen.cameraWindow->previewStretchSlider->value());

                        cv::normalize(polarisationDegree, polarisationDegree, 0, 255, cv::NORM_MINMAX);
                        polarisationDegree.convertTo(
                            polarisationDegree,
                            CV_8UC3);
                        cv::applyColorMap(polarisationDegree, polarisationDegree, cv::COLORMAP_JET);
                        PolarisationDegreePreview.setImageStreched(polarisationDegree, 1);

                        cv::normalize(polarisationAngle, polarisationAngle, 0, 255, cv::NORM_MINMAX);
                        polarisationAngle.convertTo(
                            polarisationAngle,
                            CV_8UC3);
                        cv::applyColorMap(polarisationAngle, polarisationAngle, cv::COLORMAP_JET);
                        PolarisationAnglePreview.setImageStreched(polarisationAngle, 1);

                        this->screen.cameraWindow->framesReceivedValue->setValue(this->receivedFramesCount);
                        this->screen.cameraWindow->framesSavedValue->setValue(this->savedFramesCount);
                    }
                    usleep(100000);
                }
            }
        });

    this->screen.start();
}

void AlviumSyncDetect::frameReceviedFunction(
    AlliedVisionAlviumPPSSynchronisedFrameData &frameData,
    void *arg)
{
    // Get current time with milliseconds
    AlviumSyncDetect *self = (AlviumSyncDetect *)arg;

    self->receivedFramesCount++;

    /* Add frames to detection here */
}

void AlviumSyncDetect::updateCameraCapture(void)
{
    std::string value;
    if (false == this->camera.getFeature("ExposureTime", value))
    {
        std::cerr << "Could not get camera ExposureTime" << std::endl;
    }
    else
    {
        this->screen.cameraWindow->exposure->setValue(std::stod(value));
    }

    if (false == this->camera.getFeature("Gain", value))
    {
        std::cerr << "Could not get camera Gain" << std::endl;
    }
    else
    {
        this->screen.cameraWindow->gain->setValue(std::stod(value));
    }

    if (false == this->camera.getFeature("PixelFormat", value))
    {
        std::cerr << "Could not get camera PixelFormat" << std::endl;
    }
    else
    {
        this->screen.cameraWindow->pixelFormat->setValue(value);
    }

    if (false == this->camera.getFeature("BinningHorizontal", value))
    {
        std::cerr << "Could not get camera BinningHorizontal" << std::endl;
    }
    else
    {
        this->screen.cameraWindow->binning->setValue(std::stoi(value));
    }

    if (false == this->isGNSSTriggeringEnabled.load())
    {
        if (false == this->camera.getFeature("AcquisitionFrameRate", value))
        {
            std::cerr << "Could not get camera AcquisitionFrameRate" << std::endl;
        }
        else
        {
            this->screen.cameraWindow->calculatedFPS->setValue(value);
        }
    }
}

bool AlviumSyncDetect::startImageAcquisition(void)
{
    /* We are not currently capturing... Lets start capturing*/
    /* update the currently captureured camera values */
    this->updateCameraCapture();

    /* start the camera acquisition */
    if (true == this->camera.startAcquisition(IMAGEBUFFERSIZE, AlviumSyncDetect::frameReceviedFunction, this))
    {
        this->isImageReceptionEnabled.store(true);
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
bool AlviumSyncDetect::stopImageAcquisition(void)
{
    /* We are currently capturing... Lets stop capturing*/
    /* Stop the camera acquisition */
    if (true == camera.stopAcquisition())
    {
        this->isImageReceptionEnabled.store(false);
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

bool AlviumSyncDetect::createNewFITS(void)
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

bool AlviumSyncDetect::startImageSaving(void)
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

        this->isSavingEnabled.store(true);
        this->screen.capture->recordingButton->setCaption("Stop Recording");
        this->screen.capture->recordingButton->setBackgroundColor(
            screen.capture->RED);
        std::cout << "Started to record... " << std::endl;

        return true;
    }
}

bool AlviumSyncDetect::stopImageSaving(void)
{
    this->isSavingEnabled.store(false);

    this->fits.closeFITS();

    screen.capture->recordingButton->setCaption("Start Recording");
    screen.capture->recordingButton->setBackgroundColor(
        screen.capture->GREEN);
    std::cout << "Stopped recording... " << std::endl;
    return true;
}

bool AlviumSyncDetect::enableExternalTriggering(void)
{
    if (false == this->isGNSSTriggeringEnabled.load())
    {
        /* Disable image reception if it is going */
        if (true == this->isImageReceptionEnabled.load())
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
        // else if (false == this->camera.setFeature("TriggerMode", "On"))
        // {
        //     std::cerr << "Could not set TriggerMode" << std::endl;
        //     return false;
        // }
        /* If we are here the camera is configured.*/

        this->isGNSSTriggeringEnabled.store(true);
        this->screen.screen->performLayout();
        std::cout << "Triggering enabled... " << std::endl;
    }

    this->screen.cameraWindow->calculatedFPS->setValue("");

    return true;
}

bool AlviumSyncDetect::disableExternalTriggering(void)
{
    std::string value;
    if (true == this->isGNSSTriggeringEnabled.load())
    {
        /* Disable image reception if it is going */
        if (true == this->isImageReceptionEnabled.load())
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
    this->isGNSSTriggeringEnabled.store(false);
    this->screen.screen->performLayout();
    std::cout << "Triggering Disabled... " << std::endl;

    updateCameraCapture();
    return true;
}

void AlviumSyncDetect::cameraPPSCallback(
    int64_t cameraPPSTimestamp,
    int64_t systemPPSTimestamp,
    int64_t cameraPPSJitter,
    int64_t systemPPSJitter,
    void *arg)
{
    AlviumSyncDetect *capture = (AlviumSyncDetect *)arg;
    /* Update the camera PPS jtiter */
    capture->screen.gnssWindow->CameraPPSJitter->setValue(
        std::to_string(cameraPPSJitter));

    time_t seconds = systemPPSTimestamp / 1000000000LL;
    time_t milliseconds = (systemPPSTimestamp % 1000000000LL) / 1000000LL;
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

    /* Update the camera PPS jtiter */
    capture->screen.gnssWindow->CameraSystemPPSJitter->setValue(std::to_string(systemPPSJitter));
}

bool AlviumSyncDetect::createNewSaveDirectory(std::string directoryName)
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

uint64_t AlviumSyncDetect::systemClockToUTC(std::chrono::system_clock::time_point now)
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

bool AlviumSyncDetect::setGNSSTriggerFrequency(int frequency)
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
    std::string output;

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        output += buffer;
    }

    pclose(pipe);

    // Find the position of the target substring
    size_t pos = output.find("CFG-TP-FREQ_LOCK_TP2 not found");
    std::string target_line;
    // If the substring is found
    if (pos != std::string::npos)
    {
        std::cout << "CFG-TP-FREQ_LOCK_TP2 not found." << std::endl;
        return false;
    }

    return true;
}

bool AlviumSyncDetect::getGNSSTriggerFrequency(int &frequency)
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

int main(int argc, const char **argv)
{
    bool polariser;
    std::string cameraName;

    argparse::ArgumentParser parser("AlviumSyncDetect");
    parser.add_argument("--cameraName")
        .help("Name of Camera to connect to. If not used, will attempt to connect to first detected camera");
    parser.add_argument("--polarimetric")
        .flag()
        .help("Use this flag if the camera is a polarimetric camera based off the IMX250MZR");
    parser.add_argument("--listCameras")
        .flag()
        .help("Lists all connected cameras and exits");
    parser.parse_args(argc, argv);

    if (true == parser.is_used("listCameras"))
    {
        std::cout << "Listing Cameras" << std::endl;
        AlliedVisionAlvium cameras;
        std::vector<std::string> userIds = cameras.getUserIds();
        for (auto userId : userIds)
        {
            std::cout << userId << std::endl;
        }
        return 1;
    }

    if (true == parser.is_used("cameraName"))
    {
        cameraName = parser.get<std::string>("cameraName");
    }
    polariser = parser.is_used("polarimetric");

    AlviumSyncDetect AlviumSyncDetect;
    AlviumSyncDetect.run(cameraName, polariser);
    return 1;
}