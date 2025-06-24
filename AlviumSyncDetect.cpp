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
#include "opencv2/opencv.hpp"
#include "AlliedVisionAlvium/AlliedVisionAlviumPPSSync.hpp"
#include "OpenCVFITS/OpenCVFITS.hpp"
#include <mutex>

/*****************************************************************************/
/* Class                                                                      */
/*****************************************************************************/
AlviumSyncDetect::AlviumSyncDetect()
{

    /* Set what happens when the acquise image button is pushed */
    this->screen.capture->acquireButton->setCallback(
        [this]
        {
            if (false == isImageReceptionEnabled.load())
            {
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
                if (true == this->camera.getSingleSyncedFrame(
                                frame,
                                ACQUIRESINGLEIMAGETIMEOUT_MS))
                {
                    std::cout << "Got Image" << std::endl;
                    ImagePreviewWindow singleFrame("Single Image", 800, 600);
                    try
                    {
                        singleFrame.setImageStreched(frame.image, this->screen.cameraWindow->previewStretchSlider->value(), this->screen.cameraWindow->previewMinStretchSlider->value());
                    }
                    catch (const std::exception &e)
                    {
                        std::cerr << e.what() << '\n';
                    }

                    // Get current time with milliseconds
                    auto now = std::chrono::system_clock::now();
                    // Extract the number of microseconds
                    uint64_t timestamp = systemClockToUTC(now);

                    this->receivedFramesCount++;

                    /* Get the temperature to write to file. */

                    double sensorTemeprature = 0;
                    double mainboardTemperature = 0;
                    this->getCameraTemperature(sensorTemeprature, mainboardTemperature);

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

    screen.detectionWindow->detectionStackSize->setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setDetectionStackExposureTargetUs(value * 1000);
        });

    screen.detectionWindow->backgroundStackSize->setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setBackgroundStackExposureTargetUs(value * 1000);
        });

    screen.detectionWindow->prePostDetectionImages->setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setPrePostDetectionBufferSize(value);
        });

    screen.detectionWindow->detectionSigma->setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setDetectionSigma(value);
        });

    screen.detectionWindow->detectionMinSize->setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setMinimumDetectionPixelSize(value);
        });

    screen.detectionWindow->detectionMaxSize->setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setMaximumDetectionPixelSize(value);
        });

    screen.detectionWindow->detectionMaxDuration.setCallback(
        [this](int value)
        {
            this->videoTransientDetection.setMaximumDetectionIntervalS(value);
        });

    screen.detectionWindow->enableDebug->setCallback(
        [this](bool value)
        {
            this->videoTransientDetection.setDebug(value);
        });
};

void AlviumSyncDetect::run(
    std::string name = "")
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
        this->camera.enableSync();
        this->updateCameraCapture();
    }

    this->videoTransientDetection.setDetectionSigma(this->screen.detectionWindow->detectionSigma->value());
    this->videoTransientDetection.setMaximumDetectionPixelSize(this->screen.detectionWindow->detectionMaxSize->value());
    this->videoTransientDetection.setMinimumDetectionPixelSize(this->screen.detectionWindow->detectionMinSize->value());
    this->videoTransientDetection.setMaximumDetectionIntervalS(this->screen.detectionWindow->detectionMaxDuration->value());
    this->videoTransientDetection.setDetectionStackExposureTargetUs(this->screen.detectionWindow->detectionStackSize->value() * 1000);
    this->videoTransientDetection.setBackgroundStackExposureTargetUs(this->screen.detectionWindow->backgroundStackSize->value() * 1000);
    this->videoTransientDetection.setPrePostDetectionBufferSize(this->screen.detectionWindow->prePostDetectionImages->value());
    this->videoTransientDetection.disableDataReduction();
    this->videoTransientDetection.setDebugMode(true);
    this->videoTransientDetection.setDetectionCallback(
        [this](
            std::vector<DetectionFrame> preDetectionFrames,
            std::vector<DetectionFrame> detectionFrames,
            cv::Mat stackedDetectionImage,
            int64_t startTime,
            int64_t endTime)
        {
            this->detectionCount++;
            this->screen.detectionWindow->detectionCount->setValue(this->detectionCount);

            ImagePreviewWindow detection("Detection " + std::to_string(this->detectionCount), 800, 600);
            detection.setImageStreched(this->stackedDetectionImage, 1, 0);
            /* Reset the tail frame count for the next detection */

            if (this->isSavingEnabled.load())
            {
                /* This goes in the callback function in the initialise function */

                writeDetectionToFile(
                    preDetectionFrames,
                    detectionFrames,
                    stackedDetectionImage,
                    startTime,
                    endTime,
                    this);
            }
        });

    std::thread temperatureThread(
        [this]
        {
            while (!exitFlag.load())
            {
                if (false == this->isImageReceptionEnabled)
                {
                    double sensor = 0;
                    double mainboard = 0;
                    getCameraTemperature(sensor, mainboard);
                    if (sensor != 0)
                    {
                        this->screen.cameraWindow->temperatureValue->setValue(sensor);
                    }
                    else if (mainboard != 0)
                    {
                        this->screen.cameraWindow->temperatureValue->setValue(mainboard);
                    }
                    else
                    {
                        this->screen.cameraWindow->temperatureValue->setValue(0);
                    }
                }
                sleep(1);
            }
        });

    std::thread imagePreviewThread(
        [this]
        {
            ImagePreviewWindow imagePreview("Image Preview", 800, 600);

            while (!exitFlag.load())
            {
                cv::Mat frame;
                cv::Mat mask;
                this->lastRecievedImageMutex.lock();
                frame = this->lastRecievedImage.clone();
                this->lastRecievedImageMutex.unlock();

                if (false == frame.empty())
                {
                    imagePreview.setImageStreched(frame, this->screen.cameraWindow->previewStretchSlider->value(), this->screen.cameraWindow->previewMinStretchSlider->value());
                    this->screen.cameraWindow->framesReceivedValue->setValue(this->receivedFramesCount);
                    this->screen.cameraWindow->framesSavedValue->setValue(this->savedFramesCount);
                }
                usleep(100000);
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
    self->lastRecievedImageMutex.lock();
    self->lastRecievedImage = frameData.image.clone();
    self->lastRecievedImageMutex.unlock();
    self->videoTransientDetection.addImage(frameData);
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

    if (false == this->camera.getFeature("AcquisitionFrameRate", value))
    {
        std::cerr << "Could not get camera AcquisitionFrameRate" << std::endl;
    }
    else
    {
        this->screen.cameraWindow->calculatedFPS->setValue(value);
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

    screen.capture->recordingButton->setCaption("Start Recording");
    screen.capture->recordingButton->setBackgroundColor(
        screen.capture->GREEN);
    std::cout << "Stopped recording... " << std::endl;
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

void AlviumSyncDetect::getCameraTemperature(double sensor, double mainboard)
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
            sensor = std::stod(sensorTemperature);
            mainboard = std::stod(mainBoardTemperature);
        }
    }
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

void AlviumSyncDetect::writeDetectionToFile(
    std::vector<DetectionFrame> preDetectionFrames,
    std::vector<DetectionFrame> detectionFrames,
    cv::Mat stackedDetectionImage,
    int64_t startTime,
    int64_t endTime,
    Application *app)
{
    std::string filepath;
    double sensorTemperature;
    double mainboardTemperature;
    uint64_t sensorTimestamp;
    uint64_t mainboardTimestamp;
    OpenCVFITS detectionFits;

    app->temperatureMutex.lock();
    mainboardTemperature = app->mainBoardTemperature;
    sensorTemperature = app->sensorTemperature;
    mainboardTimestamp = app->mainBoardTemperatureTimestamp;
    sensorTimestamp = app->sensorTemperatureTimestamp;
    app->temperatureMutex.unlock();

    filepath = app->createDetectionDirectoryAndFilePath();

    /* Create the fits file*/
    if (false == detectionFits.createFITS(filepath))
    {
        std::cerr << "Could not open detecton to fits file " << filepath << std::endl;
        return;
    }

    cv::normalize(stackedDetectionImage, stackedDetectionImage, 0, 65535, cv::NORM_MINMAX, CV_16U);
    /* Add the detection stack to the fits file*/
    if (false == detectionFits.addMat2FITS(stackedDetectionImage))
    {
        std::cerr << "Could not add detectionStack to fits file " << filepath << std::endl;
        return;
    }
    else if (false == detectionFits.addKey2FITS("detectionStartTime", startTime))
    {
        std::cerr << "Could not add detectionStartTime starttime to fits file " << filepath << std::endl;
        return;
    }
    else if (false == detectionFits.addKey2FITS("detectionEndTime", endTime))
    {
        std::cerr << "Could not add detectionEndTime endtime to fits file " << filepath << std::endl;
        return;
    }

    cv::Mat image;

    /* Now write all of the predetection images */
    for (int i = 0; i < preDetectionFrames.size(); i++)
    {
        if (false == detectionFits.addMat2FITS(preDetectionFrames.at(i).frameData.image))
        {
            std::cerr << "Could not add image to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("ExposureTime", preDetectionFrames.at(i).frameData.exposureTimeUs))
        {
            std::cerr << "Could not add ExposureTime to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("Gain", preDetectionFrames.at(i).frameData.gainDb))
        {
            std::cerr << "Could not add Gain to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("SensorTemperature", sensorTemperature))
        {
            std::cerr << "Could not add SensorTemperature to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("SensorTemperatureTimestamp", sensorTimestamp))
        {
            std::cerr << "Could not add SensorTemperatureTimestamp to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("mainboardTemperature", mainboardTemperature))
        {
            std::cerr << "Could not add mainboardTemperature to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("mainboardTemperatureTimestamp", mainboardTimestamp))
        {
            std::cerr << "Could not add mainboardTemperatureTimestamp to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("FrameID", preDetectionFrames.at(i).frameData.cameraFrameId))
        {
            std::cerr << "Could not add FrameID to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cameraFrameStartTimestamp", preDetectionFrames.at(i).frameData.cameraFrameStartTimestamp))
        {
            std::cerr << "Could not add cameraFrameStartTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "systemImageReceptionTimestampUTC",
                              preDetectionFrames.at(i).frameData.systemImageReceivedTimestamp))
        {
            std::cerr << "Could not add systemImageReceptionTimestampUTC to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("detectionX", (int64_t)preDetectionFrames.at(i).detectionBoundingBox.x))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("detectionY", (int64_t)preDetectionFrames.at(i).detectionBoundingBox.y))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("detectionWidth", (int64_t)preDetectionFrames.at(i).detectionBoundingBox.width))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("detectionHeight", (int64_t)preDetectionFrames.at(i).detectionBoundingBox.height))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropX", (int64_t)preDetectionFrames.at(i).cropBox.x))
        {
            std::cerr << "Could not add y to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropY", (int64_t)preDetectionFrames.at(i).cropBox.y))
        {
            std::cerr << "Could not add size to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropWidth", (int64_t)preDetectionFrames.at(i).cropBox.width))
        {
            std::cerr << "Could not add y to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropHeight", (int64_t)preDetectionFrames.at(i).cropBox.height))
        {
            std::cerr << "Could not add size to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastSystemTimeAtCameraPPSTimestamp",
                              preDetectionFrames.at(i).frameData.systemTimestampAtLastPPS))
        {
            std::cerr << "Could not add lastSystemTimeAtCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastSystemTimeJitterAtCameraPPSuS",
                              preDetectionFrames.at(i).frameData.systemJitterAtLastPPS))
        {
            std::cerr << "Could not add lastSystemTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastCameraPPSTimestamp",
                              preDetectionFrames.at(i).frameData.cameraTimestampAtLastPPS))
        {
            std::cerr << "Could not add lastCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastCameraTimeJitterAtCameraPPSuS",
                              preDetectionFrames.at(i).frameData.cameraJitterAtLastCameraPPS))
        {
            std::cerr << "Could not add lastCameraTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastGNSSPPSTimestamp", preDetectionFrames.at(i).frameData.lastGNSStimestamp))
        {
            std::cerr << "Could not add lastGNSSPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastLatitude", preDetectionFrames.at(i).frameData.lastGNSSLatitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastLongitude", preDetectionFrames.at(i).frameData.lastGNSSLongitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastAltitudeMSL", preDetectionFrames.at(i).frameData.lastGNSSAltitudeMSL))
        {
            std::cerr << "Could not add lastAltitudeMSL to fits file " << std::endl;
        }
    }
    /* Now write all of the detection image */
    for (int i = 0; i < detectionFrames.size(); i++)
    {
        if (false == detectionFits.addMat2FITS(detectionFrames.at(i).frameData.image))
        {
            std::cerr << "Could not add image to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("ExposureTime", detectionFrames.at(i).frameData.exposureTimeUs))
        {
            std::cerr << "Could not add ExposureTime to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("Gain", detectionFrames.at(i).frameData.gainDb))
        {
            std::cerr << "Could not add Gain to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("SensorTemperature", sensorTemperature))
        {
            std::cerr << "Could not add SensorTemperature to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("SensorTemperatureTimestamp", sensorTimestamp))
        {
            std::cerr << "Could not add SensorTemperatureTimestamp to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("mainboardTemperature", mainboardTemperature))
        {
            std::cerr << "Could not add mainboardTemperature to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("mainboardTemperatureTimestamp", mainboardTimestamp))
        {
            std::cerr << "Could not add mainboardTemperatureTimestamp to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("FrameID", detectionFrames.at(i).frameData.cameraFrameId))
        {
            std::cerr << "Could not add FrameID to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cameraFrameStartTimestamp", detectionFrames.at(i).frameData.cameraFrameStartTimestamp))
        {
            std::cerr << "Could not add cameraFrameStartTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "systemImageReceptionTimestampUTC",
                              detectionFrames.at(i).frameData.systemImageReceivedTimestamp))
        {
            std::cerr << "Could not add systemImageReceptionTimestampUTC to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("detectionX", (int64_t)detectionFrames.at(i).detectionBoundingBox.x))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("detectionY", (int64_t)detectionFrames.at(i).detectionBoundingBox.y))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("detectionWidth", (int64_t)detectionFrames.at(i).detectionBoundingBox.width))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("detectionHeight", (int64_t)detectionFrames.at(i).detectionBoundingBox.height))
        {
            std::cerr << "Could not add boundingBoxX to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropX", (int64_t)detectionFrames.at(i).cropBox.x))
        {
            std::cerr << "Could not add y to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropY", (int64_t)detectionFrames.at(i).cropBox.y))
        {
            std::cerr << "Could not add size to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropWidth", (int64_t)detectionFrames.at(i).cropBox.width))
        {
            std::cerr << "Could not add y to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS("cropHeight", (int64_t)detectionFrames.at(i).cropBox.height))
        {
            std::cerr << "Could not add size to fits file " << filepath << std::endl;
            return;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastSystemTimeAtCameraPPSTimestamp",
                              detectionFrames.at(i).frameData.systemTimestampAtLastPPS))
        {
            std::cerr << "Could not add lastSystemTimeAtCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastSystemTimeJitterAtCameraPPSuS",
                              detectionFrames.at(i).frameData.systemJitterAtLastPPS))
        {
            std::cerr << "Could not add lastSystemTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastCameraPPSTimestamp",
                              detectionFrames.at(i).frameData.cameraTimestampAtLastPPS))
        {
            std::cerr << "Could not add lastCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "lastCameraTimeJitterAtCameraPPSuS",
                              detectionFrames.at(i).frameData.cameraJitterAtLastCameraPPS))
        {
            std::cerr << "Could not add lastCameraTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastGNSSPPSTimestamp", detectionFrames.at(i).frameData.lastGNSStimestamp))
        {
            std::cerr << "Could not add lastGNSSPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastLatitude", detectionFrames.at(i).frameData.lastGNSSLatitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastLongitude", detectionFrames.at(i).frameData.lastGNSSLongitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastAltitudeMSL", detectionFrames.at(i).frameData.lastGNSSAltitudeMSL))
        {
            std::cerr << "Could not add lastAltitudeMSL to fits file " << std::endl;
        }
    }

    if (false == app->noCalibrationMode)
    {
        /* Lock the calibration image so we can write it to the file */
        app->lastcalibrationImageMutex.lock();

        if (false == detectionFits.addMat2FITS(app->lastCalibrationImage.image))
        {
            std::cerr << "Could not add image to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("ExposureTime", app->lastCalibrationImage.exposureTimeUs))
        {
            std::cerr << "Could not add ExposureTime to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("Gain", app->lastCalibrationImage.gainDb))
        {
            std::cerr << "Could not add Gain to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("sensorTemperature", sensorTemperature))
        {
            std::cerr << "Could not add sensorTemperature to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("mainboardTemperature", mainboardTemperature))
        {
            std::cerr << "Could not add mainboardTemperature to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("FrameID", app->lastCalibrationImage.cameraFrameId))
        {
            std::cerr << "Could not add FrameID to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("cameraFrameStartTimestamp", app->lastCalibrationImage.cameraFrameStartTimestamp))
        {
            std::cerr << "Could not add cameraImageTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS(
                              "systemImageReceptionTimestampUTC",
                              app->lastCalibrationImage.systemImageReceivedTimestamp))
        {
            std::cerr << "Could not add systemImageReceptionTimestampUTC to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastSystemTimeAtCameraPPSTimestamp", app->lastCalibrationImage.systemTimestampAtLastPPS))
        {
            std::cerr << "Could not add lastSystemTimeAtCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastSystemTimeJitterAtCameraPPSuS", app->lastCalibrationImage.systemJitterAtLastPPS))
        {
            std::cerr << "Could not add lastSystemTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastCameraPPSTimestamp", app->lastCalibrationImage.cameraTimestampAtLastPPS))
        {
            std::cerr << "Could not add lastCameraPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastCameraTimeJitterAtCameraPPSuS", app->lastCalibrationImage.cameraJitterAtLastCameraPPS))
        {
            std::cerr << "Could not add lastCameraTimeJitterAtCameraPPSuS to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastGNSSPPSTimestamp", app->lastCalibrationImage.lastGNSStimestamp))
        {
            std::cerr << "Could not add lastGNSSPPSTimestamp to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastLatitude", app->lastCalibrationImage.lastGNSSLatitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastLongitude", app->lastCalibrationImage.lastGNSSLongitude))
        {
            std::cerr << "Could not add lastLatitude to fits file " << std::endl;
        }
        else if (false == detectionFits.addKey2FITS("lastAltitudeMSL", app->lastCalibrationImage.lastGNSSAltitudeMSL))
        {
            std::cerr << "Could not add lastAltitudeMSL to fits file " << std::endl;
        }

        app->lastcalibrationImageMutex.unlock();
    }

    if (true == detectionFits.closeFITS())
    {
        std::cout << "Saved detection image to " << filepath << std::endl;
    }

    try
    {
        /* code */

        std::string mp4filepath = filepath + ".mp4";
        cv::VideoWriter writer;
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        cv::Size frameSize = cv::Size(250, 250);

        int fps = 30;
        if (0 < app->cameraFrameRate)
        {
            fps = app->cameraFrameRate;
        }
        writer.open(mp4filepath, codec, fps, frameSize, false);
        if (!writer.isOpened())
        {
            std::cerr << "Could not open the output video file for write: " << mp4filepath << std::endl;
            return;
        }

        /* Now write all of the detection image */
        for (int i = 1; i < preDetectionFrames.size() - 1; i++)
        {
            cv::Mat stretchedFrame;
            cv::resize(preDetectionFrames.at(i).frameData.image, stretchedFrame, frameSize);
            cv::normalize(stretchedFrame, stretchedFrame, 0, 255, cv::NORM_MINMAX, CV_8U);
            if (stretchedFrame.empty())
            {
                std::cerr << "Warning: Stretched frame " << i << " is empty. Skipping." << std::endl;
                continue;
            }

            writer.write(stretchedFrame);
        }

        /* Now write all of the detection image */
        for (int i = 1; i < detectionFrames.size() - 1; i++)
        {
            cv::Mat stretchedFrame;
            cv::resize(detectionFrames.at(i).frameData.image, stretchedFrame, frameSize);
            cv::normalize(stretchedFrame, stretchedFrame, 0, 255, cv::NORM_MINMAX, CV_8U);
            if (stretchedFrame.empty())
            {
                std::cerr << "Warning: Stretched frame " << i << " is empty. Skipping." << std::endl;
                continue;
            }

            writer.write(stretchedFrame);
        }

        writer.release(); // Release the VideoWriter
        std::cout << "Finished writing video to " << filepath << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Unable to write video file: " << filepath << ": " << e.what() << '\n';
    }

    try
    {
        std::string command;
        command = "python3 /usr/local/bin/BlueSkyReportingV2.py ";
        command += filepath;
        system(command.c_str());
        std::cout << "Posted detection to bluesky" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Unable to post to bluesky: " << filepath << ": " << e.what() << '\n';
    }
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

    AlviumSyncDetect AlviumSyncDetect;
    AlviumSyncDetect.run(cameraName);
    return 1;
}
