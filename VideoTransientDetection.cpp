#include "VideoTransientDetection.hpp"
#include "ImageTransientDetection/ImagePreviewWindow.hpp"
/**
 * @brief Constructor for the VideoTransientDetection class.
 *
 * This constructor initializes the VideoTransientDetection object and sets
 * a callback function for when a new input stack is completed. The callback
 * function is responsible for handling the new input stack data and is set
 * using a lambda that binds to the `newdetectionStackCallback` method.
 */

VideoTransientDetection::VideoTransientDetection(bool debugMode) : debugMode(debugMode)
{
    this->detectionStack.setNewStackCallback(
        [this](cv::Mat image,
               double brightnessFactor,
               double accumulatedExposure,
               uint32_t stackCount,
               cv::Mat &lastAddedImage)
        {
            this->newDetectionStackCallback(
                image,
                brightnessFactor,
                accumulatedExposure,
                stackCount,
                lastAddedImage);

            if (nullptr != this->newDetectionStackAvailableCallback)
            {
                this->newDetectionStackAvailableCallback(
                    image,
                    brightnessFactor,
                    accumulatedExposure,
                    stackCount,
                    lastAddedImage);
            }
        });

    this->backgroundStack.setNewStackCallback(
        [this](cv::Mat &image,
               double brightnessFactor,
               double accumulatedExposure,
               uint32_t stackCount,
               cv::Mat &lastAddedImage)
        {
            this->newBackgroundStackCallback(
                image,
                brightnessFactor,
                accumulatedExposure,
                stackCount,
                lastAddedImage);

            if (nullptr != this->newBackgroundStackAvailableCallback)
            {
                this->newBackgroundStackAvailableCallback(
                    image,
                    brightnessFactor,
                    accumulatedExposure,
                    stackCount,
                    lastAddedImage);
            }
        });

    this->imageTransientDetection.setDebugMode(debugMode);
    /* Start off by making this locked */
    this->workerMutex.lock();
}

void VideoTransientDetection::run(void)
{
    bool ret;
    cv::Mat image;
    cv::Mat inputA;
    cv::Mat inputB;
    double brightnessFactorA;
    double brightnessFactorB;

    std::cout << "Starting Video Transient Detection" << std::endl;
    while (false == this->exitFlag.load())
    {
        this->workerMutex.lock();
        if (true == this->exitFlag.load())
        {
            std::cout << "Video Transient Detection: shutdown signal received" << std::endl;
            break;
        }

        /* Check if the current detection have been going on for too long */
        if (true == detectionActiveFlag)
        {

            auto time =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();

            long long difference_ms = time - this->detectionStartTime;

            if (difference_ms > (this->maximumDetectionIntervalS * 1000))
            {
                this->detectionActiveFlag = false;
                this->detectionTailFrameCount = 0;
                this->handlingDetectionFlag = true;
                this->reset();
                this->handlingDetectionFlag = false;

                auto t = std::time(nullptr);
                auto tm = *std::gmtime(&t);
                std::cout << "DETECTION TIMEOUT: " << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl;
                continue;
            }
        }

        // First things first. Grab a copy of the stacks
        /* These are the current image to be accessed */
        inputA = this->detectionImage.clone();
        /* This is the background image that will be diffed against */
        inputB = this->backgroundImage.clone();
        /* These two brightness factors will help compensate for differences
         between exposure, gain, or differences in the number of stacks */
        brightnessFactorA = this->detectionBrightnessFactor;
        brightnessFactorB = this->backgroundBrightnessFactor;

        // Apply brightness factor corrections
        if (brightnessFactorA != brightnessFactorB)
        {
            double ratioA = brightnessFactorA / brightnessFactorB;
            double ratioB = brightnessFactorB / brightnessFactorA;

            if (ratioA > 1)
            {

                inputA *= ratioB;
            }
            else
            {
                inputB *= ratioA;
            }
        }

        /* Perform the detection on the two frames. If it returns true, and bounding
            box, centroid and detection size will be put in those parameters */
        cv::Rect detectionBox;
        ret = this->imageTransientDetection.detect(
            inputA,
            inputB,
            this->userMask,
            detectionBox);

        /* If this is true, a detection should start or continue */
        if (true == ret)
        {
            this->detectionBoundingBox = detectionBox;

            this->detectionTailFrameCount = 0;

            /* If a detection is not active, activate it */
            /* As long as a previous detection has been dealt with */
            if (false == this->detectionActiveFlag)
            {
                if (true == this->detectionsEnabledFlag)
                {
                    /* Reset the detection stack */
                    this->detectionComposite = cv::Mat::zeros(inputA.size(), CV_64F);
                    cv::accumulate(inputA, this->detectionComposite);
                    this->startDetection();
                }
            }
            else
            {
                // If a detection is still going, add to the detection stack image
                cv::accumulate(inputA, this->detectionComposite);
            }

            /* Get the maximum pixel value within the detection to return */
            double maxVal;
            double minVal;
            cv::minMaxLoc(
                this->autoBrightnessImage(this->detectionBoundingBox),
                &minVal,
                &maxVal);
            this->detectionMaxPixelValue = maxVal;
        }
        /* If it is not true, check if the tail count exceeds a certain value*/
        else
        {
            /* This just updates the time in which the detection has stopped. */
            /* If it is 0, it means we want to log the time as it may be the end of the detection (we will not know until the tail count has expired)*/
            if (this->detectionTailFrameCount == 0)
            {

                this->detectionEndTime =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
            }

            /* If detection is active but a transient cannot be found, check how many frames */
            /* have been received since the last transient was found */
            if (true == this->detectionActiveFlag)
            {
                if (this->detectionTailFrameCount < this->prePostDetectionBufferSize)
                {
                    /* Keep on adding to the detection stack */
                    cv::accumulate(inputA, this->detectionComposite);
                }
                /* If it has been over the tail count, the detection is over... finalise everything */
                else
                {
                    /* Set the detection flag to false */
                    auto t = std::time(nullptr);
                    auto tm = *std::gmtime(&t);
                    std::cout << "DETECTION STOPPED: " << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl;
                    this->detectionActiveFlag = false;
                    this->detectionTailFrameCount = 0;

                    this->handlingDetectionFlag = true;

                    /* Reset the tail frame count for the next detection */
                    if (nullptr != this->detectionCallback)
                    {
                        this->detectionCallback(
                            this->predetectionImages,
                            this->detectionImages,
                            this->detectionComposite,
                            this->detectionStartTime,
                            this->detectionEndTime);
                    }

                    this->reset();

                    this->handlingDetectionFlag = false;
                }
            }
        }
    }

    std::cout << "Stopping Video Transient Detection" << std::endl;
}

void VideoTransientDetection::stop(void)
{
    this->exitFlag.store(true);
    this->workerMutex.unlock();
}

/**
 * @brief
 * Set the debug mode for the image transient detection algorithm.
 *
 * When debug mode is enabled, the image transient detection algorithm will
 * display intermediate results in a window to help with debugging. When
 * debug mode is disabled, this window will not be displayed.
 *
 * @param debugMode
 * A boolean indicating whether to enable or disable debug mode.
 */
void VideoTransientDetection::setDebugMode(bool debugMode)
{
    this->imageTransientDetection.setDebugMode(debugMode);
}
/**
 * @brief
 * Set the sigma value for the detection process.
 *
 * @param sigma
 * The sigma value used to calculate the threshold for transient detection.
 * A higher sigma results in a higher threshold, making detection more selective.
 */

void VideoTransientDetection::setDetectionSigma(uint8_t sigma)
{
    this->imageTransientDetection.setSigma(sigma);
}

/**
 * @brief
 * Set the minimum size of a transient object in terms of the number of pixels.
 *
 * @param size
 * The minimum size of a transient object. Any contours with fewer pixels than
 * this will be rejected.
 */
void VideoTransientDetection::setMinimumDetectionPixelSize(uint32_t size)
{
    this->imageTransientDetection.setMinimumSize(size);
}

/**
 * @brief
 * Set the maximum size of a transient object in terms of the number of pixels.
 *
 * @param size
 * The maximum size of a transient object. Any contours with more pixels than
 * this will be rejected.
 */
void VideoTransientDetection::setMaximumDetectionPixelSize(uint32_t size)
{
    this->imageTransientDetection.setMaximumSize(size);
}

void VideoTransientDetection::setMaximumDetectionIntervalS(uint32_t seconds)
{
    this->maximumDetectionIntervalS = seconds;
}

/**
 * @brief
 * Set the target accumulated exposure time for the input stack in microseconds.
 *
 * This method sets the target accumulated exposure time for the input stack.
 * The stack will be updated when the accumulated exposure time of the stack
 * exceeds this value.
 *
 * @param accumulatedExposureTargetUs
 * The target accumulated exposure time for the input stack in microseconds.
 */
void VideoTransientDetection::setDetectionStackExposureTargetUs(
    uint32_t accumulatedExposureTargetUs)
{
    this->detectionStack.setStackMode(StackedImage::MODE::ACCUMULATEDEXPOSURE);
    this->detectionStack.setStackAccumulatedExposure(
        accumulatedExposureTargetUs);
}

void VideoTransientDetection::setDetectionStackImageCount(
    uint64_t imageCount)
{
    this->detectionStack.setStackMode(StackedImage::MODE::IMAGECOUNT);
    this->detectionStack.setStackNumberOfImages(imageCount);
}
void VideoTransientDetection::setNewDetectionStackAvailableCallback(
    std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> callback)
{
    this->newDetectionStackAvailableCallback = callback;
}

void VideoTransientDetection::setNewBackgroundStackAvailableCallback(
    std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> callback)
{
    this->newBackgroundStackAvailableCallback = callback;
}
void VideoTransientDetection::setBackgroundStackExposureTargetUs(
    uint32_t accumulatedExposureTargetUs)
{
    this->backgroundStack.setStackMode(StackedImage::MODE::ACCUMULATEDEXPOSURE);
    this->backgroundStack.setStackAccumulatedExposure(
        accumulatedExposureTargetUs);
}
void VideoTransientDetection::setBackgroundStackImageCount(
    uint64_t imageCount)
{
    this->backgroundStack.setStackMode(StackedImage::MODE::IMAGECOUNT);
    this->backgroundStack.setStackNumberOfImages(imageCount);
}

/**
 * @brief
 * Set the number of frames to store in the pre-detection ring buffer.
 *
 * This method sets the number of frames to store in the pre-detection ring
 * buffer. The pre-detection ring buffer stores the frames before transient
 * detection is performed. The buffer is a ring buffer, so the oldest frames
 * are discarded when the buffer is full.
 *
 * @param size
 * The number of frames to store in the pre-detection ring buffer.
 */
void VideoTransientDetection::setPrePostDetectionBufferSize(uint32_t size)
{
    this->prePostDetectionBufferSize = size;
    predetectionImageRingBuffer =
        std::make_unique<RingBuffer<DetectionFrame>>(this->prePostDetectionBufferSize);
}

/**
 * @brief
 * Set the downsample factor for processing video frames.
 *
 * @param factor
 * The downsample factor to apply. This value determines how much the
 * resolution of the video frames should be reduced. A higher factor
 * results in lower resolution frames, which may speed up processing
 * but reduce detail.
 */
void VideoTransientDetection::setDownsampleFactor(uint8_t factor)
{
    this->downsampleFactor = factor;
}

/**
 * @brief
 * Set the minimum pixel size for data reduction.
 *
 * @param pixelSize
 * The minimum number of pixels in a detection image.
 */
void VideoTransientDetection::setDataReductionMinimumSize(uint32_t pixelSize)
{
    this->dataReductionMinimumSize = pixelSize;
    this->enableDataReduction();
}

void VideoTransientDetection::enableSavingStacks(
    std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> callback,
    uint32_t accumulatedExposureTargetUs)
{
    this->newStackCallback = callback;
    this->stack.setStackMode(StackedImage::MODE::ACCUMULATEDEXPOSURE);
    this->stack.setStackAccumulatedExposure(accumulatedExposureTargetUs);
    this->stack.setNewStackCallback(this->newStackCallback);
    this->saveStackFlag = true;
}

void VideoTransientDetection::disableSavingStacks(void)
{
    this->saveStackFlag = false;
}

void VideoTransientDetection::enableDetections()
{
    this->detectionsEnabledFlag = true;
}

void VideoTransientDetection::disableDetections()
{
    this->detectionsEnabledFlag = false;
    this->reset();
}
/**
 * @brief
 * Enable data reduction in the detection process.
 *
 * This method sets the data reduction flag to true, enabling the data
 * reduction feature. This crops each image in a detection to save less data.
 */
void VideoTransientDetection::enableDataReduction()
{
    this->dataReductionFlag = true;
}

/**
 * @brief
 * Disable data reduction in the detection process.
 *
 * This method sets the data reduction flag to false, disabling the data
 * reduction feature. This results in the full image being stored for each
 * detection.
 */
void VideoTransientDetection::disableDataReduction()
{
    this->dataReductionFlag = false;
}

/**
 * @brief
 * Check if data reduction is enabled in the detection process.
 *
 * This method returns the current state of the data reduction flag.
 * If true, data reduction is enabled and only a subset of the image
 * is stored for each detection. If false, the full image is stored.
 *
 * @return
 * The state of the data reduction flag.
 */
bool VideoTransientDetection::isDataReductionEnabled()
{
    return this->dataReductionFlag;
}

/**
 * @brief
 * Set a user-defined mask to be used in the detection process.
 *
 * @param mask
 * The user-defined mask to be used in the detection process. The mask
 * should be a binary image, where non-zero pixels are used to detect
 * transients and zero pixels are ignored.
 *
 * @details
 * This method sets a user-defined mask to be used in the detection process.
 * The mask is converted to an 8-bit, single-channel image before being
 * used in the detection process. If the mask is not empty, it is cloned
 * and the clone is stored as the user-defined mask.
 */
void VideoTransientDetection::setUserDefinedMask(cv::Mat &mask)
{
    if (mask.empty())
    {
        std::cerr << "Error: User mask is empty" << std::endl;
        return;
    }
    else
    {
        if (mask.type() != CV_8UC1)
        {
            mask.convertTo(this->userMask, CV_8UC1);
        }
        else
        {
            this->userMask = mask.clone();
        }
    }
}

/**
 * @brief Set the callback function for detection events.
 *
 * @param detectionCallback A std::function that will be called when a detection
 * event occurs. The callback receives the following parameters:
 * - A vector of DetectionFrame objects representing the detected frames.
 * - A cv::Mat object containing the image data associated with the detection.
 * - An int64_t representing the start time of the detection event.
 * - An int64_t representing the end time of the detection event.
 *
 * @details This method assigns a user-defined callback function to be executed
 * whenever a transient detection event is triggered. The callback provides
 * detailed information about the detection, including the frames involved,
 * the associated image, and the time interval of the detection.
 */

void VideoTransientDetection::setDetectionCallback(
    std::function<void(std::vector<DetectionFrame>,
                       std::vector<DetectionFrame>,
                       cv::Mat,
                       int64_t,
                       int64_t)>
        detectionCallback)
{
    this->detectionCallback = detectionCallback;
}

/**
 * @brief Process an image frame for transient detection.
 *
 * This function processes a given image frame by adding it to input and background
 * stacks for transient detection. If bright object masking is enabled, the image
 * is also added to the bright object input stack. A `DetectionFrame` is created
 * with metadata from the input frame. If downsampling is used, the detection
 * bounding box is adjusted accordingly. Based on the detection state and data
 * reduction settings, the frame is either added to the pre-detection buffer or
 * processed for detection, potentially applying a crop based on the bounding box
 * and data reduction minimum size.
 *
 * @param frame The input image frame data to be processed.
 */

void VideoTransientDetection::addImage(
    AlliedVisionAlviumPPSSynchronisedFrameData &frame)
{
    DetectionFrame detectionImage;
    cv::Mat resizeImage;

    /* This makes sure there are images saved before and after detections */
    if (true == this->detectionActiveFlag)
    {
        this->detectionTailFrameCount++;
    }

    if (1 < this->downsampleFactor)
    {
        /* If downsampling is used, we need to adjust the detection bounding box
         * so it is compensated for the downsampling. */

        int originalX = static_cast<int>(
            this->detectionBoundingBox.x * this->downsampleFactor);
        int originalY = static_cast<int>(
            this->detectionBoundingBox.y * this->downsampleFactor);

        int originalWidth = static_cast<int>(
            this->detectionBoundingBox.width * this->downsampleFactor);
        int originalHeight = static_cast<int>(
            this->detectionBoundingBox.height * this->downsampleFactor);

        detectionImage.detectionBoundingBox =
            cv::Rect(originalX, originalY, originalWidth, originalHeight);
    }
    else
    {
        detectionImage.detectionBoundingBox = this->detectionBoundingBox;
    }

    detectionImage.frameData.cameraFrameId = frame.cameraFrameId;
    detectionImage.frameData.cameraFrameStartTimestamp =
        frame.cameraFrameStartTimestamp;
    detectionImage.frameData.cameraJitterAtLastCameraPPS =
        frame.cameraJitterAtLastCameraPPS;
    detectionImage.frameData.cameraTimestampAtLastPPS =
        frame.cameraTimestampAtLastPPS;
    detectionImage.frameData.exposureTimeUs = frame.exposureTimeUs;
    detectionImage.frameData.gainDb = frame.gainDb;
    frame.cameraFrameStartTimestamp;
    detectionImage.frameData.height = frame.height;
    detectionImage.frameData.width = frame.width;
    detectionImage.frameData.offsetX = frame.offsetX;
    detectionImage.frameData.offsetY = frame.offsetY;
    detectionImage.frameData.systemTimestampAtLastPPS = frame.systemTimestampAtLastPPS;
    detectionImage.frameData.systemImageReceivedTimestamp =
        frame.systemImageReceivedTimestamp;
    detectionImage.frameData.systemJitterAtLastPPS = frame.systemJitterAtLastPPS;

    /* If we are not in a detection state, just add the image to the
     * predetection buffer
     */
    if (false == this->detectionActiveFlag)
    {
        detectionImage.frameData.image = frame.image.clone();
        this->predetectionImageRingBuffer.get()->push(detectionImage);
    }
    /* Detection is active, lets check if data reduction is enabled */
    /* If it isn't enable, just add the image whole to the detection buffer */
    else if (false == this->isDataReductionEnabled() && false == this->handlingDetectionFlag)
    {
        if (true == detectionImage.detectionBoundingBox.empty())
        {
            std::cerr << "Detection Bounding Box is empty" << std::endl;
        }
        else
        {
            detectionImage.cropBox =
                cv::Rect(0, 0, frame.image.size().width, frame.image.size().height);
            detectionImage.frameData.image = frame.image.clone();
            this->detectionImages.push_back(detectionImage);
        }
    }
    /* If it is enabled, lets crop the image */
    else if (false == this->handlingDetectionFlag)
    {

        if (true == this->detectionBoundingBox.empty())
        {
            std::cerr << "Detection Bounding Box is empty" << std::endl;
        }
        else
        {
            /* We are going to add to the box the datareductionMinimumSize */
            int cropWidth =
                detectionImage.detectionBoundingBox.width +
                this->dataReductionMinimumSize;
            int cropHeight =
                detectionImage.detectionBoundingBox.height +
                this->dataReductionMinimumSize;
            int cropX =
                detectionImage.detectionBoundingBox.x -
                (cropWidth / 2);
            int cropY =
                detectionImage.detectionBoundingBox.y -
                (cropHeight / 2);

            cropX = std::max(0, cropX);
            cropY = std::max(0, cropY);
            cropWidth = std::min(cropWidth, frame.image.cols - cropX);
            cropHeight = std::min(cropHeight, frame.image.rows - cropY);

            detectionImage.cropBox = cv::Rect(cropX, cropY, cropWidth, cropHeight);
            detectionImage.frameData.image = frame.image(detectionImage.cropBox).clone();
            this->detectionImages.push_back(detectionImage);
        }
    }

    double linearGain = std::pow(10.0, frame.gainDb / 20.0);
    double brightnessFactor = frame.exposureTimeUs * linearGain;
    if (1 < this->downsampleFactor)
    {

        int newWidth = frame.image.size().width / this->downsampleFactor;
        int newHeight = frame.image.size().height / this->downsampleFactor;
        try
        {
            cv::resize(
                frame.image,
                resizeImage,
                cv::Size(newWidth, newHeight),
                0,
                0,
                cv::INTER_NEAREST);
            this->detectionStack.add(resizeImage, frame.exposureTimeUs, brightnessFactor);
        }
        catch (const std::exception &e)
        {
            std::cout << "Error resizing image." << e.what() << std::endl;
            return;
        }
    }
    else
    {
        /* Add the image to the input stack */
        this->detectionStack.add(frame.image, frame.exposureTimeUs, brightnessFactor);
        detectionImage.detectionBoundingBox = this->detectionBoundingBox;
    }

    return;
}

/**
 * @brief Resets the state of the VideoTransientDetection object.
 *
 * This method clears all detection-related data and resets the input stacks
 * and flags. It releases any resources associated with the detection process,
 * such as the detection stack, previous input stack, and bright object mask.
 * It also clears the detection images and predetection image buffer, and
 * resets the state of the bright object input stack. The detection active
 * flag is set to false, indicating that the detection process is inactive.
 */

void VideoTransientDetection::reset(void)
{
    this->detectionActiveFlag = false;
    this->predetectionImages.clear();
    this->detectionImages.clear();
    this->detectionComposite.release();
}

/**
 * @brief Callback for processing a new input stack of images.
 *
 * @details This function processes a new input stack of images for transient
 * detection. It combines user-defined and bright object masks, applies downsampling
 * if necessary, and performs transient detection between the current and previous
 * image stacks. If a detection is active, it updates the detection stack and manages
 * the detection tail frame count. If no transient is detected for a certain number
 * of frames, it finalizes the detection and invokes the detection callback.
 *
 * @param stack A reference to the current image stack.
 * @param accumulatedExposureuS The accumulated exposure time in microseconds for
 * the current stack.
 */

void VideoTransientDetection::newDetectionStackCallback(
    cv::Mat &stack,
    double brightnessFactor,
    double accumulatedExposure,
    uint64_t numberOfImages,
    cv::Mat &lastAddedImage)
{
    /* Get the last input and see if it has been set yet*/
    if (true == this->backgroundImage.empty())
    {
        /* If there aint no background image, go twiddle your thumbs
        for a little bit */
    }
    else
    {

        this->detectionImage = stack.clone();
        this->autoBrightnessImage = lastAddedImage.clone();
        this->detectionBrightnessFactor = brightnessFactor;
        this->newdetectionStackCount++;
        this->workerMutex.unlock();
    }

    this->backgroundStack.add(stack, accumulatedExposure, brightnessFactor);
}
void VideoTransientDetection::newBackgroundStackCallback(
    cv::Mat &stack,
    double brightnessFactor,
    double accumulatedExposure,
    uint64_t numberOfImages,
    cv::Mat &lastAddedImage)
{

    this->backgroundImage = stack.clone();
    this->backgroundBrightnessFactor = brightnessFactor;

    if (true == this->saveStackFlag)
    {
        this->stack.add(
            stack,
            accumulatedExposure,
            brightnessFactor);
    }
}

bool VideoTransientDetection::startDetection()
{
    this->detectionCount++;
    /* If a detection is already active, something is wrong, getouttahere*/
    if (true == this->detectionActiveFlag)
    {
        std::cerr << "Detection is already active. Something might be wrong";
        std::cerr << std::endl;
        return false;
    }
    else
    {
        /* This flag is used internally to know to keep frames for saving */
        this->detectionActiveFlag = true;
    }

    /* Get the time for the start of the detection */
    this->detectionStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count();

    cv::Rect convertedDetectionBoundingBox;
    if (1 < this->downsampleFactor)
    {
        int originalX = static_cast<int>(
            this->detectionBoundingBox.x * this->downsampleFactor);
        int originalY = static_cast<int>(
            this->detectionBoundingBox.y * this->downsampleFactor);

        int originalWidth = static_cast<int>(
            this->detectionBoundingBox.width * this->downsampleFactor);
        int originalHeight = static_cast<int>(
            this->detectionBoundingBox.height * this->downsampleFactor);

        convertedDetectionBoundingBox =
            cv::Rect(originalX, originalY, originalWidth, originalHeight);
    }
    else
    {
        convertedDetectionBoundingBox = this->detectionBoundingBox;
    }

    if (false == this->isDataReductionEnabled())
    {
        /* Detection is active, lets check if data reduction is enabled */
        if (true == this->detectionBoundingBox.empty())
        {
            std::cerr << "Error: Detection Bounding Box is empty" << std::endl;
            return false;
        }

        for (int i = 0; i < this->prePostDetectionBufferSize; i++)
        {

            DetectionFrame preDetectionFrame;

            // If this is true all is well.
            if (true == predetectionImageRingBuffer.get()->pop(preDetectionFrame))
            {
                preDetectionFrame.detectionBoundingBox =
                    convertedDetectionBoundingBox;
                preDetectionFrame.cropBox =
                    cv::Rect(
                        0,
                        0,
                        preDetectionFrame.frameData.image.size().width,
                        preDetectionFrame.frameData.image.size().height);
                preDetectionFrame.frameData.image = preDetectionFrame.frameData.image.clone();
                this->predetectionImages.push_back(preDetectionFrame);
            }
            /* If it is false, the buffer might not be full yet... get out of here*/
            else
            {
                break;
            }
        }
    }
    else
    {
        if (true == this->detectionBoundingBox.empty())
        {
            std::cerr << "Detection Bounding Box is empty" << std::endl;
            return false;
        }

        int count = 0;
        for (int i = 0; i < this->prePostDetectionBufferSize; i++)
        {
            /* We are going to add to the box the datareductionMinimumSize */
            int newWidth =
                convertedDetectionBoundingBox.width +
                this->dataReductionMinimumSize;
            int newHeight =
                convertedDetectionBoundingBox.height +
                this->dataReductionMinimumSize;

            int newX =
                convertedDetectionBoundingBox.x -
                (newWidth / 2);
            int newY =
                convertedDetectionBoundingBox.y -
                (newHeight / 2);

            newX = std::max(0, newX);
            newY = std::max(0, newY);

            DetectionFrame preDetectionFrame;

            // If this is true all is well.
            if (true == predetectionImageRingBuffer->pop(preDetectionFrame))
            {
                count++;
                preDetectionFrame.detectionBoundingBox =
                    convertedDetectionBoundingBox;

                newWidth = std::min(
                    newWidth,
                    preDetectionFrame.frameData.image.cols - newX);
                newHeight = std::min(
                    newHeight,
                    preDetectionFrame.frameData.image.rows - newY);
                preDetectionFrame.cropBox =
                    cv::Rect(newX, newY, newWidth, newHeight);

                preDetectionFrame.frameData.image =
                    preDetectionFrame.frameData.image(
                                                   preDetectionFrame.cropBox)
                        .clone();

                this->predetectionImages.push_back(preDetectionFrame);
            }
            /* If it is false, the buffer might not be full yet... get out of here*/
            else
            {
                break;
            }
        }
    }

    auto t = std::time(nullptr);
    auto tm = *std::gmtime(&t);
    std::cout << "DETECTION STARTED: " << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl;

    return true;
}

bool VideoTransientDetection::getDetectionStatus(void)
{
    return this->detectionActiveFlag;
}

bool VideoTransientDetection::getDetectionStatus(double &maxPixelValue)
{
    maxPixelValue = this->detectionMaxPixelValue;
    return this->detectionActiveFlag;
}

void VideoTransientDetection::getStats(
    uint64_t &detectionStackCount,
    uint64_t &detectionCount)
{
    detectionStackCount = this->newdetectionStackCount;
    detectionCount = this->detectionCount;
}

void VideoTransientDetection::getStats(
    uint64_t &detectionStackCount,
    uint64_t &detectionCount,
    ImageTransientDetection::Stats &stats)
{
    detectionStackCount = this->newdetectionStackCount;
    detectionCount = this->detectionCount;
    stats = imageTransientDetection.getLastImageStats();
}
