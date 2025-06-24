#ifndef VIDEOTRANSIENTDETECTION_H
#define VIDEOTRANSIENTDETECTION_H

#include <opencv2/opencv.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <atomic>

#include "OpenCVFITS/OpenCVFITS.hpp"
#include "RingBuffer/RingBuffer.hpp"
#include "ImageTransientDetection/ImageTransientDetection.hpp"
#include "ImageTransientDetection/StackedImage.hpp"
#include "AlliedVisionAlvium/AlliedVisionAlviumPPSSync.hpp"

typedef struct
{
    AlliedVisionAlviumPPSSynchronisedFrameData frameData;
    cv::Rect detectionBoundingBox;
    cv::Rect cropBox;
} DetectionFrame;

class VideoTransientDetection
{
public:
    VideoTransientDetection(bool debugMode = false);
    ~VideoTransientDetection() {};

    void run(void);
    void stop(void);

    void setDetectionSigma(uint8_t sigma);
    void setMinimumDetectionPixelSize(uint32_t size);
    void setMaximumDetectionPixelSize(uint32_t size);
    void setMaximumDetectionIntervalS(uint32_t seconds);
    void setDetectionStackExposureTargetUs(uint32_t accumulatedExposureTargetUs);
    void setDetectionStackImageCount(
        uint64_t imageCount);
    void setNewDetectionStackAvailableCallback(
        std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> callback);
    void setBackgroundStackExposureTargetUs(uint32_t accumulatedExposureTargetUs);
    void setBackgroundStackImageCount(
        uint64_t imageCount);
    void setNewBackgroundStackAvailableCallback(
        std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> callback);
    void setPrePostDetectionBufferSize(uint32_t size);
    void setDownsampleFactor(uint8_t factor);
    void setDataReductionMinimumSize(uint32_t pixelSize);

    void enableSavingStacks(
        std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> callback,
        uint32_t accumulatedExposureTargetUs);
    void disableSavingStacks(void);

    void enableDetections();
    void disableDetections();

    void enableDataReduction();
    void disableDataReduction();
    bool isDataReductionEnabled();

    void setUserDefinedMask(cv::Mat &mask);

    void setDetectionCallback(
        std::function<void(
            std::vector<DetectionFrame>,
            std::vector<DetectionFrame>,
            cv::Mat,
            int64_t,
            int64_t)>
            detectionCallback);

    void addImage(
        AlliedVisionAlviumPPSSynchronisedFrameData &frame);

    bool getDetectionStatus(void);
    bool getDetectionStatus(double &maxPixelValue);
    void getStats(
        uint64_t &detectionStackCount,
        uint64_t &detectionCount);

    void getStats(
        uint64_t &detectionStackCount,
        uint64_t &detectionCount,
        ImageTransientDetection::Stats &stats);

    void setDebugMode(bool debugMode);
    void reset(void);

private:
    void newDetectionStackCallback(
        cv::Mat &stack,
        double brightnessFactor,
        double accumulatedExposure,
        uint64_t numberOfImages,
        cv::Mat &lastAddedImage);

    void newBackgroundStackCallback(
        cv::Mat &stack,
        double brightnessFactor,
        double accumulatedExposure,
        uint64_t numberOfImages,
        cv::Mat &lastAddedImage);

    bool startDetection(void);

    bool detectionsEnabledFlag = true;

    uint64_t newdetectionStackCount = 0;
    uint64_t detectionCount = 0;

    bool debugMode = false;

    bool saveStackFlag = false;
    StackedImage stack;
    std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> newStackCallback;

    bool dataReductionFlag = true;
    uint32_t dataReductionMinimumSize = 100;

    cv::Mat userMask;

    double threshold = 0;

    uint32_t prePostDetectionBufferSize = 1;
    uint8_t downsampleFactor = 1;

    bool detectionActiveFlag = false;
    uint64_t detectionTailFrameCount = 0;
    cv::Rect detectionBoundingBox;
    double detectionMaxPixelValueInBoundingBox;
    int64_t detectionStartTime;
    int64_t detectionEndTime;
    double detectionMaxPixelValue = 0;
    double maximumDetectionIntervalS = 180;

    ImageTransientDetection imageTransientDetection;

    std::function<void(
        std::vector<DetectionFrame>,
        std::vector<DetectionFrame>,
        cv::Mat,
        int64_t,
        int64_t)>
        detectionCallback;
    bool handlingDetectionFlag = false;

    std::unique_ptr<RingBuffer<DetectionFrame>>
        predetectionImageRingBuffer;

    std::vector<DetectionFrame> detectionImages;
    std::vector<DetectionFrame> predetectionImages;

    StackedImage detectionStack;
    cv::Mat detectionImage;
    cv::Mat autoBrightnessImage;
    std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> newDetectionStackAvailableCallback = nullptr;

    double detectionBrightnessFactor;
    StackedImage backgroundStack;
    cv::Mat backgroundImage;
    std::function<void(cv::Mat &, double, double, uint64_t, cv::Mat &)> newBackgroundStackAvailableCallback = nullptr;

    double backgroundBrightnessFactor;

    cv::Mat detectionComposite;

    std::mutex workerMutex;
    std::atomic<bool> exitFlag = false;
};

#endif