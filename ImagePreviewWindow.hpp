#ifndef IMAGEPREVIEWWINDOW_H
#define IMAGEPREVIEWWINDOW_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
class ImagePreviewWindow
{
public:
    ImagePreviewWindow(std::string windowName) : name(windowName), zoomName(windowName + " zoom"), statName(windowName + " stats")
    {
        cv::namedWindow(name, cv::WINDOW_NORMAL || cv::WindowFlags::WINDOW_KEEPRATIO);
        cv::setMouseCallback(name, ImagePreviewWindow::onMouseWheel, this);
    };

    void setSize(uint16_t width, uint16_t height)
    {
        cv::setWindowProperty(this->name, cv::WindowPropertyFlags::WND_PROP_ASPECT_RATIO, width / height);
        cv::resizeWindow(this->name, width, height);
    }

    void setImage(cv::Mat &image)
    {
        /* Convert the image to a strech RGBA image */
        double minVal, maxVal;
        bool isColour;
        int srcType;

        if (image.empty())
        {
            return;
        }

        this->currentImage = image.clone();

        srcType = image.type();
        // Check for common color and grayscale formats:
        isColour =
            image.type() ==
                CV_8UC3 ||
            srcType == CV_16UC3 || srcType == CV_32FC3;

        cv::minMaxLoc(image, &minVal, &maxVal);

        if (true == isColour)
        {
            image.convertTo(
                currentDisplayImage,
                CV_8UC3);
        }
        else
        {
            image.convertTo(
                currentDisplayImage,
                CV_8UC1);
        }

        cv::imshow(this->name, currentDisplayImage);
        cv::waitKey(50);
    };

    void setImageStreched(cv::Mat &image, double percent)
    {
        /* Convert the image to a strech RGBA image */
        double minVal;
        double maxVal;
        double statsMin;
        double statsMax;
        cv::Scalar std;
        cv::Scalar mean;
        bool isColour;
        int srcType;

        if (image.empty())
        {
            return;
        }

        currentImage = image.clone();

        srcType = image.type();
        // Check for common color and grayscale formats:
        isColour =
            image.type() ==
                CV_8UC3 ||
            srcType == CV_16UC3 || srcType == CV_32FC3;

        cv::minMaxLoc(image, &statsMin, &statsMax);
        cv::meanStdDev(image, mean, std);

        maxVal = statsMax * percent;
        double newMaxVal = maxVal + (maxVal - statsMin);

        if (true == isColour && CV_8UC3)
        {
            image.convertTo(
                currentDisplayImage,
                CV_8UC3,
                255.0 / (newMaxVal - minVal), -minVal * 255.0 / (newMaxVal - minVal));
        }
        else if (true == isColour && CV_16UC3)
        {
            image.convertTo(
                currentDisplayImage,
                CV_8UC3,
                65525.0 / (newMaxVal - minVal), -minVal * 65525.0 / (newMaxVal - minVal));
        }
        else if (image.type() == CV_8UC1)
        {
            image.convertTo(
                this->currentDisplayImage,
                CV_8UC1,
                255.0 / (newMaxVal - minVal), -minVal * 255.0 / (newMaxVal - minVal));
        }
        else if (image.type() == CV_16UC1)
        {
            image.convertTo(
                this->currentDisplayImage,
                CV_8UC1,
                4095.0 / (newMaxVal - minVal), -minVal * 4095.0 / (newMaxVal - minVal));
        }
        else
        {
            cv::normalize(image, currentDisplayImage, 0, 255, cv::NORM_MINMAX);
            currentDisplayImage.convertTo(
                currentDisplayImage,
                CV_8UC3);
        }
        std::string title = name + " Min: " + std::to_string(static_cast<int>(statsMin)) + " Max: " + std::to_string(static_cast<int>(statsMax)) + " Mean: " + std::to_string(static_cast<int>(mean[0])) + " STD: " + std::to_string(static_cast<int>(std[0]));

        cv::setWindowTitle(this->name, title);
        cv::imshow(this->name, this->currentDisplayImage);

        if (true == this->zoomEnabled)
        {
            this->showZoomed();
        }

        if (true == this->statEnabled)
        {
            this->showZoomedStats();
        }
        cv::waitKey(50);
    }

private:
    cv::Mat currentDisplayImage;
    cv::Mat currentImage;
    bool zoomEnabled = false;
    bool statEnabled = false;
    int xzoom = 0;
    int yzoom = 0;
    std::string name;
    std::string zoomName;
    std::string statName;

    void showZoomed(void)
    {
        // Calculate the top-left corner of the ROI, ensuring it's within the image boundaries
        int x1 = std::max(0, this->xzoom - 50);
        int y1 = std::max(0, this->yzoom - 50);

        // Calculate the bottom-right corner, ensuring it's within the image boundaries
        int x2 = std::min(this->currentDisplayImage.cols - 1, this->xzoom + 50);
        int y2 = std::min(this->currentDisplayImage.rows - 1, this->yzoom + 50);

        // Create a rectangle for the ROI
        cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);

        // Extract the ROI from the image
        cv::Mat cropped_img = this->currentDisplayImage(roi).clone();
        cv::resize(cropped_img, cropped_img, cv::Size(400, 400));
        cv::imshow(this->zoomName, cropped_img);
    };

    void showZoomedStats(void)
    {
        cv::Scalar std;
        cv::Scalar mean;
        double statsMin;
        double statsMax;
        bool isColour;

        // Calculate the top-left corner of the ROI, ensuring it's within the image boundaries
        int x1 = std::max(0, this->xzoom - 25);
        int y1 = std::max(0, this->yzoom - 25);

        // Calculate the bottom-right corner, ensuring it's within the image boundaries
        int x2 = std::min(this->currentDisplayImage.cols - 1, this->xzoom + 25);
        int y2 = std::min(this->currentDisplayImage.rows - 1, this->yzoom + 25);

        // Create a rectangle for the ROI
        cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);

        // Extract the ROI from the image
        cv::Mat cropped_img = this->currentImage(roi).clone();

        cv::meanStdDev(cropped_img, mean, std);
        cv::minMaxLoc(cropped_img, &statsMin, &statsMax);

        if (true == isColour)
        {
            cropped_img.convertTo(
                cropped_img,
                CV_8UC3,
                255.0 / (statsMax - statsMin), -statsMin * 255.0 / (statsMax - statsMin));
        }
        else
        {
            cropped_img.convertTo(
                cropped_img,
                CV_8UC1,
                255.0 / (statsMax - statsMin), -statsMin * 255.0 / (statsMax - statsMin));
        }
        cv::resize(cropped_img, cropped_img, cv::Size(400, 400), 0, 0, cv::INTER_NEAREST);
        std::string title = "Min: " + std::to_string((int)statsMin) + " Max: " + std::to_string((int)statsMax) + " Mean: " + std::to_string((int)mean[0]) + " STD: " + std::to_string((int)std[0]);
        cv::setWindowTitle(this->statName, title);
        cv::imshow(this->statName, cropped_img);
    }

    static void onMouseWheel(int event, int x, int y, int flags, void *userdata)
    {
        ImagePreviewWindow *window = (ImagePreviewWindow *)userdata;

        if (event == cv::EVENT_MOUSEMOVE)
        {
            window->xzoom = x;
            window->yzoom = y;
        }

        else if (event == cv::EVENT_LBUTTONDOWN)
        {
            if (false == window->zoomEnabled)
            {
                window->zoomEnabled = true;
            }
            else
            {
                window->zoomEnabled = false;
                cv::destroyWindow(window->zoomName);
            }
        }
        else if (event == cv::EVENT_RBUTTONDOWN)
        {
            if (false == window->statEnabled)
            {
                window->statEnabled = true;
            }
            else
            {
                window->statEnabled = false;
                cv::destroyWindow(window->statName);
            }
        }

        if (true == window->zoomEnabled)
        {
            window->showZoomed();
        }

        if (true == window->statEnabled)
        {
            window->showZoomedStats();
        }
    };
};
#endif // IMAGEPREVIEWWINDOW_H
