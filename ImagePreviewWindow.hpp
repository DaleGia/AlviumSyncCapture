#ifndef IMAGEPREVIEWWINDOW_H
#define IMAGEPREVIEWWINDOW_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
class ImagePreviewWindow
{
public:
    ImagePreviewWindow(std::string windowName) : name(windowName)
    {
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::resizeWindow(name, 800, 600);
        cv::setMouseCallback(name, ImagePreviewWindow::onMouseWheel, this);
        cv::namedWindow(name + " zoom");
    };

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
                currentImage,
                CV_8UC3);
        }
        else
        {
            image.convertTo(
                currentImage,
                CV_8UC1);
        }

        cv::imshow(name, currentImage);
        cv::waitKey(50);
    };

    void setImageStreched(cv::Mat &image, double percent)
    {
        /* Convert the image to a strech RGBA image */
        double minVal, maxVal;
        bool isColour;
        int srcType;

        if (image.empty())
        {
            return;
        }

        srcType = image.type();
        // Check for common color and grayscale formats:
        isColour =
            image.type() ==
                CV_8UC3 ||
            srcType == CV_16UC3 || srcType == CV_32FC3;

        cv::minMaxLoc(image, &minVal, &maxVal);
        maxVal = maxVal * percent;

        double newMaxVal = maxVal + (maxVal - minVal);
        if (true == isColour)
        {
            image.convertTo(
                currentImage,
                CV_8UC3,
                255.0 / (newMaxVal - minVal), -minVal * 255.0 / (newMaxVal - minVal));
        }
        else
        {
            image.convertTo(
                this->currentImage,
                CV_8UC1,
                255.0 / (newMaxVal - minVal), -minVal * 255.0 / (newMaxVal - minVal));
        }

        cv::imshow(name, this->currentImage);

        if (true == this->zoomEnabled)
        {
            // Assuming x and y are the coordinates of the center of the ROI

            // Calculate the top-left corner of the ROI, ensuring it's within the image boundaries
            int x1 = std::max(0, this->xzoom - 50);
            int y1 = std::max(0, this->yzoom - 50);

            // Calculate the bottom-right corner, ensuring it's within the image boundaries
            int x2 = std::min(this->currentImage.cols - 1, this->xzoom + 50);
            int y2 = std::min(this->currentImage.rows - 1, this->yzoom + 50);

            // Create a rectangle for the ROI
            cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);

            // Extract the ROI from the image
            cv::Mat cropped_img = this->currentImage(roi);
            cv::resize(cropped_img, cropped_img, cv::Size(400, 400));
            cv::imshow(this->name + " zoom", cropped_img);
        }
        cv::waitKey(50);
    }

private:
    cv::Mat currentImage;
    bool zoomEnabled = false;
    int xzoom = 0;
    int yzoom = 0;
    std::string name;

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
            }
        }

        if (true == window->zoomEnabled)
        {
            // Assuming x and y are the coordinates of the center of the ROI

            // Calculate the top-left corner of the ROI, ensuring it's within the image boundaries
            int x1 = std::max(0, x - 50);
            int y1 = std::max(0, y - 50);

            // Calculate the bottom-right corner, ensuring it's within the image boundaries
            int x2 = std::min(window->currentImage.cols - 1, x + 50);
            int y2 = std::min(window->currentImage.rows - 1, y + 50);

            // Create a rectangle for the ROI
            cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);

            // Extract the ROI from the image
            cv::Mat cropped_img = window->currentImage(roi);
            cv::resize(cropped_img, cropped_img, cv::Size(400, 400));
            cv::imshow(window->name + " zoom", cropped_img);
        }
        else
        {
            if (cv::getWindowProperty(window->name + " zoom", cv::WND_PROP_VISIBLE))
            {
                cv::destroyWindow(window->name + " zoom");
            }
        }
    };
};
#endif // IMAGEPREVIEWWINDOW_H
