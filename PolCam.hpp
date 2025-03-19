#ifndef POLCAM_H_
#define POLCAM_H_

/**
 * @file PolCam.hpp
 * Copyright (c) 2024 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
/*****************************************************************************/
/*MACROS                                                             */
/*****************************************************************************/
static const int POLCAM_WIDTH = 2464;
static const int POLCAM_HEIGHT = 2056;
static uint32_t lookup[4][POLCAM_WIDTH][POLCAM_HEIGHT][2];

/*****************************************************************************/
/* CLASS DECLARATION                                                           tem           */
/*****************************************************************************/
class PolCam
{
public:
    PolCam();

    void getFilteredImages(
        cv::Mat &image,
        cv::Mat &pol0,
        cv::Mat &pol45,
        cv::Mat &pol90,
        cv::Mat &pol135);

    void getPolarisation(
        cv::Mat &image,
        cv::Mat &pol0,
        cv::Mat &pol45,
        cv::Mat &pol90,
        cv::Mat &pol135,
        cv::Mat &I,
        cv::Mat &P,
        cv::Mat &thetaDeg);

private:
    enum POLARIZATION
    {
        POL_0 = 0U,
        POL_45 = 1U,
        POL_90 = 2U,
        POL_135 = 3U
    };
};

#endif // PolCam_H_