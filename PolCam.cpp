#include "PolCam.hpp"

PolCam::PolCam()
{
    for (int row = 0; row < POLCAM_HEIGHT; row++)
    {
        uint32_t polRow;

        if (row == 0)
        {
            polRow = 0;
        }
        else
        {
            polRow = row / 2;
        }

        for (int col = 0; col < POLCAM_WIDTH; col++)
        {
            uint32_t polCol;

            if (col == 0)
            {
                polCol = 0;
            }
            else
            {
                polCol = col / 2;
            }

            if (row % 2)
            {
                if (col % 2)
                {
                    // This means its 0
                    lookup[PolCam::POLARIZATION::POL_0][polRow][polCol][0] = col;
                    lookup[PolCam::POLARIZATION::POL_0][polRow][polCol][1] = row;
                }
                else
                {
                    // This means its 135
                    lookup[PolCam::POLARIZATION::POL_135][polRow][polCol][0] = col;
                    lookup[PolCam::POLARIZATION::POL_135][polRow][polCol][1] = row;
                }
            }
            else
            {
                if (col % 2)
                {
                    // This means its 45
                    lookup[PolCam::POLARIZATION::POL_45][polRow][polCol][0] = col;
                    lookup[PolCam::POLARIZATION::POL_45][polRow][polCol][1] = row;
                }
                else
                {
                    // This means its 90
                    lookup[PolCam::POLARIZATION::POL_90][polRow][polCol][0] = col;
                    lookup[PolCam::POLARIZATION::POL_90][polRow][polCol][1] = row;
                }
            }
        }
    }
}
void PolCam::getFilteredImages(
    cv::Mat &image,
    cv::Mat &pol0,
    cv::Mat &pol45,
    cv::Mat &pol90,
    cv::Mat &pol135)
{
    int imageColumns = image.size().width / 2;
    int imageRows = image.size().height / 2;
    pol0 = cv::Mat(imageRows, imageColumns, image.type());
    pol45 = cv::Mat(imageRows, imageColumns, image.type());
    pol90 = cv::Mat(imageRows, imageColumns, image.type());
    pol135 = cv::Mat(imageRows, imageColumns, image.type());

    if (image.type() == CV_8U)
    {
        for (int col = 0; col < imageColumns; col++)
        {
            for (int row = 0; row < imageRows; row++)
            {
                pol0.at<uint8_t>(row, col) =
                    image.at<uint8_t>(
                        lookup[PolCam::POLARIZATION::POL_0][row][col][1],
                        lookup[PolCam::POLARIZATION::POL_0][row][col][0]);
                pol45.at<uint8_t>(row, col) =
                    image.at<uint8_t>(
                        lookup[PolCam::POLARIZATION::POL_45][row][col][1],
                        lookup[PolCam::POLARIZATION::POL_45][row][col][0]);
                pol90.at<uint8_t>(row, col) =
                    image.at<uint8_t>(
                        lookup[PolCam::POLARIZATION::POL_90][row][col][1],
                        lookup[PolCam::POLARIZATION::POL_90][row][col][0]);
                pol135.at<uint8_t>(row, col) =
                    image.at<uint8_t>(lookup[PolCam::POLARIZATION::POL_135][row][col][1],
                                      lookup[PolCam::POLARIZATION::POL_135][row][col][0]);
            }
        }
    }
    else
    {
        for (int col = 0; col < imageColumns; col++)
        {
            for (int row = 0; row < imageRows; row++)
            {
                pol0.at<uint16_t>(row, col) =
                    image.at<uint16_t>(
                        lookup[PolCam::POLARIZATION::POL_0][row][col][1],
                        lookup[PolCam::POLARIZATION::POL_0][row][col][0]);
                pol45.at<uint16_t>(row, col) =
                    image.at<uint16_t>(
                        lookup[PolCam::POLARIZATION::POL_45][row][col][1],
                        lookup[PolCam::POLARIZATION::POL_45][row][col][0]);
                pol90.at<uint16_t>(row, col) =
                    image.at<uint16_t>(
                        lookup[PolCam::POLARIZATION::POL_90][row][col][1],
                        lookup[PolCam::POLARIZATION::POL_90][row][col][0]);
                pol135.at<uint16_t>(row, col) =
                    image.at<uint16_t>(lookup[PolCam::POLARIZATION::POL_135][row][col][1],
                                       lookup[PolCam::POLARIZATION::POL_135][row][col][0]);
            }
        }
    }
}

void PolCam::getPolarisation(
    cv::Mat &image,
    cv::Mat &pol0,
    cv::Mat &pol45,
    cv::Mat &pol90,
    cv::Mat &pol135,
    cv::Mat &I,
    cv::Mat &P,
    cv::Mat &thetaDeg)
{
    cv::Mat Q;
    cv::Mat U;
    cv::Mat theta;

    cv::Mat sum;
    cv::Mat squareRoot;

    getFilteredImages(image, pol0, pol45, pol90, pol135);

    cv::subtract(pol0, pol90, Q, cv::noArray(), CV_32F);

    cv::subtract(pol45, pol135, U, cv::noArray(), CV_32F);

    cv::add(pol0, pol45, I, cv::noArray(), CV_32F);

    cv::add(I, pol90, I, cv::noArray(), CV_32F);

    cv::add(I, pol135, I, cv::noArray(), CV_32F);

    cv::add(Q.mul(Q, 2), U.mul(U, 2), sum);

    cv::sqrt(Q.mul(Q, 2) + U.mul(U, 2), squareRoot);

    cv::divide(squareRoot, I, P);

    cv::phase(U, Q, thetaDeg, true);

    return;
}
