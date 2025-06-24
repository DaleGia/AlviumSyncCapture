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
    cv::Mat &Intensity,
    cv::Mat &P,
    cv::Mat &thetaDeg)
{

    cv::Mat squareRoot;

    cv::Mat S0;
    cv::Mat S0_2;

    cv::Mat S1;
    cv::Mat S1_2;

    cv::Mat S2;
    cv::Mat S2_2;

    cv::Mat S3;

    cv::Mat S1_2_plus_S2_2;

    getFilteredImages(image, pol0, pol45, pol90, pol135);

    // /* Calculates stokes parameters*/
    cv::add(pol0, pol90, S0, cv::noArray(), CV_32F);
    cv::subtract(pol0, pol90, S1, cv::noArray(), CV_32F);
    cv::subtract(pol45, pol135, S2, cv::noArray(), CV_32F);

    P = cv::Mat::zeros(S0.size(), CV_32F);

    for (int y = 0; y < S0.rows; ++y)
    {
        for (int x = 0; x < S0.cols; ++x)
        {
            float s0 = S0.at<float>(y, x);
            if (s0 == 0)
            {
                s0 = 1;
            }
            float s1 = S1.at<float>(y, x);
            float s2 = S2.at<float>(y, x);
            P.at<float>(y, x) = std::sqrt(s1 * s1 + s2 * s2) / (s0);
            if (std::isnan(P.at<float>(y, x)))
            {
                P.at<float>(y, x) = 0;
            }
        }
    }

    thetaDeg = cv::Mat::zeros(S0.size(), CV_32F);

    for (int y = 0; y < S1.rows; ++y)
    {
        for (int x = 0; x < S1.cols; ++x)
        {
            float s1 = S1.at<float>(y, x);
            float s2 = S2.at<float>(y, x);
            thetaDeg.at<float>(y, x) = std::atan2(s2, s1) * 180 / M_PI;
        }
    }
    double min, max;
    cv::minMaxLoc(S0, &min, &max);
    std::cout << "S0 min: " << min << " max: " << max << std::endl;
    cv::minMaxLoc(S1, &min, &max);

    std::cout << "S1 min: " << min << " max: " << max << std::endl;
    cv::minMaxLoc(S2, &min, &max);
    std::cout << "S2 min: " << min << " max: " << max << std::endl;
    cv::minMaxLoc(P, &min, &max);
    // Calculate and print the average intensity of S0
    cv::Scalar mean = cv::mean(P);
    std::cout
        << "P min: " << min << " max: " << max << ", mean: " << mean[0] << std::endl
        << std::endl;
    cv::minMaxLoc(thetaDeg, &min, &max);
    std::cout << "deg min: " << min << " max: " << max << std::endl
              << std::endl;

    /* Creates overall intensity */
    cv::add(pol0, pol45, Intensity, cv::noArray(), CV_32F);
    cv::add(Intensity, pol90, Intensity, cv::noArray(), CV_32F);
    cv::add(Intensity, pol135, Intensity, cv::noArray(), CV_32F);

    // cv::Mat Q;
    // cv::Mat Q_2;
    // cv::Mat U;
    // cv::Mat U_2;
    // cv::Mat I;
    // cv::Mat sum;

    // cv::add(pol0, pol90, I, cv::noArray(), CV_32F);
    // cv::subtract(pol0, pol90, Q, cv::noArray(), CV_32F);
    // cv::subtract(pol45, pol135, U, cv::noArray(), CV_32F);

    // /* Get total intensity*/
    // cv::add(pol0, pol45, Intensity, cv::noArray(), CV_32F);
    // cv::add(Intensity, pol90, Intensity, cv::noArray(), CV_32F);
    // cv::add(Intensity, pol135, Intensity, cv::noArray(), CV_32F);

    // cv::multiply(Q, Q, Q_2);
    // cv::multiply(U, U, U_2);

    // cv::add(Q_2, U_2, sum);
    // cv::sqrt(sum, squareRoot);

    // I = I + 1;
    // cv::divide(squareRoot, I, P);
    // double min, max;
    // cv::Scalar mean;
    // cv::minMaxLoc(P, &min, &max);
    // cv::phase(U, Q, thetaDeg, true);

    return;
}
