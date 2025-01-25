#include "../PolCam.hpp"
#include "iostream"
#include "../OpenCVFITS/OpenCVFITS.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    std::cout << "Hi!" << std::endl;
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <filepath>" << std::endl;
        return 1;
    }

    // Load the image
    FITSOpenCV fits;
    std::cout << "Reading " << argv[1] << std::endl;
    cv::Mat image;
    if (false == fits.openFITS(argv[1]))
    {
        std::cerr << "Failed to openFits file: " << argv[1] << std::endl;
        return 1;
    }
    else if (false == fits.getNextImage(image))
    {
        std::cerr << "Failed to get image: " << argv[1] << std::endl;
        return 1;
    }
    std::cout << "Read " << argv[1] << std::endl;

    // Create an instance of the PolCam class

    // Benchmark the execution of each function
    std::cout << "Creating lookup table: " << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    PolCam polcam;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "Lookup table creation time: " << duration << " ms" << std::endl;

    // Benchmark the execution of the PolCam constructor
    cv::Mat P;
    cv::Mat D;
    cv::Mat I;
    cv::Mat pol0;
    cv::Mat pol45;
    cv::Mat pol90;
    cv::Mat pol135;

    start_time = std::chrono::high_resolution_clock::now();
    polcam.getPolarisation(image, pol0, pol45, pol90, pol135, I, P, D);
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "PolCam Processing Time: " << duration << " ms" << std::endl;

    cv::Mat disp;
    cv::normalize(pol0, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Polarisation 0", disp);

    cv::normalize(pol45, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Polarisation 45", disp);

    cv::normalize(pol90, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Polarisation 90", disp);

    cv::normalize(pol135, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Polarisation 135", disp);

    cv::normalize(I, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
    cv::imshow("Intensity", disp);

    cv::normalize(P, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
    cv::imshow("Polarisation Degree", disp);

    cv::normalize(D, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
    cv::imshow("Polarisation Angle", disp);

    cv::waitKey(0);

    return 0;
}