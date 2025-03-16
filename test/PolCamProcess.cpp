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

    OpenCVFITS pol0Fits;
    OpenCVFITS pol45Fits;
    OpenCVFITS pol90Fits;
    OpenCVFITS pol135Fits;
    OpenCVFITS intensityFits;
    OpenCVFITS degreeFits;
    OpenCVFITS angleFits;

    std::string pol0Filepath = argv[1] + std::string("pol0.fits");
    std::string pol45Filepath = argv[1] + std::string("pol45.fits");
    std::string pol90Filepath = argv[1] + std::string("pol90.fits");
    std::string pol135Filepath = argv[1] + std::string("pol135.fits");
    std::string intensityFilepath = argv[1] + std::string("intensity.fits");
    std::string degreeFilepath = argv[1] + std::string("degree.fits");
    std::string angleFilepath = argv[1] + std::string("angle.fits");

    if (false == pol0Fits.createFITS(pol0Filepath, true))
    {
        std::cerr << "Failed to create FITS file: " << pol0Filepath << std::endl;
        return 1;
    }
    if (false == pol45Fits.createFITS(pol45Filepath, true))
    {
        std::cerr << "Failed to create FITS file: " << pol45Filepath << std::endl;
        return 1;
    }
    if (false == pol90Fits.createFITS(pol90Filepath, true))
    {
        std::cerr << "Failed to create FITS file: " << pol90Filepath << std::endl;
        return 1;
    }
    if (false == pol135Fits.createFITS(pol135Filepath, true))
    {
        std::cerr << "Failed to create FITS file: " << pol135Filepath << std::endl;
        return 1;
    }
    if (false == intensityFits.createFITS(intensityFilepath, true))
    {
        std::cerr << "Failed to create FITS file: " << intensityFilepath << std::endl;
        return 1;
    }
    if (false == degreeFits.createFITS(degreeFilepath, true))
    {
        std::cerr << "Failed to create FITS file: " << intensityFilepath << std::endl;
        return 1;
    }
    if (false == angleFits.createFITS(angleFilepath, true))
    {
        std::cerr << "Failed to create FITS file: " << intensityFilepath << std::endl;
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

    std::cout << "Read " << argv[1] << std::endl;

    // Create an instance of the PolCam class
    // Benchmark the execution of each function
    std::cout << "Creating lookup table: " << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    PolCam polcam;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "Lookup table creation time: " << duration << " ms" << std::endl;

    while (true == fits.getNextImage(image))
    {

        // Benchmark the execution of the PolCam constructor
        cv::Mat P;
        cv::Mat D;
        cv::Mat I;
        cv::Mat pol0;
        cv::Mat pol45;
        cv::Mat pol90;
        cv::Mat pol135;
        polcam.getPolarisation(image, pol0, pol45, pol90, pol135, I, P, D);

        cv::Mat disp;
        pol0Fits.addMat2FITS(pol0);
        cv::normalize(pol0, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Polarisation 0", disp);

        pol45Fits.addMat2FITS(pol45);
        cv::normalize(pol45, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Polarisation 45", disp);

        pol90Fits.addMat2FITS(pol90);
        cv::normalize(pol90, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Polarisation 90", disp);

        pol135Fits.addMat2FITS(pol135);
        cv::normalize(pol135, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Polarisation 135", disp);

        cv::normalize(I, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
        intensityFits.addMat2FITS(disp);
        cv::imshow("Intensity", disp);

        cv::normalize(P, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
        degreeFits.addMat2FITS(disp);
        cv::imshow("Polarisation Degree", disp);

        cv::normalize(D, disp, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
        cv::imshow("Polarisation Angle", disp);
        angleFits.addMat2FITS(disp);
        cv::waitKey(1);
    }

    pol0Fits.closeFITS();
    pol45Fits.closeFITS();
    pol90Fits.closeFITS();
    pol135Fits.closeFITS();
    return 0;
}