/**
 * @file GUIScreen.hpp
 * Copyright (c) 2023 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include <gps.h>  // .. for gps_*()
#include <thread> // .. for gps_*()
#include <math.h> // for isfinite()
#include <functional>

/*****************************************************************************/
/*MACROS                                                             */
/*****************************************************************************/
#define MODE_STR_NUM 4

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
class GNSS
{
public:
    GNSS();
    ~GNSS();

    void start();

private:
    static void GNSS_loop(void *arg);

    const char *mode_str[MODE_STR_NUM] =
        {
            "n/a",
            "None",
            "2D",
            "3D"};
    std::atomic<bool> GNSSStopFlag;
    std::thread thread;
    struct gps_data_t gpsData;

    std::atomic<double> latitude = 0.0;
    std::atomic<double> longitude = 0.0;
    std::atomic<double> altitudeMSL = 0.0;
    std::atomic<double> timeSeconds = 0.0;
    std::atomic<double> timeNanoSeconds = 0.0;
    std::string mode;
};

GNSS::GNSS()
{
    GNSSStopFlag = false;
}

GNSS::~GNSS()
{
    GNSSStopFlag = true;
    if (true == thread.joinable())
    {
        thread.join();
    }
}

void GNSS::GNSS_loop(void *arg)
{
    GNSS *self = (GNSS *)arg;
    int ret;

    if (0 != gps_open("localhost", "2947", &self->gpsData))
    {
        std::cout << "GNSS Open error.  Bye, bye" << std::endl;
        return;
    }
    else
    {
        std::cout << "GNSS Opened. " << std::endl;
    }

    (void)gps_stream(&self->gpsData, WATCH_ENABLE | WATCH_JSON, NULL);

    while (false == self->GNSSStopFlag)
    {
        gps_waiting(&self->gpsData, 2000000);
        ret = gps_read(&self->gpsData, NULL, 0);
        if (-1 == ret)
        {
            std::cout << "GNSS Read Error... " << ret << std::endl;
            continue;
        }

        /* See if the mode is set */
        if (MODE_SET != (MODE_SET & self->gpsData.set))
        {
            // did not even get mode, nothing to see here
            continue;
        }

        /* Update the fix */
        if (0 > self->gpsData.fix.mode || MODE_STR_NUM <= self->gpsData.fix.mode)
        {
            self->gpsData.fix.mode = 0;
        }

        self->mode = self->mode_str[self->gpsData.fix.mode];

        /* Set the time if the time is set */
        if (TIME_SET == (TIME_SET & self->gpsData.set))
        {
            self->timeSeconds = self->gpsData.fix.time.tv_sec;
            self->timeNanoSeconds = self->gpsData.fix.time.tv_nsec;
        }

        /* Set the lat, lon, if it is valid*/

        if (isfinite(self->gpsData.fix.latitude) && isfinite(self->gpsData.fix.longitude))
        {
            self->latitude = self->gpsData.fix.latitude;
            self->longitude = self->gpsData.fix.longitude;
        }

        if (isfinite(self->gpsData.fix.altMSL))
        {
            self->altitudeMSL = self->gpsData.fix.altMSL;
        }
    }

    // When you are done...
    (void)gps_stream(&self->gpsData, WATCH_DISABLE, NULL);
    (void)gps_close(&self->gpsData);
    std::cout << "GNSS Closed... " << std::endl;
}

void GNSS::start()
{
    thread = std::thread(GNSS::GNSS_loop, this);
}
