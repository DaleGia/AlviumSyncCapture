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
#include "nanogui/nanogui.h"
#include "GUICameraWindow.hpp"
#include "GUIGNSSWindow.hpp"
#include "GUICaptureWindow.hpp"
#include <thread>

/**************************s***************************************************/
/*MACROS                                                             */
/*****************************************************************************/

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
class GUIScreen
{
public:
    GUIScreen();

    ~GUIScreen()
    {
        nanogui::shutdown();
    };

    nanogui::Screen *screen;
    GUICaptureWindow *capture;
    GUICameraWindow *cameraWindow;
    GUIGNSSWindow *gnssWindow;

    void start();

private:
};

GUIScreen::GUIScreen()
{
    /*
    So what do we want to do? We want to have a welcome screen saying please
    connect camera. Once a camera is connected, that screen closes and a new
    screen pops up with what is the UI for the camera recording. When the
    camera disconnects this screen disapears and the orginal screen appears
    */
    nanogui::init();

    screen =
        new nanogui::Screen(
            Eigen::Vector2i(
                500,
                925),
            "AlviumSyncCapture");

    capture =
        new GUICaptureWindow(
            screen,
            250,
            500,
            0,
            0);

    cameraWindow =
        new GUICameraWindow(screen, 300, 500, 0, 250);

    gnssWindow =
        new GUIGNSSWindow(screen, 300, 500, 0, 550);
    this->capture->setVisible(true);
    this->cameraWindow->setVisible(true);
    this->gnssWindow->setVisible(false);
    screen->setVisible(true);
    screen->performLayout();
}

void GUIScreen::start()
{
    nanogui::mainloop();
    nanogui::shutdown();
}
