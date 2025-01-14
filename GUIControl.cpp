/**
 * @file GUIControl.cpp
 * Copyright (c) 2024 Dale Giancono All rights reserved.
 *
 * @brief
 * TODO add me
 */
/*****************************************************************************/
/*INLCUDES                                                                   */
/*****************************************************************************/
#include "GUIControl.hpp"
#include <thread>

/**************************s***************************************************/
/*MACROS                                                             */
/*****************************************************************************/

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
GUIControl::GUIControl()
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
                600,
                900),
            "AlviumSyncCapture");

    capture =
        new GUICaptureWindow(
            screen,
            250,
            600,
            0,
            0);

    cameraWindow =
        new GUICameraWindow(screen, 325, 600, 0, 250);

    gnssWindow =
        new GUIGNSSWindow(screen, 325, 600, 0, 575);
    this->capture->setVisible(true);
    this->cameraWindow->setVisible(true);
    this->gnssWindow->setVisible(true);
    screen->setVisible(true);
    screen->performLayout();
}

void GUIControl::start()
{
    nanogui::mainloop();
    nanogui::shutdown();
}
