/**
 * @file GUIDetect.hpp
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
#include "GUIDetectionSettings.hpp"

/**************************s***************************************************/
/*MACROS                                                             */
/*****************************************************************************/

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
class GUIDetect
{
public:
    GUIDetect();

    ~GUIDetect()
    {
        nanogui::shutdown();
    };

    nanogui::Screen *screen;
    GUICaptureWindow *capture;
    GUICameraWindow *cameraWindow;
    GUIGNSSWindow *gnssWindow;

    nanogui::Screen *detectionScreen;
    GUIDetectionWindow *detectionWindow;

    void
    start();

private:
};
