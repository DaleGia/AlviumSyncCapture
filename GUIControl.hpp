/**
 * @file GUIControl.hpp
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

/**************************s***************************************************/
/*MACROS                                                             */
/*****************************************************************************/

/*****************************************************************************/
/* CLASS DECLARATION                                                                      */
/*****************************************************************************/
class GUIControl
{
public:
    GUIControl();

    ~GUIControl()
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
