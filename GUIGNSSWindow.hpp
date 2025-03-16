#ifndef GUIGNSSWindow_h_
#define GUIGNSSWindow_h_

#include <nanogui/nanogui.h>
#include <filesystem>

class GUIGNSSWindow : public nanogui::Window
{
public:
    GUIGNSSWindow(
        Widget *parent,
        int height,
        int width,
        int positionx,
        int positiony) : Window(parent, "GNSS")
    {
        nanogui::Vector2i position(positionx, positiony);
        setFixedHeight(height);
        setFixedWidth(width);
        setPosition(position);

        nanogui::GridLayout *GNSSLayout =
            new nanogui::GridLayout(
                nanogui::Orientation::Horizontal,
                2,
                nanogui::Alignment::Middle,
                15,
                5);

        GNSSLayout->setColAlignment(
            {nanogui::Alignment::Maximum,
             nanogui::Alignment::Fill});
        GNSSLayout->setRowAlignment(
            {nanogui::Alignment::Maximum,
             nanogui::Alignment::Fill});
        GNSSLayout->setSpacing(0, 0);
        this->setLayout(GNSSLayout);

        new nanogui::Label(this, "GNSS Time (UTC):", "sans-bold");
        this->TimeUTCBox = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Camera PPS (UTC)", "sans-bold");
        this->CameraSystemUTCPPS = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Camera PPS (Local)", "sans-bold");
        this->CameraSystemLocalPPS = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Camera PPS Jitter (us)", "sans-bold");
        this->CameraPPSJitter = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "System PPS Jitter (us)", "sans-bold");
        this->CameraSystemPPSJitter = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Location (WGS84):", "sans-bold");
        this->LocationBox = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Altitude MSL:", "sans-bold");
        this->altitudeMSLBox = new nanogui::TextBox(this, "");
    };

    nanogui::TextBox *TimeUTCBox = nullptr;
    nanogui::TextBox *CameraSystemLocalPPS;
    nanogui::TextBox *CameraSystemUTCPPS;
    nanogui::TextBox *CameraPPSJitter;
    nanogui::TextBox *CameraSystemPPSJitter;
    nanogui::TextBox *LocationBox = nullptr;
    nanogui::TextBox *altitudeMSLBox = nullptr;

    const nanogui::Color GREEN = nanogui::Color(50, 255, 50, 100);
    const nanogui::Color RED = nanogui::Color(255, 50, 50, 100);

private:
};

#endif // GUIGNSSWindow_h_
