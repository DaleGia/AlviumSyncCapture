#include <nanogui/nanogui.h>

#include <filesystem>

class GUICameraWindow : public nanogui::Window
{
public:
    GUICameraWindow(
        Widget *parent,
        int height,
        int width,
        int positionx,
        int positiony) : Window(parent, "Camera")
    {
        nanogui::Vector2i position(positionx, positiony);
        setFixedHeight(height);
        setFixedWidth(width);
        setPosition(position);

        nanogui::GridLayout *cameraLayout =
            new nanogui::GridLayout(
                nanogui::Orientation::Horizontal,
                2,
                nanogui::Alignment::Middle,
                15,
                5);

        cameraLayout->setColAlignment(
            {nanogui::Alignment::Maximum,
             nanogui::Alignment::Fill});
        cameraLayout->setRowAlignment(
            {nanogui::Alignment::Maximum,
             nanogui::Alignment::Fill});
        cameraLayout->setSpacing(0, 0);
        this->setLayout(cameraLayout);

        new nanogui::Label(this, "Pixel Format:", "sans-bold");
        this->pixelFormat = new nanogui::TextBox(this);
        this->pixelFormat->setEditable(true);
        new nanogui::Label(this, "Gain (dB):", "sans-bold");
        this->gain = new nanogui::FloatBox<double>(this, 0);
        this->gain->setEditable(true);
        new nanogui::Label(this, "Exposure (us):", "sans-bold");
        this->exposure = new nanogui::FloatBox<double>(this, 0);
        this->exposure->setEditable(true);
        new nanogui::Label(this, "Frames Received:", "sans-bold");
        this->framesReceivedValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Frames Saved:", "sans-bold");
        this->framesSavedValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Preview Stretch:", "sans-bold");
        previewStretchSlider = new nanogui::Slider(this);
        previewStretchSlider->setRange(std::pair<float, float>(0.01, 1));
        previewStretchSlider->setValue(1);
        new nanogui::Label(this, "External Triggering:", "sans-bold");
        this->externalButton = new nanogui::Button(
            this,
            "Enable External Triggering");
        this->externalButton->setBackgroundColor(
            GREEN);
        new nanogui::Label(this, "Camera PPS (Since Camera Power On)", "sans-bold");
        this->CameraPPS = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Camera PPS (Local)", "sans-bold");
        this->CameraSystemLocalPPS = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Camera PPS (UTC)", "sans-bold");
        this->CameraSystemUTCPPS = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Camera PPS Jitter (ns)", "sans-bold");
        this->CameraPPSJitter = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "System PPS Jitter (us)", "sans-bold");
        this->CameraSystemPPSJitter = new nanogui::TextBox(this, "");
    };

    nanogui::TextBox *pixelFormat = nullptr;
    nanogui::FloatBox<double> *gain = nullptr;
    nanogui::FloatBox<double> *triggerRate = nullptr;
    nanogui::FloatBox<double> *exposure = nullptr;
    nanogui::IntBox<int> *framesReceivedValue = nullptr;
    nanogui::IntBox<int> *framesSavedValue = nullptr;
    nanogui::Slider *previewStretchSlider = nullptr;
    nanogui::TextBox *CameraPPS;
    nanogui::TextBox *CameraSystemLocalPPS;
    nanogui::TextBox *CameraSystemUTCPPS;
    nanogui::TextBox *CameraPPSJitter;
    nanogui::TextBox *CameraSystemPPSJitter;

    nanogui::Button *externalButton;

    const nanogui::Color GREEN = nanogui::Color(50, 255, 50, 100);
    const nanogui::Color RED = nanogui::Color(255, 50, 50, 100);

private:
};
