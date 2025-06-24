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
        new nanogui::Label(this, "Binning:", "sans-bold");
        this->binning = new nanogui::IntBox<int>(this, 1);
        this->binning->setEditable(true);
        new nanogui::Label(this, "FPS:", "sans-bold");
        this->calculatedFPS = new nanogui::TextBox(this, "");
        new nanogui::Label(this, "Frames Received:", "sans-bold");
        this->framesReceivedValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Frames Saved:", "sans-bold");
        this->framesSavedValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Camera Temperature:", "sans-bold");
        this->temperatureValue = new nanogui::FloatBox<double>(this, 0);

        new nanogui::Label(this, "Preview Max Stretch:", "sans-bold");
        previewStretchSlider = new nanogui::Slider(this);
        previewStretchSlider->setRange(std::pair<float, float>(0.01, 1));
        previewStretchSlider->setValue(1);
        new nanogui::Label(this, "Preview Min Stretch:", "sans-bold");
        previewMinStretchSlider = new nanogui::Slider(this);
        previewMinStretchSlider->setRange(std::pair<float, float>(0, 4096));
        previewMinStretchSlider->setValue(0);
    };

    nanogui::TextBox *pixelFormat = nullptr;
    nanogui::FloatBox<double> *gain = nullptr;
    nanogui::TextBox *calculatedFPS = nullptr;
    nanogui::FloatBox<double> *exposure = nullptr;
    nanogui::IntBox<int> *binning = nullptr;
    nanogui::IntBox<int> *framesReceivedValue = nullptr;
    nanogui::IntBox<int> *framesSavedValue = nullptr;
    nanogui::FloatBox<double> *temperatureValue = nullptr;
    nanogui::Slider *previewStretchSlider = nullptr;
    nanogui::Slider *previewMinStretchSlider = nullptr;

private:
};
