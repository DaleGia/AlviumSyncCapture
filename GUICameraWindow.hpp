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

        this->syncLabel =
            new nanogui::Label(this, "GNSS Sync (This should be flashing when capturing):", "sans-bold");
        this->syncButton = new nanogui::Button(
            this,
            "");
        this->syncButton->setBackgroundColor(
            RED);

        this->gainLabel =
            new nanogui::Label(this, "Gain (dB):", "sans-bold");
        this->gain = new nanogui::FloatBox<double>(this, 0);
        this->gain->setEditable(true);
        this->exposureLabel =
            new nanogui::Label(this, "Exposure (us):", "sans-bold");
        this->exposure = new nanogui::FloatBox<double>(this, 0);
        this->exposure->setEditable(true);

        new nanogui::Label(this, "max pixel value:", "sans-bold");
        this->maxPixelValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "min pixel value:", "sans-bold");
        this->minPixelValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "average pixel value:", "sans-bold");
        this->averagePixelValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Frames Received:", "sans-bold");
        this->framesReceivedValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Frames Saved:", "sans-bold");
        this->framesSavedValue = new nanogui::IntBox<int>(this, 0);
        new nanogui::Label(this, "Preview Stretch:", "sans-bold");
        previewStretchSlider = new nanogui::Slider(this);
        previewStretchSlider->setRange(std::pair<float, float>(0, 100));
        previewStretchSlider->setValue(100);
    };

    nanogui::FloatBox<double> *gain = nullptr;
    nanogui::FloatBox<double> *triggerRate = nullptr;
    nanogui::FloatBox<double> *exposure = nullptr;
    nanogui::IntBox<int> *maxPixelValue = nullptr;
    nanogui::IntBox<int> *minPixelValue = nullptr;
    nanogui::IntBox<int> *averagePixelValue = nullptr;
    nanogui::IntBox<int> *framesReceivedValue = nullptr;
    nanogui::IntBox<int> *framesSavedValue = nullptr;
    nanogui::Slider *previewStretchSlider = nullptr;
    nanogui::Button *syncButton;

    const nanogui::Color GREEN = nanogui::Color(50, 255, 50, 100);
    const nanogui::Color RED = nanogui::Color(255, 50, 50, 100);

private:
    nanogui::Label *syncLabel = nullptr;
    nanogui::Label *gainLabel = nullptr;
    nanogui::Label *exposureLabel = nullptr;
    nanogui::Label *previewStrechSLabel = nullptr;
};
