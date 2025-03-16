#include <nanogui/nanogui.h>

class GUIDetectionWindow : public nanogui::Window
{
public:
    GUIDetectionWindow(
        Widget *parent,
        int height,
        int width,
        int positionx,
        int positiony) : Window(parent, "")
    {
        nanogui::Vector2i position(positionx, positiony);
        setFixedHeight(height);
        setFixedWidth(width);
        setPosition(position);

        this->setLayout(new nanogui::GroupLayout());

        new nanogui::Label(this, "Detection Downsampling Factor:", "sans-bold");
        this->detectionDownsamplingFactor = new nanogui::IntBox<int>(this, 1);

        new nanogui::Label(this, "Detection Sigma:", "sans-bold");
        this->detectionSigma = new nanogui::FloatBox<double>(this, 5);

        new nanogui::Label(this, "Detection Min Size:", "sans-bold");
        this->detectionMinSize = new nanogui::IntBox<int>(this, 1);

        new nanogui::Label(this, "Detection Max Size:", "sans-bold");
        this->detectionMaxSize = new nanogui::IntBox<int>(this, 100);
    };

    nanogui::IntBox<int> *detectionDownsamplingFactor = nullptr;

    nanogui::IntBox<int> *detectionMinSize = nullptr;
    nanogui::IntBox<int> *detectionMaxSize = nullptr;

    nanogui::FloatBox<double> *detectionSigma = nullptr;

private:
};
