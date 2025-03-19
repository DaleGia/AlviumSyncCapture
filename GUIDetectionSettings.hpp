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

        new nanogui::Label(this, "Detection Stack Size:", "sans-bold");
        this->detectionStackSize = new nanogui::IntBox<int>(this, 1);
        this->detectionStackSize->setMinValue(1);
        this->detectionStackSize->setEditable(true);

        new nanogui::Label(this, "Detection Downsampling Factor:", "sans-bold");
        this->detectionDownsamplingFactor = new nanogui::IntBox<int>(this, 1);
        this->detectionDownsamplingFactor->setMinValue(1);
        this->detectionDownsamplingFactor->setEditable(true);

        new nanogui::Label(this, "Pre/Post Detection Buffer Size:", "sans-bold");
        this->prePostDetectionImages = new nanogui::IntBox<int>(this, 10);
        this->prePostDetectionImages->setMinValue(1);
        this->prePostDetectionImages->setEditable(true);

        new nanogui::Label(this, "Detection Sigma:", "sans-bold");
        this->detectionSigma = new nanogui::FloatBox<double>(this, 5);
        this->detectionSigma->setMinValue(1);
        this->detectionSigma->setEditable(true);

        new nanogui::Label(this, "Detection Min Size:", "sans-bold");
        this->detectionMinSize = new nanogui::IntBox<int>(this, 1);
        this->detectionMinSize->setMinValue(1);
        this->detectionMinSize->setEditable(true);

        new nanogui::Label(this, "Detection Max Size:", "sans-bold");
        this->detectionMaxSize = new nanogui::IntBox<int>(this, 100);
        this->detectionMaxSize->setEditable(true);

        new nanogui::Label(this, "Mask Bright Objects:", "sans-bold");
        this->enableBrightMask = new nanogui::CheckBox(this, "Enable");

        new nanogui::Label(this, "Detection Count:", "sans-bold");
        this->detectionCount = new nanogui::IntBox<int>(this, 0);
        this->detectionCount->setEditable(false);
    };

    nanogui::IntBox<int> *detectionStackSize = nullptr;
    nanogui::IntBox<int> *detectionDownsamplingFactor = nullptr;
    nanogui::IntBox<int> *prePostDetectionImages = nullptr;
    nanogui::IntBox<int> *detectionMinSize = nullptr;
    nanogui::IntBox<int> *detectionMaxSize = nullptr;
    nanogui::FloatBox<double> *detectionSigma = nullptr;

    nanogui::CheckBox *enableBrightMask = nullptr;
    nanogui::IntBox<int> *detectionCount = nullptr;

private:
};
