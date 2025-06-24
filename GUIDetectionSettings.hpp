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

        new nanogui::Label(this, "Detection Stack Exposure (ms):", "sans-bold");
        this->detectionStackSize = new nanogui::IntBox<int>(this, 1000);
        this->detectionStackSize->setMinValue(250);
        this->detectionStackSize->setEditable(true);

        new nanogui::Label(this, "Background Stack Exposure (ms):", "sans-bold");
        this->backgroundStackSize = new nanogui::IntBox<int>(this, 1000);
        this->backgroundStackSize->setMinValue(1000);
        this->backgroundStackSize->setEditable(true);

        new nanogui::Label(this, "Pre/Post Detection Buffer Size:", "sans-bold");
        this->prePostDetectionImages = new nanogui::IntBox<int>(this, 10);
        this->prePostDetectionImages->setMinValue(1);
        this->prePostDetectionImages->setEditable(true);

        new nanogui::Label(this, "Detection Sigma:", "sans-bold");
        this->detectionSigma = new nanogui::FloatBox<double>(this, 5);
        this->detectionSigma->setMinValue(1);
        this->detectionSigma->setEditable(true);

        new nanogui::Label(this, "Detection Min Size:", "sans-bold");
        this->detectionMinSize = new nanogui::IntBox<int>(this, 5);
        this->detectionMinSize->setMinValue(1);
        this->detectionMinSize->setEditable(true);

        new nanogui::Label(this, "Detection Max Size:", "sans-bold");
        this->detectionMaxSize = new nanogui::IntBox<int>(this, 100);
        this->detectionMaxSize->setEditable(true);

        new nanogui::Label(this, "Detection Max Duration (S):", "sans-bold");
        this->detectionMaxDuration = new nanogui::IntBox<int>(this, 10);
        this->detectionMaxDuration->setEditable(true);

        new nanogui::Label(this, "Enable Debug Windows:", "sans-bold");
        this->enableDebug = new nanogui::CheckBox(this, "Enable");

        new nanogui::Label(this, "Detection Count:", "sans-bold");
        this->detectionCount = new nanogui::IntBox<int>(this, 0);
        this->detectionCount->setEditable(false);
    };

    nanogui::IntBox<int> *detectionStackSize = nullptr;
    nanogui::IntBox<int> *backgroundStackSize = nullptr;
    nanogui::IntBox<int> *prePostDetectionImages = nullptr;
    nanogui::IntBox<int> *detectionMinSize = nullptr;
    nanogui::IntBox<int> *detectionMaxSize = nullptr;
    nanogui::IntBox<int> *detectionMaxDuration = nullptr;
    nanogui::FloatBox<double> *detectionSigma = nullptr;
    nanogui::CheckBox *enableDebug = nullptr;
    nanogui::IntBox<int> *detectionCount = nullptr;

private:
};
