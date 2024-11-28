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

        this->PPSLabel =
            new nanogui::Label(this, "PPS", "sans-bold");
        this->PPSButton = new nanogui::Button(
            this,
            "");
        this->PPSButton->setBackgroundColor(
            RED);

        this->triggerFrequencyLabel =
            new nanogui::Label(this, "Trigger Frequency (Hz):", "sans-bold");
        this->triggerFrequency = new nanogui::IntBox<int>(this, 1);

        this->TimeUTCLabel =
            new nanogui::Label(this, "Time (UTC):", "sans-bold");
        this->TimeUTCBox = new nanogui::TextBox(this, "");
        this->TimeLocalLabel =
            new nanogui::Label(this, "Time (Local):", "sans-bold");
        this->TimeLocalBox = new nanogui::TextBox(this, "");
        this->TimeLocalLabel =
            new nanogui::Label(this, "Location (WGS84):", "sans-bold");
        this->LocationBox = new nanogui::TextBox(this, "");
        this->altitudeMSLLabel =
            new nanogui::Label(this, "Altitude MSL:", "sans-bold");
        this->altitudeMSLBox = new nanogui::TextBox(this, "");
        this->FixLabel =
            new nanogui::Label(this, "Fix:", "sans-bold");
        this->FixBox = new nanogui::TextBox(this, "N/A");
    };

    nanogui::IntBox<int> *triggerFrequency = nullptr;
    nanogui::Button *PPSButton;
    nanogui::TextBox *TimeUTCBox = nullptr;
    nanogui::TextBox *TimeLocalBox = nullptr;
    nanogui::TextBox *LocationBox = nullptr;
    nanogui::TextBox *altitudeMSLBox = nullptr;
    nanogui::TextBox *FixBox = nullptr;

    const nanogui::Color GREEN = nanogui::Color(50, 255, 50, 100);
    const nanogui::Color RED = nanogui::Color(255, 50, 50, 100);

private:
    nanogui::Label *triggerFrequencyLabel = nullptr;
    nanogui::Label *PPSLabel = nullptr;
    nanogui::Label *TimeUTCLabel = nullptr;
    nanogui::Label *TimeLocalLabel = nullptr;
    nanogui::Label *FixLabel = nullptr;
    nanogui::Label *Location = nullptr;
    nanogui::Label *altitudeMSLLabel = nullptr;
};
