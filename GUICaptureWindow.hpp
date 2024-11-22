#include <nanogui/nanogui.h>

class GUICaptureWindow : public nanogui::Window
{
public:
    GUICaptureWindow(
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

        this->connectedCamera = new nanogui::TextBox(this, "No Camera Connected");

        this->acquireButton = new nanogui::Button(
            this,
            "Start acquiring images");
        this->acquireButton->setBackgroundColor(
            GREEN);

        this->acquireSingleButton = new nanogui::Button(
            this,
            "Acquire single image");
        this->acquireSingleButton->setBackgroundColor(
            GREEN);

        this->recordingButton = new nanogui::Button(
            this,
            "Start saving images");
        this->recordingButton->setBackgroundColor(
            GREEN);

        this->fileDialogButton = new nanogui::Button(this, "Set save Location");
        this->fileDialogButton->setCallback(
            [&]
            {
                try
                {
                    char buffer[1000];
                    std::string string;
                    FILE *output = popen("zenity --file-selection --directory", "r");
                    if (output == nullptr)
                    {
                        throw std::runtime_error("popen() failed -- could not launch zenity!");
                    }
                    else
                    {
                        while (fgets(buffer, sizeof(buffer), output))
                        {
                        }
                        pclose(output);
                        string = buffer;
                        string.erase(std::remove(string.begin(), string.end(), '\n'), string.end());
                        if (!std::filesystem::exists(string))
                        {
                            // Directory doesn't exit... somethings wrong
                            throw std::runtime_error("Directory doesnt exist...");
                        }

                        imageSaveLocation->setValue(string);
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << e.what() << '\n';
                }
            });

        this->imageSaveLocation =
            new nanogui::TextBox(this, "");
        this->imageSaveLocation->setEditable(false);
    };

    nanogui::Button *acquireButton;
    nanogui::Button *acquireSingleButton;
    nanogui::Button *recordingButton;
    nanogui::TextBox *connectedCamera;
    nanogui::TextBox *imageSaveLocation = nullptr;

    const nanogui::Color GREEN = nanogui::Color(50, 255, 50, 100);
    const nanogui::Color RED = nanogui::Color(255, 50, 50, 100);

private:
    nanogui::Button *fileDialogButton = nullptr;
};