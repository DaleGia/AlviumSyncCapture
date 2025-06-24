#include <nanogui/nanogui.h>

class GUIStackSettings
{
public:
    GUIStackSettings()
    {
        screen =
            new nanogui::Screen(
                Eigen::Vector2i(
                    400,
                    100),
                "Stack Settings");
        window = new nanogui::Window(screen, "");
        window->setFixedHeight(100);
        window->setFixedWidth(400);
        window->setLayout(new nanogui::GroupLayout());
        nanogui::Vector2i position(0, 0);

        window->setPosition(position);
        new nanogui::Label(window, "Stack Period (us):", "sans-bold");

        this->stackPeriod = new nanogui::IntBox<int>(window, 1000000);
        this->stackPeriod->setEditable(true);
        window->setVisible(true);
        screen->setVisible(true);
        screen->performLayout();
    };

    nanogui::IntBox<int> *stackPeriod = nullptr;

private:
    nanogui::Screen *screen;
    nanogui::Window *window;
};
