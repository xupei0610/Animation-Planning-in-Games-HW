#ifndef PX_CG_MENU_HPP
#define PX_CG_MENU_HPP

#include "option.hpp"
#include "shader/base_shader.hpp"
#include "shader/rectangle.hpp"
#include "shader/text.hpp"

namespace px
{
class App;
class Menu;
}

class px::Menu
{
public:
    static const int FONT_SIZE;
public:
    enum class Page
    {
        Option,
        Pause
    } page;

    Menu();

    void init(int font_size = FONT_SIZE);
    void render(App &app);
    void resize(int w, int h);
    inline const int &fontSize() const noexcept { return _font_size; }

    [[noreturn]]
    void err(std::string const & msg);

    void cursor(App &app, float cursor_x, float cursor_y);
    void click(App &app, int button, int button_state, int action);
    void renderScene(App &app);

    ~Menu();

    RectangleShader *rectangle_shader;
    TextShader *text_shader;

protected:
    std::size_t font;

    bool on[4];
    int button[4];
    unsigned int fbo, bto, rbo;

private:
    int _font_size;
};

#endif
