#ifndef PX_CG_SHADER_TEXT_HPP
#define PX_CG_SHADER_TEXT_HPP

#include "shader/base_shader.hpp"

#include <vector>
#include <array>

namespace px
{
enum class Anchor : int
{
    LeftTop = 11,
    LeftCenter = 12,
    LeftBottom = 13,
    CenterTop = 21,
    Center = 22,
    CenterBottom = 23,
    RightTop = 31,
    RightCenetr = 32,
    RightBottom = 33
};

class TextShader;
}

class px::TextShader : public Shader
{
public:
    static const std::size_t FONT_HEIGHT;

    struct Character
    {
        unsigned int size_x;
        unsigned int size_y;
        int bearing_x;
        int bearing_y;
        long int advance;
    };
    static const char *VS;
    static const char *FS;

    TextShader();
    ~TextShader();

    void render(std::string const &text,
                float x, float y, float scale,
                glm::vec4 const &color,
                Anchor anchor);

    void setFontHeight(std::size_t font_height);
    inline std::size_t const &fontHeight() const noexcept {return _font_height;}

    std::size_t addFont(const unsigned char *data, std::size_t data_size);
    bool activateFont(std::size_t index);

protected:
    std::vector<std::array<unsigned, 95> > textures;
    std::vector<Character> chars;
    unsigned int vao, vbo;

    std::size_t current_font;

private:
    std::size_t _font_height;
};

#endif
