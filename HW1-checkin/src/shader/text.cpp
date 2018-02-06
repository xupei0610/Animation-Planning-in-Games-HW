#include "shader/text.hpp"

#include "glfw.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

using namespace px;

const char *TextShader::VS =
#include "shader/glsl/text_shader.vs"
;

const char *TextShader::FS =
#include "shader/glsl/text_shader.fs"
;

const std::size_t TextShader::FONT_HEIGHT = 40;

TextShader::TextShader()
    : Shader(VS, FS),
      vao(0), vbo(0),
      current_font(0),
      _font_height(FONT_HEIGHT)
{
    glBindFragDataLocation(pid(), 0, "color");

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 24*sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4*sizeof(float), 0);
    glEnableVertexAttribArray(0);
}

#include <iostream>
std::size_t TextShader::addFont(const unsigned char *data, std::size_t data_size)
{
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    FT_Face face;
    FT_Library ft;
    FT_Init_FreeType(&ft);
    FT_New_Memory_Face(ft, reinterpret_cast<const FT_Byte*>(data), data_size, 0, &face);

    textures.push_back(std::array<unsigned int, 95>());
    glGenTextures(95, textures.back().data());

    FT_Set_Pixel_Sizes(face, 0,  _font_height);
    for (auto i = 0; i < 95; ++i)
    {
        auto e =
                FT_Load_Char(face, i+32, FT_LOAD_RENDER);
        if (e != 0) std::cout << "Err: " << e << std::endl;
        glBindTexture(GL_TEXTURE_2D, textures.back()[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED,
                     face->glyph->bitmap.width, face->glyph->bitmap.rows,
                     0, GL_RED, GL_UNSIGNED_BYTE,
                     face->glyph->bitmap.buffer);

        chars.push_back({face->glyph->bitmap.width, face->glyph->bitmap.rows,
                         face->glyph->bitmap_left, face->glyph->bitmap_top,
                         face->glyph->advance.x >> 6});
    }

    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    return textures.size() - 1;
}

bool TextShader::activateFont(std::size_t index)
{
    if (index < textures.size())
    {
        current_font = index;
        return true;
    }
    return false;
}

TextShader::~TextShader()
{
    for (auto &t : textures)
        glDeleteTextures(128, t.data());
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
}

void TextShader::setFontHeight(std::size_t font_height)
{
    _font_height = font_height;
}

void TextShader::render(std::string const &text,
                        float x, float y, float scale,
                        glm::vec4 const &color,
                        Anchor anchor)
{
    if (textures.empty())
        return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    int w, h;
    glfwGetFramebufferSize(glfwGetCurrentContext(), &w, &h);

    float xoff, yoff = h - y;
    auto width = 0.0f, height=0.0f;
    for (const auto &c: text)
    {
        auto i = c - 32;
        if (i < 0 || i > 94)
            continue;
        width += chars[i].advance * scale;
        if (chars[i].size_y > height)
        {
            height = chars[i].size_y;
        }
    }

    switch(static_cast<int>(anchor)/10)
    {
        case 2:
            xoff = -width/2.0f;
            break;
        case 3:
            xoff = -width;
            break;
        default:
            xoff = 0;
    }
    switch (static_cast<int>(anchor) - static_cast<int>(anchor)/10*10)
    {
        case 1:
            yoff -= height * scale;
            break;
        case 2:
            yoff -= height * scale /2.0f;
            break;
        default:
            break;
    }

    use();

    set("text_color", color);
    set("proj", glm::ortho(0.0f, static_cast<float>(w),
                           0.0f, static_cast<float>(h)));

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(vao);
    for (const auto &c: text)
    {
        auto i = c - 32;
        if (i < 0 || i > 94)
            continue;
        auto xpos = xoff + x + chars[i].bearing_x * scale;
        auto ypos = yoff - (chars[i].size_y - chars[i].bearing_y) * scale;
        auto w = chars[i].size_x * scale;
        auto h = chars[i].size_y * scale;
        float vertices[] = {
                xpos,   ypos+h, 0.0f, 0.0f,
                xpos,   ypos,   0.0f, 1.0f,
                xpos+w, ypos,   1.0f, 1.0f,
                xpos,   ypos+h, 0.0f, 0.0f,
                xpos+w, ypos,   1.0f, 1.0f,
                xpos+w, ypos+h, 1.0f, 0.0f
        };
        glBindTexture(GL_TEXTURE_2D, textures[current_font][i]);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

        glDrawArrays(GL_TRIANGLES, 0, 6);

        x += chars[i].advance * scale;
    }
}
