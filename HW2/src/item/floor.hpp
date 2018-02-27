#ifndef PX_CG_ITEM_FLOOR_HPP
#define PX_CG_ITEM_FLOOR_HPP

#include "item.hpp"

namespace px { namespace item
{
class Floor;
} }

class px::item::Floor : public Item
{
private:
    static ItemInfo ITEM_INFO;

public:
    static unsigned int texture[4];
    static void initShader();
    static void destroyShader();

public:
    static std::shared_ptr<Item> create();
    static std::size_t regItem();

    bool preRender() const override { return true; }

    void scale(glm::vec3 const &s) override { _scale = s; _scale.y = 1.f; _half_size = scal();}
    void zoom(glm::vec3 const &s) override { _scale *= s; _scale.y = 1.f; _half_size = scal();}

    void setGrid(float u, float v) {_u = u; _v = v; _need_upload = true;}

    void init() override;
    void render(Shader &scene_shader) override;

    Floor(glm::vec3 const &pos = glm::vec3(0.f),
          float width = 1.f, float height = 1.f,
          float u = 1.f, float v = 1.f);
    ~Floor() override;

protected:
    unsigned int vao, vbo;
private:
    float _u, _v;
    bool _need_upload;
};

#endif
