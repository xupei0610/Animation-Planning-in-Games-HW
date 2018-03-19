#ifndef PX_CG_ITEM_PILLAR_HPP
#define PX_CG_ITEM_PILLAR_HPP

#include "item.hpp"

namespace px { namespace item
{
class Pillar;
} }

class px::item::Pillar : public Item
{
private:
    static ItemInfo ITEM_INFO;

public:
    static Shader *shader;
    static void initShader();
    static void destroyShader();

public:
    struct
    {
        glm::vec3 ambient;
        glm::vec3 diffuse;
        glm::vec3 specular;
        float shininess;
    } color;

    static std::shared_ptr<Item> create();
    static std::size_t regItem();

    bool postRender() const override { return true; }

    void scale(glm::vec3 const &s) override { _scale = s; _half_size = scal();}
    void zoom(glm::vec3 const &s) override { _scale *= s; _half_size = scal();}

    void init() override;
    void render() override;

    Pillar(glm::vec3 const &pos = glm::vec3(0.f),
           float radius = 1.f, float height = 2.f);
    ~Pillar() override;

protected:
    unsigned int vao[3], vbo[6];
private:
    int _n_indices;
};

#endif
