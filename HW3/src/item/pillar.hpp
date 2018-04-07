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
    static int initShader();
    static void destroyShader();
protected:
    static unsigned int vao[3];
    static unsigned int vbo[6];

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

    void init() override;
    void render() override;

    Pillar(glm::vec3 const &pos = glm::vec3(0.f),
           float radius = 1.f, float height = 2.f);
    ~Pillar() override;

private:
    int _n_indices;
};

#endif
