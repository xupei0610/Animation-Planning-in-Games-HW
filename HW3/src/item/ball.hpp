#ifndef PX_CG_ITEM_BALL_HPP
#define PX_CG_ITEM_BALL_HPP

#include "item.hpp"

namespace px { namespace item
{
class Ball;
} }

class px::item::Ball : public Item
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

    void init() override;
    void render() override;

    Ball(glm::vec3 const &pos = glm::vec3(0.f),
         float radius = 1.f);
    ~Ball() override;

protected:
    unsigned int vao[1], vbo[2];
private:
    int _n_indices;
};

#endif
