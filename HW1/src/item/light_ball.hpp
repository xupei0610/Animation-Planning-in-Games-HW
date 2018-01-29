#ifndef PX_CG_ITEM_LIGHT_BALL_HPP
#define PX_CG_ITEM_LIGHT_BALL_HPP

#include "item.hpp"

namespace px { namespace item
{
class LightBall;
} }

class px::item::LightBall : public Item, public Rigid
{
private:
    static ItemInfo ITEM_INFO;

public:
    static unsigned int vao, vbo[2];
    static Shader *shader;
    static unsigned int initShader(unsigned int n_grid);
    static void destroyShader();

public:
    static std::shared_ptr<Item> create();
    static std::size_t regItem();

    void scale(glm::vec3 const &s) override { _scale = s; _half_size = scal();}
    void zoom(glm::vec3 const &s) override { _scale *= s; _half_size = scal();}

    bool lighting() const override { return true; }
    bool postRender() const override { return true; }
    bool move() const override { return true; }

    Light const &light() const override { return _light; }
    void enlight(Light const &light) override { _light = light; }

    void update(float dt) override;
    void hit(glm::vec3 const &at, glm::vec3 const &norm) override;
    glm::vec3 const &movement() const override { return _movement; }

    void init() override;
    void render(glm::mat4 const &view, glm::mat4 const &proj) override;

    void setGrid(unsigned int n);
    inline glm::vec3 &color() noexcept { return _color; }
    inline const glm::vec3 &color() const noexcept { return _color; }

    LightBall(glm::vec3 const &position = glm::vec3(0.f, 0.f, 0.f),
              glm::vec3 const &scale = glm::vec3(1.f),
              float mass = 1.f,
              glm::vec3 const &velocity = glm::vec3(0.f, 0.f, 0.f),
              glm::vec3 const &acceleration = glm::vec3(0.f, 0.f, 0.f));

    ~LightBall();

protected:
    inline const unsigned int &n_indices() const noexcept { return _n_indices; }
    inline const unsigned int &grid() const noexcept { return _n_theta; }

private:
    glm::vec3 _movement;
    Light _light;
    unsigned int _n_theta, _n_indices;
    glm::vec3 _color;
};

#endif
