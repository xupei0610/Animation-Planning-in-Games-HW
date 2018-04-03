#ifndef PX_CG_ITEM_HPP
#define PX_CG_ITEM_HPP

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <iostream>

#include "shader/base_shader.hpp"

namespace px
{
class ItemError;
class Item;
struct ItemInfo;
struct Light;
class Rigid;

typedef std::shared_ptr<Item> (*ItemGenFn)();
}


struct px::Light
{
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
    glm::vec3 coef;
};

struct px::ItemInfo
{
private:
    std::size_t _id;

public:
    const std::string name;
    const std::string description;
    const int weight;
    const bool stackable;
    const bool collectible;
    const bool placeable;

    ItemInfo(std::string const &name, std::string const &description,
              int weight, bool stackable, bool collectible, bool placeable);

    inline const std::size_t &id() const noexcept {return _id;}

    friend Item;
};

class px::ItemError : public std::exception
{

public:
    ItemError(const std::string &msg, const int code=0)
            : msg(msg), err_code(code)
    {}
    const char *what() const noexcept override
    {
        return msg.data();
    }
    inline int code() const
    {
        return err_code;
    }

protected:
    std::string msg;
    int err_code;
};

class px::Item
{
private:
    static std::vector<ItemInfo> _items;
    static std::vector<ItemGenFn> _item_gen_fn;
public:
    static std::size_t reg(ItemInfo &item_info, ItemGenFn fn);
    static ItemInfo const &lookup(std::size_t const &id);
    static std::shared_ptr<Item> gen(std::size_t const &index);

public:
    ItemInfo const &attribute;

    [[noreturn]]
    static void err(std::string const &msg);

    virtual void place(glm::vec3 const &pos) { _position = pos; }
    virtual void move(glm::vec3 const &span) { _position += span; }
    virtual glm::vec3 const &pos() const { return _position; }

    virtual void face(glm::vec3 const &rot) { _rotation = rot; }
    virtual void rotate(glm::vec3 const &rot) { _rotation += rot; }
    virtual glm::vec3 const &rot() const { return _rotation; }

    virtual glm::vec3 const &scal() const { return _scale; }
    virtual glm::vec3 const &hsize() const { return _half_size; }
    virtual void scale(glm::vec3 const &scal) { _scale = scal; _half_size = _scale;}
    virtual void zoom(glm::vec3 const &scal) { _scale *= scal; _half_size = _scale;}

    virtual bool preRender() const { return false; }
    virtual bool lighting() const { return false; }
    virtual bool postRender() const { return false; }

    virtual void enlight(const Light *) {}
    virtual Light *light() { return nullptr; }

    virtual void activate(bool enable) { _alive = enable; }
    virtual bool alive() const { return _alive; }
    virtual bool move() const { return false; }

    virtual void update(float dt) {  }
    virtual glm::vec3 *movement() { return nullptr; }
    virtual void hit(glm::vec3 const &at, glm::vec3 const &norm) {}

    virtual void init() {}
    virtual void blend(GLenum sfactor, GLenum dfactor) {};

    // pre-render, rendering with scene, rendering into buffer
    virtual void render(Shader &scene_shader) {}
    // post-render, rendering after lighting rendering, rendering directly for output
    virtual void render() {}

    virtual ~Item() {}
protected:
    bool _alive;
    glm::vec3 _position;
    glm::vec3 _rotation;
    glm::vec3 _scale;
    glm::vec3 _half_size;

protected:
    Item(std::size_t id);
};

class px::Rigid
{
public:
    void mass(float m) { mass() = m; }
    float &mass() { return _mass; }

    void velocity(glm::vec3 const &v) { velocity() = v; }
    glm::vec3 &velocity() { return _velocity; }

    void acceleration(glm::vec3 const &a) { acceleration() = a; }
    glm::vec3 &acceleration() { return _acceleration; }

    void resistance(glm::vec3 const &f) { resistance() = glm::abs(f); }
    glm::vec3 &resistance() { return _resistance; }

    void force(glm::vec3 const &f) { acceleration() += f / mass(); }

    glm::vec3 step(float dt);

protected:
    Rigid(float mass = 1.f,
          glm::vec3 const &velocity = glm::vec3(0.f, 0.f, 0.f),
          glm::vec3 const &acceleration = glm::vec3(0.f, 0.f, 0.f));
    virtual ~Rigid() = default;
private:
    float _mass;
    glm::vec3 _velocity;
    glm::vec3 _acceleration;
    glm::vec3 _resistance;
};

namespace std
{
template<>
struct hash<px::Item>
{
    std::size_t operator()(const px::Item &item) const
    {
        return item.attribute.id();
    }
};
}

#endif
