#ifndef PX_CG_OPTION_HPP
#define PX_CG_OPTION_HPP

#include <array>
#include <algorithm>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/vec3.hpp>

namespace px {

#define N_ACTIONS 10
enum class Action : unsigned int
{
    MoveForward = 1,
    MoveBackward = 2,
    MoveLeft = 3,
    MoveRight = 4,
    TurnLeft = 5,
    TurnRight = 6,
    Jump = 7,
    Run = 8,
    Shoot = 9,
};

enum class System : unsigned int
{
    Pause = 0
};

static constexpr
    std::array<decltype(GLFW_KEY_0), N_ACTIONS> KEYBOARD_SHORTCUTS = {{
        // system related
        GLFW_KEY_ESCAPE,    // Pause = 0,
        // game control related
        GLFW_KEY_W,         // MoveForward = 1,
        GLFW_KEY_S,         // MoveBackward = 2,
        GLFW_KEY_A,         // MoveLeft = 3,
        GLFW_KEY_D,         // MoveRight = 4,
        GLFW_KEY_Q,   // TurnLeft = 5,
        GLFW_KEY_E,   // TurnRight = 6
        GLFW_KEY_SPACE,     // Jump = 7
        GLFW_KEY_LEFT_SHIFT, // Run = 8, Modifier
        GLFW_MOUSE_BUTTON_LEFT // Shoot = 9
    }};

class Option;
}

class px::Option
{
public:
    static const glm::vec3 RESISTANCE;
    static const glm::vec3 GRAVITY;

    // game options
    static const float MOUSE_SEN;
    static const bool INVERT_Y;

    class Shortcuts
    {
    protected:
        std::array<decltype(GLFW_KEY_0), N_ACTIONS> shortcuts;
    public:
        Shortcuts();
        ~Shortcuts() = default;
        Shortcuts &operator=(Shortcuts const &) = default;
        Shortcuts &operator=(Shortcuts &&) = default;

        decltype(GLFW_KEY_0) operator[](Action a)
        {
            return shortcuts[static_cast<unsigned int>(a)];
        }
        decltype(GLFW_KEY_0) operator[](System a)
        {
            return shortcuts[static_cast<unsigned int>(a)];
        }
        void set(Action a, decltype(GLFW_KEY_0) key);
        void reset();
    } shortcuts;

public:
    Option();

    inline const glm::vec3 &resistance() const noexcept { return _resistance; }
    inline glm::vec3 &resistance() noexcept { return _resistance; }
    inline const glm::vec3 &gravity() const noexcept { return _gravity; }
    inline glm::vec3 &gravity() noexcept { return _gravity; }

    inline const bool &invertY() const noexcept {return _invert_y;}
    inline bool &invertY() noexcept { return _invert_y; }

    inline const float &mouseSensitivity() const noexcept {return _mouse_sensitivity;}
    inline float &mouseSensitivity() noexcept { return _mouse_sensitivity; }

    void resetShortcuts();
    void resetGameParams();
    void resetOpts();

    ~Option() = default;
    Option &operator=(Option const &) = default;
    Option &operator=(Option &&) = default;

protected:
    glm::vec3 _resistance;
    glm::vec3 _gravity;
    float _mouse_sensitivity;
    bool  _invert_y;
};

#endif
