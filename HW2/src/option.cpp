#include "option.hpp"

using namespace px;

const glm::vec3 Option::RESISTANCE(1.f, 1.f, 1.f);
const glm::vec3 Option::GRAVITY(0.f, -5.f, 0.f);
const bool Option::INVERT_Y = true;
const float Option::MOUSE_SEN = 0.05f;

Option::Option()
    : _resistance(RESISTANCE), _gravity(GRAVITY),
      _mouse_sensitivity(MOUSE_SEN), _invert_y(INVERT_Y)
{}

void Option::resetShortcuts()
{
    shortcuts.reset();
}

void Option::resetGameParams()
{
    _resistance = RESISTANCE;
    _gravity = GRAVITY;
}

void Option::resetOpts()
{
    _invert_y = INVERT_Y;
    _mouse_sensitivity = MOUSE_SEN;
}

Option::Shortcuts::Shortcuts()
    : shortcuts(KEYBOARD_SHORTCUTS)
{}

void Option::Shortcuts::reset()
{
    shortcuts = KEYBOARD_SHORTCUTS;
}

void Option::Shortcuts::set(Action a, decltype(GLFW_KEY_0) key)
{
    auto index = static_cast<unsigned int>(a);
    auto used = std::find(shortcuts.begin(), shortcuts.end(), key);
    if (used != shortcuts.end())
        shortcuts[used - shortcuts.begin()] = GLFW_KEY_UNKNOWN;
    shortcuts[index] = key;
}