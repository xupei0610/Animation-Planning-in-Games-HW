#include "app.hpp"

#include "scene/string_scene.hpp"

#include <cstring>

using namespace px;

const int App::WIN_HEIGHT = 600;
const int App::WIN_WIDTH  = 800;
const char *App::WIN_TITLE = "OpenGL Particle System Demo - by Pei Xu";

App* App::instance()
{
    static App * instance = nullptr;
    if (instance == nullptr) instance = new App;
    return instance;
}

App::App()
        : opt(), scene(&opt), menu(),
          _window(nullptr),
          scenes{
              new scene::StringScene
          },

          text_shader(nullptr),
          _height(WIN_HEIGHT), _width(WIN_WIDTH), _title(WIN_TITLE)
{}

App::~App()
{
    glfwDestroyWindow(window());

    for (auto &s : scenes)
        delete s;
}

[[noreturn]]
void App::err(std::string const &msg)
{
    throw AppError("App Error: " + msg);
}

void App::init(bool fullscreen)
{
    if (window()) glfwDestroyWindow(window());

    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
#ifndef DISABLE_MULTISAMPLE
    glfwWindowHint(GLFW_SAMPLES, 4);
#endif

    // init window
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    _window = glfwCreateWindow(1, 1, "", nullptr, nullptr);
    _full_screen = !fullscreen;
    if (!window()) err("Failed to initialize window.");

    // init OpenGL
    glfwMakeContextCurrent(window());
//    std::cout << "OpenGL: " << glGetString(GL_VERSION) << std::endl;
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) err("Failed to initialize GLEW.");
    glEnable(GL_DEPTH_TEST);
#ifndef DISABLE_MULTISAMPLE
    glEnable(GL_MULTISAMPLE);
#endif

    glfwSetWindowSizeCallback(window(), &App::windowSizeCallback);
    glfwSetFramebufferSizeCallback(window(), &App::frameBufferSizeCallback);

    // load game resource
    if (text_shader == nullptr)
    {
        text_shader = new TextShader;
        text_shader->setFontHeight(static_cast<std::size_t>(40));
        static const unsigned char TITLE_FONT_DATA[] = {
#include "font/Just_My_Type.dat"
        };
        text_shader->addFont(TITLE_FONT_DATA, sizeof(TITLE_FONT_DATA));
    }
    scene.init();
    for (auto s : scenes)
        s->init(scene);
    scene.load(scenes[0], false);
    menu.init();

    // show window
    state = State::Running;
    togglePause();
    glfwShowWindow(window());
    toggleFullscreen();

    restart();
}

bool App::run()
{
    processEvents();
    if (glfwWindowShouldClose(window()))
        return false;

    switch (state)
    {
        case State::Running:
            updateTimeGap();
            if (scene.run(timeGap()))
            {
                scene.render();
                gui();
                break;
            }

        default:
            state = State::Pausing;
            glfwSetInputMode(window(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            menu.render(*this);
    }

    glfwSwapBuffers(window());

    return true;
}

void App::restart()
{
    // setup callback fns
    glfwSetKeyCallback(window(), &App::keyCallback);
    glfwSetMouseButtonCallback(window(), &App::mouseCallback);
    glfwSetCursorPosCallback(window(), &App::cursorPosCallback);
    glfwSetScrollCallback(window(), &App::scrollCallback);

    std::memset(action, 0, sizeof(action));
    std::memset(detection, 1, sizeof(detection));
    scene.restart();
    if (state == State::Pausing)
        togglePause();
}

void App::close()
{
    glfwSetWindowShouldClose(window(), 1);
}

void App::gui()
{
    auto x = frameWidth() - 10;
    constexpr auto y = 10.f;
    auto gap = 20;

    text_shader->render("FPS: " + std::to_string(_fps),
                             x, y, 0.4f,
                             glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                             Anchor::RightTop);
    text_shader->render(scene.character.canFloat() ? "Fly: on" : "Fly: off",
                             x, y+gap, 0.4f,
                             glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                             Anchor::RightTop);
}

void App::togglePause()
{
    if (state == State::Pausing)
    {
        state = State::Running;
//        glfwSetInputMode(window(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        mouse_detected = false;
        time_gap = -1;
    }
    else
    {
        state = State::Pausing;
        glfwSetInputMode(window(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
}

void App::toggleFullscreen()
{
    auto m = glfwGetWindowMonitor(window());
    if (m == nullptr) m = glfwGetPrimaryMonitor();
    auto v = glfwGetVideoMode(m);

    if (_full_screen)
    {
        glfwSetWindowMonitor(window(), nullptr,
                             (v->width - _width)/2, (v->height - _height)/2,
                             _width, _height, GL_DONT_CARE);
        glfwSetWindowSize(window(), _width, _height);
    }
    else
    {
        glfwSetWindowMonitor(window(), m, 0, 0, v->width, v->height, GL_DONT_CARE);
    }

    _full_screen = !_full_screen;
    updateWindowSize();
    updateFrameBufferSize();
}

void App::processEvents()
{
    glfwPollEvents();

    if (state == State::Pausing)
        return;

    if (action[static_cast<int>(Action::Run)])
        scene.character.activate(Action::Run, true);
    else
        scene.character.activate(Action::Run, false);

    if (action[static_cast<int>(Action::MoveForward)])
        scene.character.activate(Action::MoveForward, true);
    if (action[static_cast<int>(Action::MoveBackward)])
        scene.character.activate(Action::MoveBackward, true);

    if (action[static_cast<int>(Action::MoveLeft)])
        scene.character.activate(Action::MoveLeft, true);
    if (action[static_cast<int>(Action::MoveRight)])
        scene.character.activate(Action::MoveRight, true);

    if (action[static_cast<int>(Action::TurnLeft)])
        scene.character.activate(Action::TurnLeft, true);
    if (action[static_cast<int>(Action::TurnRight)])
        scene.character.activate(Action::TurnRight, true);

    if (action[static_cast<int>(Action::Jump)])
        scene.character.activate(Action::Jump, true);

    if (action[static_cast<int>(Action::Shoot)])
        scene.character.activate(Action::Shoot, true);
}

void App::scroll(float, float y_offset)
{
    scene.cam.zoom(y_offset);
    scene.cam.updateProj();
}

void App::cursor(float x_pos, float y_pos)
{
    if (state == State::Pausing)
    {
        menu.cursor(*this,
                    x_pos * frameWidth()  / width(),
                    y_pos * frameHeight() / height());
        return;
    }

    if (mouse_detected)
    {
        if (detection[static_cast<unsigned int>(Detection::HorizontalMouseMotion)])
        {
            auto x_offset = opt.mouseSensitivity() * (x_pos - _center_x);
            scene.cam.yaw(x_offset);
        }
        if (detection[static_cast<unsigned int>(Detection::VerticalMouseMotion)])
        {
            auto y_offset = opt.mouseSensitivity() * (y_pos - _center_y);
            scene.cam.pitch(opt.invertY() ? y_offset : -y_offset);
        }
    }
    else
    {
        mouse_detected = true;
    }
    glfwSetCursorPos(window(), _center_x, _center_y);
}

void App::click(int button, int mods, int action)
{
    static auto button_state = GLFW_RELEASE;

    if (state == State::Pausing)
        menu.click(*this, button, button_state, action);
    else
    {
        input(button, mods, action, true);
    }

    button_state = action;
}

void App::input(int keycode, int mods, int action, bool mouse)
{
    // TODO modifier check for input

    auto enable = action != GLFW_RELEASE;

#define KEY_CALLBACK(Action)                                \
    if (keycode == opt.shortcuts[Action])                   \
        this->action[static_cast<int>(Action)] = enable;
#define LOAD_SCENE(Idx)                                         \
    if (scenes.size() > Idx && scenes[Idx] != scene.scene()) { scene.load(scenes[Idx], false); restart(); }

    if (keycode == opt.shortcuts[System::Pause] && action == GLFW_PRESS)
    {
        if (menu.page != Menu::Page::Pause)
            menu.page = Menu::Page::Pause;
        else
            togglePause();
    }
    else if (keycode == GLFW_KEY_1 && action == GLFW_PRESS)
    {
        LOAD_SCENE(0);
    }
    else if (keycode == GLFW_KEY_2 && action == GLFW_PRESS)
    {
        LOAD_SCENE(1);
    }
    else if (keycode == GLFW_KEY_3 && action == GLFW_PRESS)
    {
        LOAD_SCENE(2);
    }
    else if (keycode == GLFW_KEY_4 && action == GLFW_PRESS)
    {
        LOAD_SCENE(3);
    }
    else if (keycode == GLFW_KEY_R && action == GLFW_PRESS)
    {
        restart();
    }
    else if (keycode == GLFW_KEY_F && action == GLFW_PRESS)
    {
        scene.character.setFloating(!scene.character.canFloat());
    }
    else KEY_CALLBACK(Action::MoveForward)
    else KEY_CALLBACK(Action::MoveBackward)
    else KEY_CALLBACK(Action::MoveLeft)
    else KEY_CALLBACK(Action::MoveRight)
    else KEY_CALLBACK(Action::TurnLeft)
    else KEY_CALLBACK(Action::TurnRight)
    else KEY_CALLBACK(Action::Jump)
    else KEY_CALLBACK(Action::Run)
    else KEY_CALLBACK(Action::Shoot)

#undef KEY_CALLBACK
}

void App::setSize(int width, int height)
{
    if (height < 1 || width < 1)
        err("Unable to set window size as a non-positive value.");

    _height = height;
    _width = width;

    if (!_full_screen)
    {
        glfwSetWindowSize(window(), _width, _height);
        updateWindowSize();
        updateFrameBufferSize();
    }
}

void App::setTitle(std::string const &title)
{
    _title = title;
    if (window())
        glfwSetWindowTitle(window(), title.data());
}

void App::updateWindowSize()
{
    if (window())
    {
        glfwGetWindowSize(window(), &_width, &_height);
        _center_x = _width * 0.5f;
        _center_y = _height * 0.5f;
    }
}

void App::updateFrameBufferSize()
{
    if (window())
    {
        int w, h;
        glfwGetFramebufferSize(window(), &w, &h);
        scene.resize(w, h);
        menu.resize(scene.cam.width(), scene.cam.height());
        _frame_height = scene.cam.height();
        _frame_width = scene.cam.width();
    }
}

void App::updateTimeGap()
{
    static decltype(glfwGetTime()) last_count_time;
    static int frames;

    if (time_gap == -1)
    {
        current_time = glfwGetTime();
        time_gap = 0;
        _fps = 0;
        frames = 0;
        last_count_time = current_time;
    }
    else
    {
        auto c_time = glfwGetTime();
        time_gap = c_time - current_time;
        current_time = c_time;

        if (current_time - last_count_time >= 1.0)
        {
            _fps = frames;
            frames = 0;
            last_count_time = current_time;
        }
        else
            frames += 1;
    }
}

void App::windowSizeCallback(GLFWwindow *, int width, int height)
{
    instance()->updateWindowSize();
}
void App::frameBufferSizeCallback(GLFWwindow *, int width, int height)
{
    instance()->updateFrameBufferSize();
}

void App::keyCallback(GLFWwindow* window, int key, int, int action, int mods)
{
    instance()->input(key, mods, action, false);
}

void App::scrollCallback(GLFWwindow *, double x_offset, double y_offset)
{
    instance()->scroll(float(x_offset), float(y_offset));
}

void App::cursorPosCallback(GLFWwindow *, double x_pos, double y_pos)
{
    instance()->cursor(float(x_pos), float(y_pos));
}

void App::mouseCallback(GLFWwindow *, int button, int action, int mods)
{
    instance()->click(button, mods, action);
}

void App::text(std::string const &text, float x, float y, float scale,
               glm::vec4 const &color, Anchor anchor)
{
    text_shader->render(text, x, y, scale, color, anchor);
}

void App::activate(Detection d, bool enable)
{
    auto id = static_cast<unsigned int>(d);
    if (id < N_DETECTIONS)
        detection[id] = enable;
}
