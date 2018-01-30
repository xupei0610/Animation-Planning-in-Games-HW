#ifndef PX_CG_CAMERA_HPP
#define PX_CG_CAMERA_HPP

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#ifdef GLM_FORCE_RADIANS
#define GLM_ANG(ang) ang
#else
#define GLM_ANG(ang) glm::radians(ang)
#endif

namespace px {
class Camera;
}

class px::Camera
{
public:
    static const glm::mat4 IDENTITY_MAT4;

    static const float FOV;
    static const float NEAR_CLIP;
    static const float FAR_CLIP;

public:
    Camera();

    inline const int &width() const noexcept { return _width; }
    inline const int &height() const noexcept { return _height; }
    inline const float &nearClip() const noexcept { return _near_clip; }
    inline const float &farClip() const noexcept { return _near_clip; }
    inline const float &fov() const noexcept { return _fov; }
    inline const float &yaw() const noexcept { return _yaw; }
    inline const float &pitch() const noexcept { return _pitch; }

    inline int &width() noexcept { return _width; }
    inline int &height() noexcept { return _height; }
    inline float &nearClip() noexcept { return _near_clip; }
    inline float &farClip() noexcept { return _near_clip; }
    inline float &fov() noexcept { return _fov; }
    inline float &yaw() noexcept { return _yaw; }
    inline float &pitch() noexcept { return _pitch; }

    inline const glm::vec3 &pos() const noexcept { return _pos; }
    inline glm::vec3 &pos() noexcept { return _pos; }

    void reset(float x, float y, float z, float yaw, float pitch);

    Camera &zoom(float dfov);
    Camera &pitch(float dpitch);
    Camera &yaw(float dyaw);
    Camera &roll(float droll);

    inline glm::vec3 &direction() {return forward;}
    inline glm::vec3 &right() {return strafe;}

    void updateView();
    void updateProj();

    inline const glm::mat4 &viewMat() const { return view; }
    inline const glm::mat4 &projMat() const { return proj; }

protected:
    glm::mat4 proj, view;

    glm::vec3 strafe;   // first col of view mat
    glm::vec3 forward;  // negative of the 3rd col of view mat

private:
    glm::vec3 _pos;
    int _width, _height;
    float _near_clip, _far_clip;
    float _pitch, _yaw, _roll;
    float _fov;
};

#endif
