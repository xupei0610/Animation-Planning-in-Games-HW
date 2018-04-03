#include "camera.hpp"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <iostream>

using namespace px;

const glm::mat4 Camera::IDENTITY_MAT4(1.f);
const float Camera::FOV = 45.f;
const float Camera::NEAR_CLIP = 0.01f;
const float Camera::FAR_CLIP = 100.f;

Camera::Camera()
        : _pos(0, 0, -1), _near_clip(NEAR_CLIP), _far_clip(FAR_CLIP),
          _pitch(0), _yaw(-90.f), _roll(0), _fov(FOV)
{
    updateView();
}

void Camera::reset(float x, float y, float z, float yaw, float pitch)
{
    _pos.x = x;
    _pos.y = y;
    _pos.z = z;
    _yaw = yaw;
    _pitch = pitch;
}

Camera &Camera::zoom(float dfov)
{
    _fov += dfov;
    if (_fov > 89.f)
        _fov = 89.f;
    else if (_fov < 30.f)
        _fov = 30.f;
    return *this;
}

Camera &Camera::pitch(float dpitch)
{
    _pitch += dpitch;
    if (_pitch > 89.0f)
        _pitch = 89.0f;
    else if (_pitch < -89.0f)
        _pitch = -89.0f;
    return *this;
}

Camera &Camera::yaw(float dyaw)
{
    _yaw += dyaw;
    if (_yaw > 360.0f)
        _yaw -= 360.f;
    else if (_yaw < -360.f)
        _yaw += 360.f;
    return *this;
}

Camera &Camera::roll(float droll)
{
     _roll += droll;
    return *this;
}

void Camera::updateView()
{
//    glm::quat orient = glm::quat(glm::vec3(GLM_ANG(_pitch), GLM_ANG(_yaw), 0));

    glm::quat pitch = glm::angleAxis(GLM_ANG(_pitch), glm::vec3(1, 0, 0));
    glm::quat yaw = glm::angleAxis(GLM_ANG(_yaw), glm::vec3(0, 1, 0));
    glm::quat orient = pitch * yaw;

    glm::mat4 rot = glm::mat4_cast(glm::normalize(orient));
    glm::mat4 trans = glm::translate(IDENTITY_MAT4, -_pos);

    view = rot * trans;

    forward.x = -view[0][2];
    forward.y = -view[1][2];
    forward.z = -view[2][2];
    strafe.x = view[0][0];
    strafe.y = view[1][0];
    strafe.z = view[2][0];
}

void Camera::updateProj()
{
    proj = glm::perspectiveFov(GLM_ANG(_fov),
                               static_cast<float>(_width),
                               static_cast<float>(_height),
                               _near_clip, _far_clip);
}
