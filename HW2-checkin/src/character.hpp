#ifndef PX_CG_CHARACETER_HPP
#define PX_CG_CHARACETER_HPP

#include <unordered_map>

#include "option.hpp"
#include "camera.hpp"

namespace px
{
class Scene;
class Character;
}

class px::Character
{
public:
    // character attributes
    static const float CHARACTER_HEIGHT;
    static const float CHARACTER_HALF_SIZE;
    static const float JUMP_HEIGHT;
    static const float ASC_SP;
    static const float DROP_SP;
    static const float FORWARD_SP;
    static const float BACKWARD_SP;
    static const float SIDESTEP_SP;
    static const float TURN_SP;
    static const float FORWARD_RUN_COEF;
    static const float BACKWARD_RUN_COEF;
    static const float SIDESTEP_RUN_COEF;
    static const float TURN_RUN_COEF;
    static const float SHOOT_GAP;

    // character super/transient power
    static const bool CAN_FLOATING;
    static const bool CAN_SHOOT;

    // keepable ability
    static const int MAX_LOAD;
    static const int MAX_SLOTS;

    Camera *cam;
    Scene *scene;
    std::unordered_map<std::size_t, int> items;

public:
    Character(Camera *cam, Scene *scene);

    void reset();
    void reset(float x, float y, float z,
               float yaw = -90.0f, float pitch = 0.0f);
    void resetCharacterAttributes();
    void clearBag();
    void resetKeepableAbility();

    void shoot();
    void activate(Action a, bool enable);
    glm::vec3 makeAction(float dt);

    inline const bool &isJumping() const noexcept { return current_action[static_cast<int>(Action::Jump)]; }
    inline const bool &isMovingForward() const noexcept { return current_action[static_cast<int>(Action::MoveForward)]; }
    inline const bool &isMovingBackward() const noexcept { return current_action[static_cast<int>(Action::MoveBackward)]; }
    inline const bool &isMovingLeft() const noexcept { return current_action[static_cast<int>(Action::MoveLeft)]; }
    inline const bool &isMovingRight() const noexcept { return current_action[static_cast<int>(Action::MoveRight)]; }
    inline const bool &isTurningLeft() const noexcept { return current_action[static_cast<int>(Action::TurnLeft)]; }
    inline const bool &isTurningRight() const noexcept { return current_action[static_cast<int>(Action::TurnRight)]; }
    inline const bool &isRunning() const noexcept {return current_action[static_cast<int>(Action::Run)];}
    inline const bool &isShooting() const noexcept {return current_action[static_cast<int>(Action::Shoot)];}

    inline const float &characterHp() const noexcept { return character_hp; }
    inline const float &characterMaxHp() const noexcept { return character_max_hp; }

    inline const float &characterHeight() const noexcept {return character_height; }
    inline const float &characterHalfHeight() const noexcept {return character_half_height; }
    inline const float &characterHalfSize() const noexcept {return character_half_size; }
    inline const float &jumpHeight() const noexcept {return jump_height; }
    inline const float &ascSpeed() const noexcept {return asc_speed; }
    inline const float &dropSpeed() const noexcept {return drop_speed; }
    inline float forwardSpeed() const noexcept
    {
        return isRunning() ? forward_run_coef * forward_speed : forward_speed;
    }
    inline float backwardSpeed() const noexcept
    {
        return isRunning() ? backward_run_coef * backward_speed : backward_speed;
    }
    inline float sidestepSpeed() const noexcept
    {
        return isRunning() ? sidestep_run_coef * sidestep_speed : sidestep_speed;
    }
    inline float turnSpeed() const noexcept
    {
        return isRunning() ? turn_run_coef * turn_speed : turn_speed;
    }
    inline const float &forwardRunCoef() const noexcept {return forward_run_coef;}
    inline const float &backwardRunCoef() const noexcept {return backward_run_coef;}
    inline const float &sidestepRunCoef() const noexcept {return sidestep_run_coef;}
    inline const float &turnRunCoef() const noexcept {return turn_run_coef;}
    inline const float &shootGap() const noexcept {return shoot_gap;}

    inline const bool &isAscending() const noexcept {return is_ascending;}

    inline const bool &canFloat() const noexcept {return can_float;}
    inline const bool &canShoot() const noexcept {return can_shoot;}

    int hasItem(std::size_t const &item_id) const;
    bool collectItem(std::size_t const &item_id, int n);
    bool canCarry(std::size_t const &item_id, int n) const noexcept;

    void setCharacterHp(float hp);
    void setCharacterMaxHp(float hp);
    void setCharacterHeight(float h);
    void setCharacterHalfSize(float s);
    void setAscSpeed(float s);
    void setDropSpeed(float s);
    void setForwardSpeed(float sp);
    void setBackwardSpeed(float sp);
    void setSidestepSpeed(float sp);
    void setTurnSpeed(float degree_sp);
    void setFloating(bool enable);
    void setShootable(bool enable);
    void setHeadLight(bool enable);

    void enableDropping();
    void disableDropping();

    ~Character() = default;
    Character &operator=(Character const &) = delete;
    Character &operator=(Character &&) = delete;

protected:
    int n_items;
    int max_slots;
    int current_load;
    int max_load;

    bool current_action[N_ACTIONS];
    bool is_ascending; // helper parameter for jump action;
    float last_shoot;
    glm::vec3 move_dir;

    void makeJump(float dt);
    void charTurnLeft(float dt);
    void charTurnRight(float dt);

    void updateMoveDir();
    void updateMoveDirRight();
    void updateMoveDirLeft();
    void updateMoveDirForward();
    void updateMoveDirBackward();

    bool canAscend();
    bool isDropping();
    void makeDrop(float dt);

    void characterYPosFix();

    float character_hp;
    float character_max_hp;
    float character_height;
    float character_half_height;
    float character_half_size;
    float jump_height;
    float asc_speed;
    float drop_speed;
    float forward_speed;
    float backward_speed;
    float sidestep_speed;
    float turn_speed;
    float forward_run_coef;
    float backward_run_coef;
    float sidestep_run_coef;
    float turn_run_coef;
    float shoot_gap;

    bool  can_float;
    bool  can_shoot;

    bool is_dropping;
};
#endif
