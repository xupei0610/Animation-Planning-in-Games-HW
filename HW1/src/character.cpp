#include "character.hpp"
#include "item.hpp"
#include "scene.hpp"
#include "item/light_ball.hpp"
#include "util/random.hpp"

#include <cstring>

using namespace px;

const float Character::CHARACTER_HEIGHT = 0.5f;
const float Character::CHARACTER_HALF_SIZE = 0.2f;
const float Character::JUMP_HEIGHT = 2.5f*Character::CHARACTER_HEIGHT;
const float Character::ASC_SP = 1.25f * Character::JUMP_HEIGHT;
const float Character::DROP_SP = 2.5f * Character::JUMP_HEIGHT;
const float Character::FORWARD_SP = 4.0f;
const float Character::BACKWARD_SP = 1.0f;
const float Character::SIDESTEP_SP = 2.0f;
const float Character::TURN_SP = 60.0f;
const float Character::FORWARD_RUN_COEF = 1.5f;
const float Character::BACKWARD_RUN_COEF = 1.2f;
const float Character::SIDESTEP_RUN_COEF = 1.5f;
const float Character::TURN_RUN_COEF = 1.5f;
const float Character::SHOOT_GAP = 0.5;

const bool Character::CAN_FLOATING = false;
const bool Character::CAN_SHOOT = true;

const int Character::MAX_LOAD = std::numeric_limits<int>::max();
const int Character::MAX_SLOTS = std::numeric_limits<int>::max();

Character::Character(Camera *cam, Scene *scene)
    :   cam(cam), scene(scene),

        n_items(0), max_slots(MAX_SLOTS), current_load(0), max_load(MAX_LOAD),

        character_height(CHARACTER_HEIGHT), character_half_height(CHARACTER_HEIGHT*0.5f),
        character_half_size(CHARACTER_HALF_SIZE),
        jump_height(JUMP_HEIGHT), asc_speed(ASC_SP), drop_speed(DROP_SP),
        forward_speed(FORWARD_SP), backward_speed(BACKWARD_SP),
        sidestep_speed(SIDESTEP_SP), turn_speed(TURN_SP),
        forward_run_coef(FORWARD_RUN_COEF), backward_run_coef(BACKWARD_RUN_COEF),
        sidestep_run_coef(SIDESTEP_RUN_COEF), turn_run_coef(TURN_RUN_COEF),
        shoot_gap(SHOOT_GAP),

        can_float(CAN_FLOATING), can_shoot(CAN_SHOOT)
{}

void Character::setCharacterHp(float hp)
{
    character_hp = hp;
}

void Character::setCharacterMaxHp(float hp)
{
    character_max_hp = hp;
}

void Character::setCharacterHeight(float h)
{
    character_height = h;
    character_half_height = h * 0.5f;
}

void Character::setCharacterHalfSize(float s)
{
    character_half_size = s;
}

void Character::setAscSpeed(float s)
{
    asc_speed = s;
}

void Character::setDropSpeed(float s)
{
    drop_speed = s;
}

void Character::setForwardSpeed(float sp)
{
    forward_speed = sp;
}

void Character::setBackwardSpeed(float sp)
{
    backward_speed = sp;
}

void Character::setSidestepSpeed(float sp)
{
    sidestep_speed = sp;
}

void Character::setTurnSpeed(float degree_sp)
{
    turn_speed = degree_sp;
}

void Character::setFloating(bool enable)
{
    can_float = enable;
}

void Character::setShootable(bool enable)
{
    can_shoot = enable;
}

void Character::reset()
{
    cam->updateView();
    is_ascending = false;
    is_dropping = false;
    last_shoot = 10000.f;
    std::memset(current_action, 0, sizeof(bool)*N_ACTIONS);

    resetCharacterAttributes();
}

void Character::reset(float x, float y, float z,
                      float yaw, float pitch)
{
    cam->reset(x, y, z, yaw, pitch);
    reset();
}

void Character::resetCharacterAttributes()
{
    character_height = CHARACTER_HEIGHT;
    jump_height = JUMP_HEIGHT;
    asc_speed = ASC_SP;
    drop_speed = DROP_SP;
    forward_speed = FORWARD_SP;
    backward_speed = BACKWARD_SP;
    sidestep_speed = SIDESTEP_SP;
    turn_speed = TURN_SP;
    can_float = CAN_FLOATING;
    can_shoot = CAN_SHOOT;
}

void Character::clearBag()
{
    n_items = 0;
    current_load = 0;
    items.clear();
}

void Character::resetKeepableAbility()
{
    max_load = MAX_LOAD;
    max_slots = MAX_SLOTS;
}

void Character::shoot()
{

    static Light light{glm::vec3(.5f), glm::vec3(.5f), glm::vec3(.5f), glm::vec3(1.f, 1.f, 2.f)};

    auto obj = new item::LightBall(cam->pos() + 0.1f * cam->direction(),
                                     glm::vec3(0.025f), rnd2(),
                                     cam->direction() * (5.f + rnd_np()));
    obj->resistance(scene->opt->resistance());
    obj->acceleration(scene->opt->gravity());
    obj->color().r = rnd();
    obj->color().g = rnd();
    obj->color().b = rnd();

    light.diffuse.r = obj->color().r;
    light.diffuse.g = obj->color().g;
    light.diffuse.b = obj->color().b;
    light.ambient = light.diffuse * .05f;
    light.specular = light.diffuse * .5f;
    obj->enlight(&light);

    scene->add(obj);

    last_shoot = 0.f;
    activate(Action::Shoot, false);
}

void Character::activate(Action a, bool enable)
{
    if (isDropping() || isAscending())
    {
        std::cout << "ok1 " << std::boolalpha << isDropping() << " " << isAscending() <<  std::endl;
        if (a == Action::MoveForward || a == Action::MoveBackward ||
            a == Action::MoveLeft || a == Action::MoveRight)
            return;
    }
    else if (a == Action::Jump)
    {
        is_ascending = !canFloat() && !is_dropping && enable;
    }

    current_action[static_cast<int>(a)] = enable;
}

glm::vec3 Character::makeAction(float dt)
{
//    if (isDropping())
//        makeDrop(dt);
//    else
    if (isJumping())
        makeJump(dt);

    if (!isDropping() && !isAscending())
        updateMoveDir();

    if (isTurningLeft() ^ isTurningRight())
    {
        charTurnLeft(dt);
        charTurnRight(dt);
    }

    if (last_shoot < shootGap())
    {
        last_shoot += dt;
        activate(Action::Shoot, false);
    }
    else if (isShooting() && canShoot())
        shoot();

    return dt*move_dir;
}


void Character::updateMoveDir()
{
    move_dir.x = 0;
    move_dir.y = 0;
    move_dir.z = 0;

    if (isMovingForward() ^ isMovingBackward())
    {
        updateMoveDirForward();
        updateMoveDirBackward();
    }
    if (isMovingLeft() ^ isMovingRight())
    {
        updateMoveDirLeft();
        updateMoveDirRight();
    }

    activate(Action::MoveForward, false);
    activate(Action::MoveBackward, false);
    activate(Action::MoveRight, false);
    activate(Action::MoveLeft, false);

}

void Character::makeJump(float dt)
{
    if (canFloat())
    {
        cam->pos().y += dt*ascSpeed();
        activate(Action::Jump, false);
        return;
    }

    if (canAscend())
        cam->pos().y += dt*ascSpeed();

    characterYPosFix();
}

void Character::updateMoveDirForward()
{
    if (isMovingForward() &&!isMovingBackward())
    {
        if (canFloat())
            move_dir += forwardSpeed() * cam->direction();
        else
        {
            auto len = std::sqrt(cam->direction().x * cam->direction().x +
                                 cam->direction().z * cam->direction().z);
            move_dir.x += forwardSpeed() * cam->direction().x / len;
            move_dir.z += forwardSpeed() * cam->direction().z / len;
        }
    }
}

void Character::updateMoveDirBackward()
{
    if (isMovingBackward() && !isMovingForward())
    {
        if (canFloat())
            move_dir -= backwardSpeed() * cam->direction();
        else
        {
            auto len = std::sqrt(cam->direction().x * cam->direction().x +
                                  cam->direction().z * cam->direction().z);
            move_dir.x -= backwardSpeed() * cam->direction().x / len;
            move_dir.z -= backwardSpeed() * cam->direction().z / len;
        }
    }
}

void Character::updateMoveDirRight()
{
    if (isMovingRight() && !isMovingLeft())
    {
        if (canFloat())
            move_dir += sidestepSpeed() * cam->right();
        else
        {
            auto len = std::sqrt(cam->right().x * cam->right().x +
                                 cam->right().z * cam->right().z);
            move_dir.x += sidestepSpeed() * cam->right().x / len;
            move_dir.z += sidestepSpeed() * cam->right().z / len;
        }
    }
}

void Character::updateMoveDirLeft()
{
    if (isMovingLeft() && !isMovingRight())
    {
        if (canFloat())
            move_dir -= sidestepSpeed() * cam->right();
        else
        {
            auto len = std::sqrt(cam->right().x * cam->right().x +
                                 cam->right().z * cam->right().z);
            move_dir.x -= sidestepSpeed() * cam->right().x / len;
            move_dir.z -= sidestepSpeed() * cam->right().z / len;
        }
    }
}

void Character::charTurnRight(float dt)
{
    if (isTurningRight() && !isTurningLeft())
        cam->yaw(dt * turnSpeed());
    activate(Action::TurnRight, false);
}

void Character::charTurnLeft(float dt)
{
    if (isTurningLeft() && !isTurningRight())
        cam->yaw(-dt * turnSpeed());
    activate(Action::TurnLeft, false);
}

void Character::characterYPosFix()
{
    if (is_ascending && !canAscend()) // will not ascend any more
    {
        cam->pos().y = jumpHeight();
        is_ascending = false;
    }
//    else if (cam->eye.y < characterHeight()) // will not drop any more
//    {
//        cam->eye.y = characterHeight();
//        activate(Action::Jump, false);
//        move_dir.x = 0;
//        move_dir.y = 0;
//        move_dir.z = 0;
//    }
}

bool Character::canAscend()
{
    return cam->pos().y < jumpHeight();
}

bool Character::isDropping()
{
//    return !is_ascending && cam->eye.y > characterHeight();
    return is_dropping;
}

void Character::enableDropping()
{
    is_dropping = true;
    activate(Action::Jump, false);
}

void Character::disableDropping()
{
    is_dropping = false;
}

void Character::makeDrop(float dt)
{
//    cam->eye.y -= dt*dropSpeed();
//    characterYPosFix();
}

int Character::hasItem(std::size_t const &item_id) const
{
    auto it = items.find(item_id);
    if (it == items.end())
        return 0;
    else
        return it->second;
}

bool Character::canCarry(std::size_t const &item_id, int n) const noexcept
{
    auto item = Item::lookup(item_id);
    if (item.id() == 0)
        return false;
    return item.collectible && !(n_items + n > max_slots && current_load + item.weight * n > max_load);
}

bool Character::collectItem(std::size_t const &item_id, int n)
{
    auto item = Item::lookup(item_id);
    if (item.id() == 0)
        return false;
    if (item.collectible && !(n_items + n > max_slots && current_load + item.weight * n > max_load))
    {
        auto it = items.find(item_id);
        if (it == items.end())
            items.emplace(item_id, n);
        else
            it->second += n;
        n_items += n;
        current_load += item.weight*n;
        return true;
    }
    return false;
}