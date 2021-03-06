#ifndef GAMEMANAGER_H
#define GAMEMANAGER_H

#include <SDL2/SDL.h>
#include <map>
#include <unordered_map>
#include <vector>
#include <memory>

#include "../creeps/Zombie.h"
#include "../player/Marine.h"
#include "../turrets/Turret.h"
#include "../collision/CollisionHandler.h"
#include "../buildings/Object.h"
#include "../buildings/Base.h"
#include "../buildings/Wall.h"
#include "../buildings/Barricade.h"
#include "../inventory/WeaponDrop.h"


//just for tesing weapon drop
#include "../inventory/weapons/Weapon.h"
#include "../inventory/weapons/HandGun.h"
#include "../inventory/weapons/Rifle.h"
#include "../inventory/weapons/ShotGun.h"

constexpr int initVal = 0;
constexpr int defaultSize = 100;
constexpr int PUSize = 120;


class GameManager {
public:
    static GameManager *instance();

    int32_t generateID();

    void renderObjects(const SDL_Rect& cam); // Render all objects in level

    // Methods for creating, getting, and deleting marines from the level.
    int32_t createMarine();
    bool createMarine(const float x, const float y);
    void deleteMarine(const int32_t id);

    bool addMarine(const int32_t id, const Marine& newMarine);
    Marine& getMarine(const int32_t id);

    // Methods for creating, getting, and deleting towers from the level.
    int32_t createTurret();
    void deleteTurret(const int32_t id);

    bool addTurret(const int32_t id, const Turret& newTurret);
    int32_t createTurret(const float x, const float y) ;
    Turret& getTurret(const int32_t id);
    auto& getTurretManager() const {return turretManager;};

    // Method for getting collisionHandler
    CollisionHandler& getCollisionHandler();

    void updateCollider(); // Updates CollisionHandler
    void updateMarines(const float delta); // Update marine actions
    void updateZombies(const float delta); // Update zombie actions
    void updateTurrets(const float delta); // Update turret actions

    // returns the list of zombies.
    // Jamie, 2017-03-01.
    auto& getZombies() {
        return zombieManager;
    }

    int32_t addObject(const Object&);
    void deleteObject(const int32_t id);

    int32_t addZombie(const Zombie&);
    bool createZombie(const float x, const float y);
    void deleteZombie(const int32_t id);
    bool createZombieWave(const int n);

    int32_t addWeaponDrop(WeaponDrop& newWeaponDrop);
    bool createWeaponDrop(const float x, const float y);
    void deleteWeaponDrop(const int32_t id);
    WeaponDrop& getWeaponDrop(const int32_t id);
    std::shared_ptr<Weapon> getWeapon(const int32_t id);
    int32_t addWeapon(std::shared_ptr<Weapon> newWeaponDrop);

    int32_t createBarricade(const float x, const float y);
    void deleteBarricade(const int32_t id);
    Barricade& getBarricade(const int32_t id);

    int32_t createWall(const float x, const float y, const int h, const int w); // create Wall object
    void setBoundary(const float startX, const float startY, const float endX, const float endY); // place walls for the boundaries


private:
    GameManager();
    ~GameManager();
    static GameManager sInstance;

    CollisionHandler collisionHandler;
    std::unique_ptr<WeaponDrop> wdPointer;
    std::map<int32_t, Marine> marineManager;
    std::map<int32_t, Object> objectManager;
    std::map<int32_t, Zombie> zombieManager;
    std::map<int32_t, Turret> turretManager;
    std::map<int32_t, WeaponDrop> weaponDropManager;
    std::map<int32_t, std::shared_ptr<Weapon>> weaponManager;
    std::map<int32_t, Barricade> barricadeManager;
    std::map<int32_t, Wall> wallManager;

};


#endif
