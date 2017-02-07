#ifndef GAMEMANAGER_H
#define GAMEMANAGER_H
#include<SDL2/SDL.h>
#include "Marine.h"
#include "Turret.h"
#include "CollisionHandler.h"
#include <map>
#include <unordered_map>
#include <vector>

class GameManager {
public:
	
	GameManager();
	~GameManager();
	
	void renderObjects(SDL_Renderer* gRenderer, float camX, float camY); // Render all objects in level
	
	// Methods for creating, getting, and deleting marines from the level.
	unsigned int createMarine();
	void deleteMarine(unsigned int id);
	bool addMarine(unsigned int id, Marine* newMarine);
	Marine* getMarine(unsigned int id);
	
    // Methods for creating, getting, and deleting towers from the level.
    unsigned int createTurret();
    void deleteTurret(unsigned int id);
    bool addTurret(unsigned int id, Turret* newTurret);
    Turret* getTurret(unsigned int id);
    
	void updateCollider(); // Updates CollisionHanlder
	void updateMarines(const float& delta); // Update marine actions
	
private:
	
	CollisionHandler* collisionHandler = NULL;
	std::map<unsigned int, Marine*> marineManager;
    std::map<unsigned int, Turret*> turretManager;
	
};


#endif
