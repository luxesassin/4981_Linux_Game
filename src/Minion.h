/*------------------------------------------------------------------------------------------------------------------
-- HEADER FILE: Minion.h
--
-- PROGRAM:     Astar
--
-- FUNCTIONS:

--
-- DATE:        February 1, 2017
--
-- DESIGNER:    Fred Yang, Robert Arendac, Jeonghun Lee
--
-- PROGRAMMER:  Fred Yang, Robert Arendac, Jeonghun Lee
--
-- NOTES:
--
----------------------------------------------------------------------------------------------------------------------*/
#ifndef MINION_H
#define MINION_H

#include "Astar.h"

class Minion {
public:
    explicit Minion(const int xPos = 0, const int yPos = 0, const int hp = 100,
                    const int pwr = 100, const int amr = 50,
                    const int agil = 50, const int mode = 0)
    : xPos_(xPos), yPos_(yPos), hp_(hp), pwr_(pwr), amr_(amr),
      agil_(agil), mode_(mode) {}
    virtual ~Minion() {}

    // methods
    virtual void move(int destX, int destY) {
        std::vector<std::pair<int, int>> path;

        path = buildCoordinatePath(xPos_, yPos_, destX, destY);

        setMode(1);

        for (auto i = path.begin(); i != path.end(); ++i) {
            setPosition(*i);
            //Might want to wait or something so zombie doesn't teleport
        }
    }
    virtual void attack() {
        setMode(2);
    }
    virtual void die() {}

    virtual void hit() {
        hp_--;
        if (hp_ <= 0) {
            die();
        }
    }

    virtual void setPosition(int xCoord, int yCoord) {
        xPos_ = xCoord;
        yPos_ = yCoord;
    }

    virtual void setPosition(std::pair<int, int> loc) {
        xPos_ = loc.first;
        yPos_ = loc.second;
    }

    virtual std::pair<int, int> getPostion() {
        return std::make_pair(xPos_, yPos_);
    }

    virtual void setMode(int mode) {
        if (mode < 3 && mode >= 0) {
            mode_ = mode;
        }
    }

private:
    /* Some of these variables may not be used for the final game.  These
    were made with the thought of having multiple zombie types, so it's fine
    if they are deleted.  They don't really do too much right now. */

    int xPos_;      // X coordinate on the map
    int yPos_;      // Y coordinate on the map
    int hp_;        // health points
    int pwr_;       // power
    int amr_;       // armor
    int agil_;      // agility
    int mode_;      // 0-idle, 1-move, 2-attack
};

#endif
