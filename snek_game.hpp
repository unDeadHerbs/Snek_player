#ifndef __SNEK_CLASS_HPP__
#define __SNEK_CLASS_HPP__

#include <string>
#include <utility>
#include <vector>

class Snek {
	std::pair<uint, uint> size;
	std::vector<std::pair<uint, uint>> body;  // [0] is head
	std::pair<uint, uint> food;

 public:
	typedef enum { none, up, right, down, left } Direction;
  useconds_t sleep_time=100000;

 private:
	Direction direction;
	bool alive;

	void drawWalls();
	void updateDisplay();

 public:
	Snek();
	bool Alive() { return alive; }
	bool move(Direction movement = none);
};

#endif
