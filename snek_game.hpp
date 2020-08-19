#ifndef SNEK_CLASS_HPP
#define SNEK_CLASS_HPP
#include <string>
#include <utility>
#include <vector>
#include "Console-IO/ioconsole.hpp"

#define BOX_CHAR 0


typedef std::pair<uint, uint> Point;
int distance(Point, Point, int);
class Snek {
  Point size;
  uint length;
  std::vector<Point> body; // [0] is head
  #if BOX_CHAR
  std::vector<wchar_t> body_graphics; // [0] is head
  #else
  std::vector<char> body_graphics; // [0] is head
  #endif
  Point food;
  int game_tick;

public:
  auto Size() const{return size;}
  auto Body() const { return body; }
  auto Food() const { return food; }

public:
  typedef enum { none, up, right, down, left } Direction;
  useconds_t sleep_time=100000;

private:
  Direction direction;
  bool alive;

public: // Not sure about this
  void drawWalls(decltype(udh::cio)) const;
  void updateDisplay(decltype(udh::cio)) const;

public:
  Snek(Point);
  bool Alive() const { return alive; }
  bool move(Direction movement = none);
};

#endif
