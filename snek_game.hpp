#ifndef __SNEK_CLASS_HPP__
#define __SNEK_CLASS_HPP__
#include <string>
#include <utility>
#include <vector>

typedef std::pair<uint, uint> Point;
int distance(Point, Point, int);
class Snek {
  Point size;
  std::vector<Point> body; // [0] is head
  std::vector<wchar_t> body_graphics; // [0] is head
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
  void drawWalls() const;
  void updateDisplay() const;

public:
  Snek();
  bool Alive() const { return alive; }
  bool move(Direction movement = none);
};

#endif
