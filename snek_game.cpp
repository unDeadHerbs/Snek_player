#include "snek_game.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <unistd.h>

#include "Console-IO/ioconsole.hpp"

// Point functions

int distance(Point L, Point R, int metric) {
  if(metric==1)
    return abs(int(L.first) - int(R.first)) + abs(int(L.second) - int(R.second));
  if(metric==2)
    return std::max<int>(
			 abs(int(std::sqrt(abs(int(L.first) - int(R.first))*abs(int(L.first) - int(R.first)) +
					   abs(int(L.second) - int(R.second))*abs(int(L.second) - int(R.second))
					 ))),
			       1);
  return std::max<int>(
		       abs(int(std::pow(std::pow(abs(int(L.first) - int(R.first)), metric) +
					std::pow(abs(int(L.second) - int(R.second)), metric),
					1. / metric))),
      1);
}

// Snek Functions

Point rand_point(Point min, Point max) {
  return {
      std::rand() % (max.first - min.first + 1) + min.first,
      std::rand() % (max.second - min.second + 1) + min.second,
  };
}

using udh::cio;

Snek::Snek() {
  std::srand(std::time(nullptr));
  size = cio.size();
  size.first -= 2;
  size.second -= 2;
  body.push_back({size.first / 2, size.second / 2});
  body_graphics.push_back('s'); // Start in the center.

  food = rand_point({1, 1}, {size.first - 1, size.second - 1});

  direction = none;
  alive = true;
}

void Snek::drawWalls() const {
  cio[0][0] = '+';
  cio[size.first + 1][0] = '+';
  cio[0][size.second + 1] = '+';
  cio[size.first + 1][size.second + 1] = '+';
  for (uint c = 1; c <= size.second; c++) {
    cio[0][c] = '-';
    cio[size.first + 1][c] = '-';
  }
  for (uint r = 1; r <= size.first; r++) {
    cio[r][0] = '|';
    cio[r][size.second + 1] = '|';
  }
}

void Snek::updateDisplay() const {
  static Point old_end;
  if (old_end!=Point{0,0})
    cio[old_end.first][old_end.second] = ' ';
  if (body.size() > 1){
    cio[body[1].first][body[1].second] =
        body_graphics[1]; // In case we haven't moved and need to not delete that.
    cio[body.back().first][body.back().second] =
      body_graphics.back(); // In case we haven't moved and need to not delete that.
  }
  if (alive)
    cio[body.front().first][body.front().second] = body_graphics.front();
  else
    cio[body.front().first][body.front().second] = '!';

  cio[food.first][food.second] = 'a'; // Draw food over body in case of glitch
  old_end = body.back();
  cio << std::flush;
  usleep(sleep_time);
}

bool Snek::move(Direction movement_input) {
  // TOOD: return check for body
  if (movement_input != none)
    direction = movement_input;
  switch (direction) {
  case none:
    return true;
  case up:
    body.insert(body.begin(), {body[0].first - 1, body[0].second});
    body_graphics.insert(body_graphics.begin(), '^');
    if(body_graphics.size()>1)
      switch(body_graphics[1]){
      case '^':
	body_graphics[1]='|';
	break;
      case '>':
	body_graphics[1]='/';
	break;
      case 'v':
	break;
      case '<':
	body_graphics[1]='\\';
	break;
      }
    break;
  case right:
    body.insert(body.begin(), {body[0].first, body[0].second + 1});
    body_graphics.insert(body_graphics.begin(), '>');
    if(body_graphics.size()>1)
      switch(body_graphics[1]){
      case '^':
	body_graphics[1]='/';
	break;
      case '>':
	body_graphics[1]='-';
	break;
      case 'v':
	body_graphics[1]='\\';
	break;
      case '<':
	break;
      }
    break;
  case down:
    body.insert(body.begin(), {body[0].first + 1, body[0].second});
    body_graphics.insert(body_graphics.begin(), 'v');
    if(body_graphics.size()>1)
      switch(body_graphics[1]){
      case '^':
	break;
      case '>':
	body_graphics[1]='\\';
	break;
      case 'v':
	body_graphics[1]='|';
	break;
      case '<':
	body_graphics[1]='/';
	break;
      }
    break;
  case left:
    body.insert(body.begin(), {body[0].first, body[0].second - 1});
    body_graphics.insert(body_graphics.begin(),'<');
    if(body_graphics.size()>1)
      switch(body_graphics[1]){
      case '^':
	body_graphics[1]='\\';
	break;
      case '>':
	break;
      case 'v':
	body_graphics[1]='/';
	break;
      case '<':
	body_graphics[1]='-';
	break;
      }
    break;
  }
  if (body[0] == food){
    while (std::find(body.begin(), body.end(), food) !=
           body.end()) { // Generate a new food.
      food = rand_point({1, 1}, {size.first - 1, size.second - 1});
      /*food.first++; // TODO: random, this is just lexicographic
      food.second+=food.first==size.first;
      food.first%=size.first;
      food.second%=size.second;
      if(food.first==0)food.first++;
      if(food.second==0)food.second++;*/
    }
  }
  else{
    body.pop_back(); // Don't get longer by one.
    body_graphics.pop_back();
  }
  if (body[0].first == 0 || body[0].first == size.first + 1 ||
      body[0].second == 0 || body[0].second == size.second + 1)
    alive = false;
  if (std::find(body.begin() + 1, body.end(), body[0]) !=
      body.end()) // If the new head is in the body, die.
    alive = false;
  return true;
}
