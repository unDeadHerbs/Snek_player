#include "snek_game.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <unistd.h>

// Point functions

int distance(Point const L, Point const R, int const metric) {
  auto const x_delta = abs(int(L.first) - int(R.first));
  auto const y_delta = abs(int(L.second) - int(R.second));
  if (metric == 1)
    return x_delta + y_delta;
  if (metric == 2)
    return abs(int(std::sqrt(x_delta * x_delta + y_delta * y_delta)));
  return std::max<int>(
      abs(int(std::pow(std::pow(abs(int(L.first) - int(R.first)), metric) +
                           std::pow(abs(int(L.second) - int(R.second)), metric),
                       1. / metric))),
      0);
}

// Snek Functions

Point rand_point(Point min, Point max) {
  return {std::rand() % int(max.first - min.first + 1) + int(min.first),
          std::rand() % int(max.second - min.second + 1) + int(min.second)};
}

Snek::Snek(Point siz) : size(siz) {
  game_tick = 0;
  length = 3; // starting length
  std::srand(uint(std::time(nullptr)));
  size.first -= 2;
  size.second -= 2;
  body.push_back({size.first / 2, size.second / 2});
  body_graphics.push_back('s'); // Start in the center.

  food = rand_point({1, 1}, {size.first - 1, size.second - 1});

  direction = none;
  alive = true;
}

void Snek::drawWalls(decltype(udh::cio) o) const {
  o[0][0] = '+';
  o[size.first + 1][0] = '+';
  o[0][size.second + 1] = '+';
  o[size.first + 1][size.second + 1] = '+';
  for (uint c = 1; c <= size.second; c++) {
    o[0][c] = '-';
    o[size.first + 1][c] = '-';
  }
  for (uint r = 1; r <= size.first; r++) {
    o[r][0] = '|';
    o[r][size.second + 1] = '|';
  }
}

void Snek::updateDisplay(decltype(udh::cio) o) const {
  if (body.size() > 1) {
    o[body[1].first][body[1].second] =
        body_graphics[1]; // In case we haven't moved and need to not delete
                          // that.
    o[body.back().first][body.back().second] =
        ' '; // Hidden, just to prevent walking on self.
  }
  if (alive)
    o[body.front().first][body.front().second] = body_graphics.front();
  else
    o[body.front().first][body.front().second] = '!';

  o[food.first][food.second] = 'a'; // Draw food over body in case of glitch

  // TODO: cio << position(0,0) << to_string(game_tick);
  uint pos = 0;
  for (char c : std::to_string(game_tick) + "/" +
                    std::to_string(int(std::pow(size.first * size.second, 2))) +
                    "---" + std::to_string(body.size()) + "/" +
                    std::to_string(size.first * size.second))
    o[0][pos++] = c;
  o << std::flush;
  usleep(sleep_time);
}

void Snek::place_food() {
  switch (difficulty) {
  case 1: {
    uint dif = 0;
    while (std::find(body.begin(), body.end(), food) !=
           body.end()) // Generate a valid food placement just in case.
      food = rand_point({1, 1}, {size.first - 1, size.second - 1});
    for (uint x = 1; x <= size.first; x++)
      for (uint y = 1; y <= size.second; y++)
        if (std::find(body.begin(), body.end(), decltype(food){x, y}) ==
            body.end()) {
          uint d = 1;
          if (d > dif) {
            dif = d;
            food = {x, y};
          }
          // else if(d==diff) pic random between options, might need to be a
          // set for picking from afterwards.
        }
  } break;
  case 0:
  default:
    while (std::find(body.begin(), body.end(), food) !=
           body.end()) // Generate a new food.
      food = rand_point({1, 1}, {size.first - 1, size.second - 1});
  }
}

bool Snek::move(Direction movement_input) {
  game_tick++;
  // TOOD: return check for body
  if (movement_input != none)
    direction = movement_input;
  switch (direction) {
  case none:
    return true;
  case up:
    body.insert(body.begin(), {body[0].first - 1, body[0].second});
    body_graphics.insert(body_graphics.begin(), '^');
    if (body_graphics.size() > 1)
      switch (body_graphics[1]) {
      case '^':
        body_graphics[1] = BOX_CHAR ? 0x2502 : '|';
        break;
      case '>':
        body_graphics[1] = BOX_CHAR ? 0x2518 : '/';
        break;
      case 'v':
        break;
      case '<':
        body_graphics[1] = BOX_CHAR ? 0x2514 : '\\';
        break;
      }
    break;
  case right:
    body.insert(body.begin(), {body[0].first, body[0].second + 1});
    body_graphics.insert(body_graphics.begin(), '>');
    if (body_graphics.size() > 1)
      switch (body_graphics[1]) {
      case '^':
        body_graphics[1] = BOX_CHAR ? 0x250c : '/';
        break;
      case '>':
        body_graphics[1] = BOX_CHAR ? 0x2500 : '-';
        break;
      case 'v':
        body_graphics[1] = BOX_CHAR ? 0x2514 : '\\';
        break;
      case '<':
        break;
      }
    break;
  case down:
    body.insert(body.begin(), {body[0].first + 1, body[0].second});
    body_graphics.insert(body_graphics.begin(), 'v');
    if (body_graphics.size() > 1)
      switch (body_graphics[1]) {
      case '^':
        break;
      case '>':
        body_graphics[1] = BOX_CHAR ? 0x2510 : '\\';
        break;
      case 'v':
        body_graphics[1] = BOX_CHAR ? 0x2502 : '|';
        break;
      case '<':
        body_graphics[1] = BOX_CHAR ? 0x250c : '/';
        break;
      }
    break;
  case left:
    body.insert(body.begin(), {body[0].first, body[0].second - 1});
    body_graphics.insert(body_graphics.begin(), '<');
    if (body_graphics.size() > 1)
      switch (body_graphics[1]) {
      case '^':
        body_graphics[1] = BOX_CHAR ? 0x2510 : '\\';
        break;
      case '>':
        break;
      case 'v':
        body_graphics[1] = BOX_CHAR ? 0x2518 : '/';
        break;
      case '<':
        body_graphics[1] = BOX_CHAR ? 0x2500 : '-';
        break;
      }
    break;
  }
  if (body[0] == food) {
    place_food();
    length += 1; // Amount grown by each food.
  }
  while (body.size() > length) {
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
