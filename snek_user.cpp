#include "snek_ai.hpp"
#include "snek_game.hpp"

#define DEBUG1 0
#if DEBUG1
#include <fstream>
std::ofstream log_file;
#define DBG(X)                                                                 \
  do {                                                                         \
    log_file << X << std::flush;                                               \
  } while (0)
#else
#define DBG(X)                                                                 \
  do {                                                                         \
  } while (0)
#endif

#include "Console-IO/ioconsole.hpp"

Snek::Direction get_move(Snek s /*temp for using AI in testing.*/) {
  // static Snek::Direction move = Snek::Direction::right;
  // delay

  // read user input.
  // - Discard all but the most recent arrow key.
  // Space is pause.
  // If no input is ready, return the last used input.
  auto AI = Snek_AI_go_right();
  auto moves = AI(s);
  return moves.front();
}

void game_loop() {
  DBG("Starting Visual Simulator\n");
  Snek s(udh::cio.size());
  s.drawWalls(udh::cio);
  while (s.Alive()) {
    s.updateDisplay(udh::cio);
    s.move(get_move(s));
  }
  s.updateDisplay(udh::cio);
}

int main() {
  game_loop();
  return 0;
}
