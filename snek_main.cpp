#include "snek_game.hpp"

using Direction = Snek::Direction;

class SnekAI{
  Snek const& s;
public:
  SnekAI(Snek const& S):s(S){}
  Direction get_move(){return Direction::left;}
};

int main() {
	Snek s;
	SnekAI ai(s);
	while (s.Alive()) s.move(ai.get_move());

	return 0;
}
