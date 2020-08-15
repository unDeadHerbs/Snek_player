#include "snek_game.hpp"

#include <unistd.h>
#include <algorithm>

#include "Console-IO/ioconsole.hpp"

using udh::cio;

Snek::Snek() {
	size = cio.size();
	size.first -= 2;
	size.second -= 2;
	body.push_back({size.first / 2, size.second / 2}); // Start in the center.

	food = body[0];
	food.second /= 2;  // TODO: random

	direction = none;
	alive = true;

	drawWalls();
	updateDisplay();
}

void Snek::drawWalls() {
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

void Snek::updateDisplay() {
	static std::vector<std::pair<uint, uint>> old_body;
	for (auto& p : old_body)
		cio[p.first][p.second] = ' ';  // should need to rm 1 or 0
	for (auto& p : body)
		cio[p.first][p.second] = '#';  // should only need to add one
	if (!alive) cio[body[0].first][body[0].second] = '!';
	cio[food.first][food.second] = 'a';
	old_body = body;
	cio << std::flush;
	usleep(sleep_time);
}

bool Snek::move(Direction movement_input) {
	// TOOD: return check for body
	if (movement_input != none) direction = movement_input;
	switch (direction) {
		case none:
			return true;
		case up:
			body.insert(body.begin(), {body[0].first - 1, body[0].second});
			break;
		case right:
			body.insert(body.begin(), {body[0].first, body[0].second + 1});
			break;
		case down:
			body.insert(body.begin(), {body[0].first + 1, body[0].second});
			break;
		case left:
			body.insert(body.begin(), {body[0].first, body[0].second - 1});
			break;
	}
	if (body[0] == food)
	  while(std::find(body.begin(),body.end(),food)!=body.end()){ // Generate a new food.
	    food.first++; // TODO: random, this is just lexicographic
	    food.second+=food.first==size.first;
	    food.first%=size.first;
	    food.second%=size.second;
	  }
	else
	  body.pop_back(); // Get longer by one.
	if (body[0].first == 0 || body[0].first == size.first + 1 ||
	    body[0].second == 0 || body[0].second == size.second + 1)
		alive = false;
	updateDisplay();
	return true;
}
