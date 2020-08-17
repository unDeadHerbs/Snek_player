#include "snek_game.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <deque>
#include <optional>
#include <queue>
#include <iterator>
#include "Console-IO/ioconsole.hpp"

#define DEBUG1 0
#if DEBUG1
#include <fstream>
std::ofstream log_file;
#define DBG(X)								\
  do {									\
    log_file << X << std::flush;					\
  } while (0)
#else
#define DBG(X)								\
  do {									\
  } while (0)
#endif

#define DEBUG2 1
#if DEBUG2
#include "Console-IO/ioconsole.hpp"
std::queue<Point> dots;
#define DBG2(X)                                                                \
  do {                                                                         \
    if (udh::cio[(X).first][(X).second] == ' ') {                              \
      dots.push(X);                                                            \
      udh::cio[(X).first][(X).second] = '.';                                   \
      udh::cio << std::flush;                                                  \
    }                                                                          \
  } while (0)
#define CLEAR()                                                                \
  do {                                                                         \
    while (dots.size()) {                                                      \
      auto d = dots.front();                                                   \
      dots.pop();                                                              \
      if(udh::cio[d.first][d.second] == '.')				\
      udh::cio[d.first][d.second] = ' ';                                       \
      udh::cio << std::flush;                                                  \
    }									\
  } while (0)
#else
#define DBG2(X)                                                                \
  do {                                                                         \
  } while (0)
#define CLEAR()                                                                \
  do {                                                                         \
  } while (0)
#endif

#define DEBUG3 0
#if DEBUG3
#include <unistd.h>
std::queue<Point> dots3;
#define DBG3(X)								\
  do {									\
    if (udh::cio[(X).first][(X).second] == ' ') {			\
      dots3.push(X);							\
      udh::cio[(X).first][(X).second] = '|';				\
      udh::cio << std::flush;						\
      udh::cio[(X).first][(X).second] = '_';				\
      udh::cio << std::flush;						\
    }									\
  } while (0)
#define CLEAR3()                                                                \
  do {									\
    /*sleep(1);*/							\
    while (dots3.size()) {						\
      auto d = dots3.front();						\
      dots3.pop();							\
      if(udh::cio[d.first][d.second] == '_')\
	udh::cio[d.first][d.second] = ' ';				\
      udh::cio << std::flush;						\
    }									\
  } while (0)
#else
#define DBG3(X)								\
  do {									\
  } while (0)
#define CLEAR3()							\
  do {									\
  } while (0)
#endif

using Direction = Snek::Direction;

// TODO: Make point and dir into more established things.
Point operator+(Point const pnt, Direction const dir){
  switch(dir){
  case Direction::none:
    return pnt;
  case Direction::up:
    if(pnt.second==0)return {pnt.first,pnt.second};
    return {pnt.first,pnt.second-1};
  case Direction::right:
    return {pnt.first+1,pnt.second};
  case Direction::down:
    return {pnt.first,pnt.second+1};
  case Direction::left:
    if(pnt.first==0)return {pnt.first,pnt.second};
    return {pnt.first-1,pnt.second};
  }
}
Point& operator+=(Point & pnt, Direction const dir){
  return pnt=pnt+dir;
}

// TODO: Either add a flood fill to check for possibility or have a
// max seek depth (usually easy to provide from above).
std::optional<int> a_star_distance(std::vector<Point>const walls,Point from, Point to,int max_dist=0) {
  // A Star doesn't utilize the fact that there are only two walls,
  // one external and one the snake.  This can be used to remove a lot
  // of search options.
  CLEAR3();
  if(from==to)return 0;
  auto metric=[=](std::pair<Point,int> pnt){return distance(pnt.first,to,1)*2+pnt.second*(pnt.second>3);};
  auto gt=[=](std::pair<Point,int> lhs,std::pair<Point,int> rhs){return metric(lhs)>metric(rhs);};
  std::deque<std::pair<Point,int>> tries{{from,0}};
  std::deque<Point> tried;
  while(tries.size()){
    std::sort(tries.begin(),tries.end(),gt);
    auto tri=tries.back();
    tries.pop_back();
    DBG3(tri.first); // segv?
    DBG("= a_star try={"<<tri.first.first<<","<<tri.first.second<<"}\n");
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}){
      if(tri.first+dir==to)
	return tri.second;
      if(max_dist!=0 and tri.second > max_dist)
	return {}; // TODO: Perhaps give a few tries?
      if(std::find(walls.begin(),walls.end(),tri.first+dir)!=walls.end())
	continue;
      // TODO: check if in bounds
      if(std::find(tried.begin(),tried.end(),tri.first+dir)!=tried.end())
	// Segfaults on this =std::find=?
	continue;
      tries.push_back({tri.first+dir,tri.second+1});
      tried.push_back(tri.first+dir);
    }
  }
  return {};
}

bool reachable(std::vector<Point>const walls,Point from, Point to) {
  return a_star_distance(walls,from,to).has_value();
}

typedef std::vector<Direction> Path;

int count_turns(Path v) {
  auto c = 0;
  Direction ld = v.front();
  while (v.size()) {
    auto d = v.front();
    v.erase(v.begin(),v.begin()+1);//pop();
    if (ld != d) {
      ld = d;
      c++;
    }
  }
  return c;
}

int snek_aware_distance(Snek game,Point goal){
  auto walls=game.Body();
  auto pnt=walls[0];
  auto xdist=std::abs(int(pnt.first)-int(goal.first));
  auto ydist=std::abs(int(pnt.second)-int(goal.second));
  int basic_dist=xdist+ydist;

  pnt=walls[0];
  int linear_dist=basic_dist;
  while(pnt!=goal){
    DBG3(pnt);
    if(auto intersect=std::find(walls.begin(),walls.end(),pnt);intersect!=walls.end()){
      CLEAR3();
      auto wait_dist=std::min(std::distance(intersect,walls.end()),
			      std::distance(intersect,walls.begin()));
      linear_dist+=wait_dist;
      break;
    }
    xdist=std::abs(int(pnt.first)-int(goal.first));
    ydist=std::abs(int(pnt.second)-int(goal.second));
    if(xdist>ydist)
      if(pnt.first>goal.first)
	pnt+=Direction::left;
      else
	pnt+=Direction::right;
    else
      if(pnt.second>goal.second)
	pnt+=Direction::up;
      else
	pnt+=Direction::down;
  }

  pnt=walls[0];
  int x_first_dist=basic_dist;
  while(pnt!=goal){
    DBG3(pnt);
    if(auto intersect=std::find(walls.begin(),walls.end(),pnt);intersect!=walls.end()){
      CLEAR3();
      auto wait_dist=std::min(std::distance(intersect,walls.end()),
			      std::distance(intersect,walls.begin()));
      x_first_dist+=wait_dist;
      break;
    }
    if(pnt.first!=goal.first)
      if(pnt.first>goal.first)
	pnt+=Direction::left;
      else
	pnt+=Direction::right;
    else
      if(pnt.second>goal.second)
	pnt+=Direction::up;
      else
	pnt+=Direction::down;
  }

  pnt=walls[0];
  int y_first_dist=basic_dist;
  while(pnt!=goal){
    DBG3(pnt);
    if(auto intersect=std::find(walls.begin(),walls.end(),pnt);intersect!=walls.end()){
      CLEAR3();
      auto wait_dist=std::min(std::distance(intersect,walls.end()),
			      std::distance(intersect,walls.begin()));
      y_first_dist+=wait_dist;
      break;
    }
    if(pnt.second==goal.second)
      if(pnt.first>goal.first)
	pnt+=Direction::left;
      else
	pnt+=Direction::right;
    else
      if(pnt.second>goal.second)
	pnt+=Direction::up;
      else
	pnt+=Direction::down;
  }
  // TODO: add waiting time
  return std::min(std::min(x_first_dist,y_first_dist),linear_dist);
}

struct Consideration{
  Path path;
  Snek game;
};

Path AI(Snek const & game){
  auto goal=game.Food();
  CLEAR();
  auto metric=[=](Consideration con)->int{
		int distance_guess=snek_aware_distance(con.game,goal)+con.path.size();
		int prefer_less_turns=count_turns(con.path);
		int when_lost_find_tail=(snek_aware_distance(con.game,con.game.Body().back())
					 +distance(con.game.Body()[0],con.game.Body().back(),1))>>2;
		int prefer_developed_paths=snek_aware_distance(con.game,goal)>>2;
		int escape_head=(con.path.size()<5)*-5;
		return
		  distance_guess+prefer_less_turns+when_lost_find_tail+prefer_developed_paths + escape_head;
	      };
  auto comp=[&](Consideration lhs,Consideration rhs){
	      return metric(lhs)>metric(rhs);
	    };
  std::priority_queue<Consideration,std::deque<Consideration>,decltype(comp)> possibilities(comp);
  possibilities.push({{},game});
  while(possibilities.size()){
    auto trying=possibilities.top();
    possibilities.pop();
    if(!possibilities.size())
      if(trying.path.size()) // since all must descend from this, lets advance the board
	return trying.path;
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}) {
      Snek t_game(trying.game);
      t_game.move(dir);
      if(!t_game.Alive())
	continue;
      //if (!reachable(t_game.Body(),t_game.Body().front(),t_game.Body().back()))
      if (!a_star_distance(t_game.Body(),t_game.Body().front(),t_game.Body().back(),t_game.Body().size()))
	continue;
      Path p(trying.path);
      p.push_back(dir);
      if(t_game.Body()[0]==goal)
	return p;
      DBG2(t_game.Body()[0]);
      possibilities.push({p,t_game});
    }
  }
  return {};
}

int main() {
#if DEBUG1
  log_file = std::ofstream("trying.log", std::ios::out | std::ios::trunc);
#endif
  DBG("Starting\n");
  Snek s;
  s.drawWalls();
  s.updateDisplay();
  DBG("Call to AI\n");
  auto p = AI(s);
  DBG("Got a path of length " << p.size() << "\n");
  while (s.Alive()) {
    if (!p.size())
      p = AI(s);
    if(!p.size()) break;
    s.move(p.front());
    p.erase(p.begin(),p.begin()+1); // pop front
    s.updateDisplay();
  }
  // TODO: Wait
  return 0;
}
