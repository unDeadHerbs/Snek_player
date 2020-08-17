#include "snek_game.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <deque>
#include <forward_list>
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
#define DBG2(X,SYM)							\
  do {                                                                         \
  if (udh::cio[(X).first][(X).second] == ' '				\
	     or udh::cio[(X).first][(X).second] == '.'			\
      or udh::cio[(X).first][(X).second] == ','				\
      or udh::cio[(X).first][(X).second] == '_'){			\
    dots.push(X);							\
      udh::cio[(X).first][(X).second] = SYM;                                   \
      udh::cio << std::flush;                                                  \
    }                                                                          \
  } while (0)
#define CLEAR()                                                                \
  do {                                                                         \
    while (dots.size()) {                                                      \
      auto d = dots.front();                                                   \
      dots.pop();                                                              \
      if(udh::cio[d.first][d.second] == '.' \
	 or udh::cio[d.first][d.second] == ',' \
	 or udh::cio[d.first][d.second] == '_')				\
      udh::cio[d.first][d.second] = ' ';                                       \
      udh::cio << std::flush;                                                  \
    }									\
  } while (0)
#else
#define DBG2(X,SYM)							\
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

bool reachable_flood(std::vector<Point> walls,Point from, Point to,Point border){
  CLEAR3();
  if(from==to)return true;
  std::queue<Point> to_consider{{from}};
  while(!to_consider.empty()){
    DBG("--- progress "<<to_consider.size()<<"/"<<walls.size()<<"/"<<border.first*border.second<<"\n");
    Point p=to_consider.front();
    DBG("---- point=(" << p.first<<","<<p.second<<")\n");
    DBG3(p);
    to_consider.pop();
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}){
      auto test=p+dir;
      if(test==to)
	return true;
      if(std::find(walls.begin(),walls.end(),test)!=walls.end())
	continue;
      if(test.first<=0 or test.second<=0 or test.first>border.first or test.second>border.second)
	// Somehow it was skipping over the border?
	continue;
      to_consider.push(test);
      walls.push_back(test);
    }
  }
  DBG("--- Flood Fail\n");
  return false;
}

bool reachable(std::vector<Point>const walls,Point from, Point to,Point boarder) {
  DBG("--- reachable?\n");
  return reachable_flood(walls,from,to,boarder);
  // TODO: This is still a pretty bad way of checking, much better
  // than A* though.  Make something that understands regions and the
  // single internal "wall" nature of the map.
}

typedef std::vector<Direction> Path;

int count_turns(Path v) {
  auto c = 0;
  if(v.empty())return 0;
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

int snek_aware_distance(Snek const & game,Point goal){
  auto walls=game.Body();
  auto pnt=walls[0];
  auto xdist=std::abs(int(pnt.first)-int(goal.first));
  auto ydist=std::abs(int(pnt.second)-int(goal.second));
  int basic_dist=xdist+ydist;
  if(walls.size()==1)
    return basic_dist;

  pnt=walls[0];
  int linear_dist=basic_dist;
  while(pnt!=goal){
    DBG3(pnt);
    if(auto intersect=std::find(walls.begin()+1,walls.end(),pnt);intersect!=walls.end()){
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
    if(auto intersect=std::find(walls.begin()+1,walls.end(),pnt);intersect!=walls.end()){
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
    if(auto intersect=std::find(walls.begin()+1,walls.end(),pnt);intersect!=walls.end()){
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

template<typename T,typename VAL,bool GREATER=true>
class priority_stack{
  // TODO: assert that GT is int(*)(T);
  VAL val;
  std::forward_list<std::pair<T,int>> list;
public:
  priority_stack(VAL op):val(op){}
  bool empty()const{return list.empty();}
  void push(T elm){
    int v=val(elm);
    if(empty()){
      list.push_front({elm,v});
      return;
    }
    auto ptr=list.begin();
    if((!GREATER and v<ptr->second) or
       (GREATER and v>ptr->second)){
      list.push_front({elm,v});
      return;
    }
    while(std::next(ptr)!=list.end() and ((GREATER and std::next(ptr)->second>v)
				       or (!GREATER  and std::next(ptr)->second<v)))
      ++ptr;
    list.insert_after(ptr,{elm,v});
  }
  T pop(){
    auto ret=list.front().first;
    list.pop_front();
    return ret;
  }
};

struct Consideration{
  Path path;
  Snek game;
};

Path AI(Snek const & game){
  auto goal=game.Food();
  CLEAR();
  auto metric=[=](Consideration con)->int{
		auto b=con.game.Body();
		int distance_guess=10*(snek_aware_distance(con.game,goal)+con.path.size());
		int prefer_less_turns=5*count_turns(con.path);
		int when_lost_find_tail=(snek_aware_distance(con.game,b.back())
					 +distance(b[0],b.back(),1));
		int prefer_developed_paths=5*snek_aware_distance(con.game,goal); // dis-prefer undeveloped
		int escape_head=-100*(con.path.size()<5);
		return
		  distance_guess+prefer_less_turns+when_lost_find_tail+prefer_developed_paths + escape_head;
	      };
  priority_stack<Consideration,decltype(metric),false> possibilities(metric);
  possibilities.push({{},game});
  while(!possibilities.empty()){
    auto trying=possibilities.pop();
    if(possibilities.empty())
      if(trying.path.size()){ // since all must descend from this, lets advance the board
	DBG("-- Short Cutting\n");
	return trying.path;
      }
    DBG("- considering point\n");
    DBG2(trying.game.Body()[0],'_');
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}) {
      DBG("-- consider point "<<dir<<"\n");
      Snek t_game(trying.game);
      t_game.move(dir);
      DBG("-- check valid point "<<dir<<"\n");
      if(!t_game.Alive())
	continue;
      if (!reachable(t_game.Body(),t_game.Body().front(),t_game.Body().back(),t_game.Size()))
	continue;
      Path p(trying.path);
      p.push_back(dir);
      DBG("-- check done point "<<dir<<"\n");
      if(t_game.Body()[0]==goal)
	return p;
      DBG("-- Adding consider point "<<dir<<"\n");
      DBG2(t_game.Body()[0],'.');
      possibilities.push({p,t_game});
    }
    DBG("- Adding considered point\n");    
    DBG2(trying.game.Body()[0],',');
  }
  DBG("No Path Found\n");
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
    if (!p.size()){
      DBG("\n\nNext Food\n");
      p = AI(s);
      DBG("Moving\n");
    }
    if(!p.size()) break;
    s.move(p.front());
    p.erase(p.begin(),p.begin()+1); // pop front
    s.updateDisplay();
  }
  // TODO: Wait
  return 0;
}
