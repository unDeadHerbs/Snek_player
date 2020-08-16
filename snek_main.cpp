#include "snek_game.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <queue>

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

#define DEBUG3 1
#if DEBUG3
#include "Console-IO/ioconsole.hpp"
#include <unistd.h>
std::queue<Point> dots3;
#define DBG3(X)								\
  do {									\
    if (udh::cio[(X).first][(X).second] == ' ') {			\
      dots3.push(X);							\
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

std::optional<int> a_star_distance_something_wrong(Snek const &s,Point from, Point to) {
  if(from==to)return 0;
  auto walls = s.Body();
  DBG("= a_star start\n");
  auto metric=[=](std::pair<Point,int> pnt){
		return distance(pnt.first,to,2)*2+pnt.second;
	      };
  auto comp=[&](std::pair<Point,int> lhs,std::pair<Point,int> rhs){
	      return metric(lhs)>metric(rhs);
	    };
  //std::priority_queue<std::pair<Point,int>,std::vector<std::pair<Point,int>>,decltype(comp)> q(comp);
  std::vector<std::pair<Point,int>> q{{from,0}};
  while(q.size()){
    std::sort(q.begin(),q.end(),comp);
    DBG("= a_star size = "<<q.size()<<"\n");
    auto n=q.back();
    q.pop_back();
    DBG("== a_star distance = "<<distance(n.first,to,2)<<"\n");
    DBG("== a_star length = "<<n.second<<"\n");
    DBG("== a_star metric = "<<metric(n)<<"\n");
    DBG("== a_star pos = {"<<n.first.first<<","<<n.first.second<<"}\n");
    DBG3(n.first);
   auto allowable=[=](Point pt)->bool{
		    if(pt==to)
		      return true;
		    if(std::find(walls.begin(),walls.end(),pt)!=walls.end())
		      return false;
		    if(pt.first<0 or pt.second<0 or pt.first>s.Size().first or pt.second>s.Size().second)
		      return false;
		    return true;
		  };
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}){
      if(n.first+dir == to or distance(n.first+dir,to,1)<2)
	return n.second;
      if(allowable(n.first+dir)){
	DBG("=== adding "<<dir<<"\n");
	q.push_back({n.first+dir,n.second+1});
      }
    }
  }
  DBG("= a_star not found\n");
  CLEAR3();
  return {};
}

// TODO: either add a flood fill to check for possibility or have a
// max seek depth (usually easy to provide from above).
std::optional<int> a_star_distance(std::vector<Point>const walls,Point from, Point to) {
  CLEAR3();
  if(from==to)return 0;
  auto metric=[=](std::pair<Point,int> pnt){return distance(pnt.first,to,2)*2+pnt.second;};
  auto gt=[=](std::pair<Point,int> lhs,std::pair<Point,int> rhs){return metric(lhs)>metric(rhs);};
  std::vector<std::pair<Point,int>> tries{{from,0}};
  while(tries.size()){
    std::sort(tries.begin(),tries.end(),gt);
    auto tri=tries.back();
    tries.pop_back();
    DBG3(tri.first);
    DBG("= a_star try={"<<tri.first.first<<","<<tri.first.second<<"}\n");
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}){
      if(tri.first+dir==to)
	return tri.second;
      if(std::find(walls.begin(),walls.end(),tri.first+dir)!=walls.end())
	continue;
      // todo check if in bounds
      tries.push_back({tri.first+dir,tri.second+1});
    }
  }
  return {};
}

int metric_distance(Snek const &s,Point food, int dims = 1) {
  return distance(s.Body()[0], food, dims);
}

typedef std::queue<Direction> Path;
typedef std::pair<Path, Snek> Consideration;

int count_turns(std::queue<Direction> v) {
  auto c = 0;
  Direction ld = v.front();
  while (v.size()) {
    auto d = v.front();
    v.pop();
    if (ld != d) {
      ld = d;
      c++;
    }
  }
  return c;
}

std::map<Point,int> contention;
int consideration_metric(Consideration const & val,Point food){
  // TODO: This has become very expensive and should be memoized.
  auto md=metric_distance(val.second,food);
  auto md2=metric_distance(val.second,food,2);
  auto ad=a_star_distance(val.second.Body(),val.second.Body()[0],food);
  if(ad){
    md=*ad;
    md2=*ad;
  }
  auto turns=count_turns(val.first);

  auto heuristic_distance = md2*(1+(md>5)*4);
  auto current_distance = val.first.size();
  auto quick_explore = val.first.size()<5;
  auto to_many_turns = (1+turns>1+(turns>3)*turns*(1+(turns>5)*turns)*(1+(turns>7)*turns));
  auto is_close = (md<3) && (contention[val.second.Body()[0]]<3);
  auto contentiousness = pow(contention[val.second.Body()[0]],2);

  auto distance_cost=heuristic_distance+current_distance;
  auto dislikability=to_many_turns+(1+!quick_explore)+contentiousness+(1+!is_close);

  return distance_cost*dislikability;
}

Path Astar(Snek s) {
  // TODO: This function slows down dramatically as more currently active paths are added.
  contention.clear();
  DBG("- In Astar\n");
  auto food=s.Food();
  auto comp = [=](Consideration const &lhs, Consideration const &rhs) {
		return consideration_metric(lhs,food) > consideration_metric(rhs,food);
  };
  // std::priority_queue<Consideration,std::vector<Consideration>,decltype(comp)>
  std::vector<Consideration> possibilities;
  possibilities.push_back({{}, s});
  for (;;) {
    DBG("-- have " << possibilities.size() << " options\n");
    std::sort(possibilities.begin(), possibilities.end(), comp);
    auto current = possibilities.back();
    possibilities.pop_back();
    DBG("-- Top has " << current.first.size() << " steps\n");
    DBG("-- Top has " << metric_distance(current.second) << " to go\n");
    for (auto dir :
         {Direction::up, Direction::right, Direction::down, Direction::left}) {
      Snek ss(current.second);
      ss.move(dir);
      if (ss.Alive()){
	a_star_distance(ss.Body(),food,ss.Body().back());
	auto p = current.first;
        p.push(dir);
        if (ss.Body()[0] == food) { 
	 DBG("--- found\n");
       	  CLEAR();
	  return p;
	}
	DBG("--- consideration\n");
	contention[ss.Body()[0]]++;
	DBG2(ss.Body()[0]);
	possibilities.push_back({p, ss});
      }else
	DBG("--- non-consider\n");
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
  DBG("Call to Astar\n");
  auto p = Astar(s);
  DBG("Got a path of length " << p.size() << "\n");
  while (s.Alive()) {
    // TODO: Astar can return {}.
    if (!p.size())
      p = Astar(s);
    s.move(p.front());
    p.pop();
    s.updateDisplay();
  }

  return 0;
}
