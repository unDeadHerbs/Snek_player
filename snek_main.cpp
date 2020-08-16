#include "snek_game.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <queue>

#define DEBUG1 1
#if DEBUG1
#include <fstream>
std::ofstream log_file;
#define DBG(X)                                                                 \
  do {                                                                         \
    log_file << X << std::flush;                                                    \
  } while (0)
#else
#define DBG(X)                                                                 \
  do {                                                                         \
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
    }                                                                          \
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
#include "Console-IO/ioconsole.hpp"
std::queue<Point> dots3;
#define DBG3(X)                                                                \
  do {                                                                         \
    if (udh::cio[(X).first][(X).second] == ' ') {                              \
      dots3.push(X);                                                            \
      udh::cio[(X).first][(X).second] = '.';                                   \
      udh::cio << std::flush;                                                  \
    }                                                                          \
  } while (0)
#define CLEAR3()                                                                \
  do {                                                                         \
    while (dots3.size()) {                                                      \
      auto d = dots3.front();                                                   \
      dots3.pop();                                                              \
      udh::cio[d.first][d.second] = ' ';                                       \
      udh::cio << std::flush;                                                  \
    }                                                                          \
  } while (0)
#else
#define DBG3(X)                                                                \
  do {                                                                         \
  } while (0)
#define CLEAR3()                                                                \
  do {                                                                         \
  } while (0)
#endif

using Direction = Snek::Direction;

bool tail_reachable(Snek const& s){
  // If the tail is reachable from the current head.
  return true; // TODO
}

int metric_distance(Snek const & s,int dims=1){
  return distance(s.Body()[0],s.Food(),dims);
}

typedef std::queue<Direction> Path;
typedef std::pair<Path,Snek> Consideration;

int count_turns(std::queue<Direction> v){
  auto c=0;
  Direction ld=v.front();
  while(v.size()){
    auto d=v.front();
    v.pop();
    if(ld!=d){
      ld=d;
      c++;
    }
  }
  return c;
}

std::map<Point,int> contention;
int consideration_metric(Consideration const & val){
  auto md=metric_distance(val.second);
  auto md2=metric_distance(val.second,2);
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

Path Astar(Snek s){
  contention.clear();
  DBG("- In Astar\n");
  auto comp=[](Consideration const & lhs,Consideration const & rhs){
	      return consideration_metric(lhs)>consideration_metric(rhs);
	    };
  //std::priority_queue<Consideration,std::vector<Consideration>,decltype(comp)> possibilities(comp);
  std::vector<Consideration> possibilities;
  possibilities.push_back({{},s});
  // check if would kill
  for(;;){
    DBG("-- have "<<possibilities.size()<<" options\n");
    std::sort(possibilities.begin(),possibilities.end(),comp);
    auto current=possibilities.back();
    possibilities.pop_back();
    DBG("-- Top has "<<current.first.size()<<" steps\n");
    DBG("-- Top has "<<metric_distance(current.second)<<" to go\n");
    for(auto dir:{Direction::up,Direction::right,Direction::down,Direction::left}){
      Snek ss(current.second);
      ss.move(dir);
      if(ss.Alive()){
	auto p=current.first;
	p.push(dir);
	if(ss.Body()[0]==s.Food()){
	  CLEAR();
	  if(tail_reachable(s))
	    return p;
	}else{
	  contention[ss.Body()[0]]++;
	  DBG2(ss.Body()[0]);
	}
	possibilities.push_back({p,ss});
      }
    }
  }
    
  //if(!ss.Alive())return {};
  // return happiness?
  // return 1;
}

int main() {
  #if DEBUG
  log = std::ofstream("trying.log",std::ios::out|std::ios::trunc);
  #endif
  DBG("Starting\n");
  	Snek s;
	s.drawWalls();
	s.updateDisplay();
	DBG("Call to Astar\n");
	auto p=Astar(s);
	DBG("Got a path of length "<<p.size()<<"\n");
	while (s.Alive()){
	  if(!p.size())
	    p=Astar(s);
	  s.move(p.front());
	  p.pop();
	  s.updateDisplay();
	}

	return 0;
}
