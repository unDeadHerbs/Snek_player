#include "snek_game.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <deque>
#include <optional>
#include <queue>
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
  std::deque<std::pair<Point,int>> q{{from,0}};
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
int consideration_metric(Consideration const & val,Point food,int /*&*/ seek_distance){
  // TODO: This has become very expensive and should be memoized

  // A memoizer is a band-aid, the queue shouldn't be re-evaluating
  // known points that much.

  // If =contention= isn't needed any more then the values are fully
  // memoize-able and perhaps =std::priority_queue= will work again?

  // That depends on the implementation of =std::priority_queue=.

  // Let's try switching back and then make a =priority_stack= if this
  // is still a problem.

  // That's much faster and seems to not re-evaluate too much, a more
  // bespoke structure would probably still be better, but that's not
  // a good spending of time at the moment.
  int md=seek_distance*10;   // If past seek_distance then just be big
			    // as to indicate to not search here.
  auto ad=a_star_distance(val.second.Body(),val.second.Body()[0],food,seek_distance+4);
  if(ad) {
    md=*ad;
    seek_distance=md*1.3+3;
    // Dynamically reduce the seek distance as we approach the goal.

    // TODO: This needs undoing if the path is a failure on other metrics.
  }

  auto turns=count_turns(val.first);

  auto heuristic_distance = md*(1+(md>5)*4); // Consider decreasing this distance very important 4
  auto current_distance = val.first.size();
  //return md+current_distance;
  auto quick_explore = val.first.size()<5;
  //auto to_many_turns = (1+turns>1+(turns>3)*turns*(1+(turns>5)*turns)*(1+(turns>7)*turns));
  auto to_many_turns = turns;
  auto is_close = (md<3);// && (contention[val.second.Body()[0]]<3);
  //auto contentiousness = pow(contention[val.second.Body()[0]],2);

  auto distance_cost=heuristic_distance+current_distance;
  //return distance_cost;
  //auto dislikability=to_many_turns+(1+!quick_explore)+(1+!is_close);//+contentiousness;
  auto dislikability=to_many_turns;

  return distance_cost+dislikability;
}

template<typename T,typename U,typename V>
void drop_all_but_n(std::priority_queue<T,U,V>& q,int count){
  // TODO: This is a patch instead of writing =priority_stack=.
  if(q.size()>count){
    U tmp;
    while(tmp.size()<count){
      tmp.push_back(q.top());
      q.pop();
    }
    while(q.size())q.pop();
    while(tmp.size()){
      q.push(tmp.back());
      tmp.pop_back();
    }
  }
}

Path Astar(Snek s) {
  CLEAR();
  // TODO: This function slows down dramatically as more currently active paths are added.
  contention.clear();
  DBG("- In Astar\n");
  auto target=s.Food();
  auto tail_dist=*a_star_distance(s.Body(),s.Body().front(),s.Body().back());
  // That's guaranteed to be non-null by the last search.
  int food_linear=distance(s.Body()[0],target,1);
  auto start_dist_check=a_star_distance(s.Body(),s.Body().front(),target,food_linear+4);
  // If the food isn't within the linear distance then the body is in the way.
  int start_dist;
  if(start_dist_check)
    start_dist=*start_dist_check;
  else{
    start_dist=tail_dist;
    target=s.Body().back();
  }
  auto comp = [=,&start_dist](Consideration const &lhs, Consideration const &rhs) {
		return consideration_metric(lhs,target,start_dist) > consideration_metric(rhs,target,start_dist);
  };
  std::priority_queue<Consideration,std::deque<Consideration>,decltype(comp)> possibilities(comp);
  possibilities.push({{}, s});
  for (;;) {
    if(!possibilities.size()){
      possibilities.push({{}, s});
      start_dist=tail_dist;
      target=s.Body().back();
      //CLEAR();
    }
    //drop_all_but_n(possibilities,5);
    DBG("-- have " << possibilities.size() << " options\n");
    auto current = possibilities.top();
    possibilities.pop();
    DBG("-- Top has " << current.first.size() << " steps\n");
    DBG("-- Top has " << metric_distance(current.second) << " to go\n");
    for (auto dir :
         {Direction::up, Direction::right, Direction::down, Direction::left}) {
      Snek ss(current.second);
      ss.move(dir);
      if (ss.Alive()){
	auto p = current.first;
	p.push(dir);
	if (ss.Body()[0] == target and a_star_distance(ss.Body(),target,ss.Body().back())) {
	  DBG("--- found\n");
	  //CLEAR();
	  return p;
	}
	DBG("--- consideration\n");
	contention[ss.Body()[0]]++;
	if(a_star_distance(ss.Body(),ss.Body().front(),ss.Body().back())){
	  DBG2(ss.Body()[0]);
	  possibilities.push({p, ss});
	}
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
    if (!p.size())
      p = Astar(s);
    if(!p.size()) break;
    s.move(p.front());
    p.pop();
    s.updateDisplay();
  }
  // TODO: Wait
  return 0;
}
