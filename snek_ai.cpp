#include "snek_ai.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <deque>
#include <list>
#include <optional>
#include <queue>
#include <iterator>

#define DEBUG1 0
#if DEBUG1
#include <fstream>
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#pragma clang diagnostic ignored "-Wglobal-constructors"
static std::ofstream log_file("trying.log", std::ios::out | std::ios::trunc);
#pragma clang diagnostic pop
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
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#pragma clang diagnostic ignored "-Wglobal-constructors"
static std::queue<Point> dots;
#pragma clang diagnostic pop
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

int unreachable_count(std::vector<Point> walls,Point from,Point border){
  CLEAR3();
  std::queue<Point> to_consider{{from}};
  while(!to_consider.empty()){
    // DBG("--- progress "<<to_consider.size()<<"/"<<walls.size()<<"/"<<border.first*border.second<<"\n");
    Point p=to_consider.front();
    // DBG("---- point=(" << p.first<<","<<p.second<<")\n");
    DBG3(p);
    to_consider.pop();
    for (auto dir :
	   {Direction::up, Direction::right, Direction::down, Direction::left}){
      auto test=p+dir;
      if(std::find(walls.begin(),walls.end(),test)!=walls.end())
	continue;
      if(test.first<=0 or test.second<=0 or test.first>border.first or test.second>border.second)
	continue;
      to_consider.push(test);
      walls.push_back(test);
    }
  }
  return int(border.first*border.second) - int(walls.size());// TODO
}

bool reachable_flood(std::vector<Point> walls,Point from, Point to,Point border){
  CLEAR3();
  if(from==to)return true;
  std::queue<Point> to_consider{{from}};
  while(!to_consider.empty()){
    // DBG("--- progress "<<to_consider.size()<<"/"<<walls.size()<<"/"<<border.first*border.second<<"\n");
    Point p=to_consider.front();
    // DBG("---- point=(" << p.first<<","<<p.second<<")\n");
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
	continue;
      to_consider.push(test);
      walls.push_back(test);
    }
  }
  // DBG("--- Flood Fail\n");
  return false;
}

bool reachable(std::vector<Point>const walls,Point from, Point to,Point boarder) {
  return reachable_flood(walls,from,to,boarder);
}

int count_turns(std::vector<Direction> path) {
  auto c = 0;
  if(path.empty())return 0;
  Direction last_dir = path.front();
  for(auto step:path)
    if(step!=last_dir)
      c++,last_dir=step;
  return c;
}

int count_wiggles(std::vector<Direction> path) {
  int c = 0;
  if(path.empty())return 0;
  Direction last_dir = path.front();
  Direction l2_dir = path.front();
  for(auto dir:path){
    if(dir!=last_dir and last_dir!=l2_dir)
      c++;
    l2_dir=last_dir;
    last_dir=dir;
  }
  return c;
}

int snek_aware_distance(Snek const & game,Point goal){
  auto const walls=game.Body();
  Point pnt=walls[0];
  auto xdist=std::abs(int(pnt.first)-int(goal.first));
  auto ydist=std::abs(int(pnt.second)-int(goal.second));
  int const basic_dist=xdist+ydist;
  if(walls.size()<=3)
    return basic_dist;

  int linear_dist=basic_dist;
  while(pnt!=goal){
    DBG3(pnt);
    if(auto intersect=std::find(walls.begin()+1,walls.end(),pnt);intersect!=walls.end()){
      CLEAR3();
      linear_dist+=std::distance(intersect,walls.end());
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
      x_first_dist+=std::distance(intersect,walls.end());
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
      y_first_dist+=std::distance(intersect,walls.end());
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
  return std::min(std::min(x_first_dist,y_first_dist),linear_dist);
}

int body_to_goal_dist(std::vector<Point> body,Point goal){
  auto dist=distance(body[0],goal,1);
  for(auto segment:body)
    dist=std::min(dist,distance(segment,goal,1));
  return dist;
}

int wall_count(std::vector<Point> const & walls,Point const pnt,Point const border){
  int c=0;
  for (auto dir :
	 {Direction::up, Direction::right, Direction::down, Direction::left}){
    Point t=pnt+dir;
    if(t.first<=0 or t.second<=0 or t.first>border.first or t.second>border.second){
      c++;
      continue;
    }
    if(std::find(walls.begin(),walls.end(),t)!=walls.end())
      c++;
  }
  return c;
}

template<typename T,typename VAL,bool GREATER=true,bool QUEUE=false>
class priority_stack{
  // TODO: assert that GT is int(*)(T);
  VAL val;
  std::list<std::pair<T,int>> list;
public:
  priority_stack(VAL op):val(op){}
  bool empty()const{return list.empty();}
  void push(T elm){
    int v=val(elm);
    if(empty()){
      list.push_front({elm,v});
      return;
    }
    auto comp=[=](int lhs,int rhs){
		if(QUEUE)
		  if(GREATER)
		    return lhs>=rhs;
		  else
		    return lhs<=rhs;
		else
		  if(GREATER)
		    return lhs>rhs;
		  else
		    return lhs<rhs;
	      };
    auto ptr=list.begin();
    if(comp(v,ptr->second)){
      list.push_front({elm,v});
      return;
    }
    while(std::next(ptr)!=list.end() and ((GREATER and std::next(ptr)->second>v)
				       or (!GREATER  and std::next(ptr)->second<v)))
      ++ptr;
    list.insert(ptr,{elm,v});
  }
  T pop(){
    auto ret=list.front().first;
    list.pop_front();
    return ret;
  }
  T operator[](int dep)const{
    if(dep>=list.size())
      throw "";
    auto it=list.begin();
    while(dep--)
      it=std::next(it);
    return it->first;
  }
};

enum SearchStyle{DFS,BFS};
/*
template<SearchStyle SS,typename State,typename Action>
auto mk_AI(int (*(*metric(State)))(std::pair<std::vector<Action>,State>),
	   bool (*(*should_cut(State)))(std::pair<std::vector<Action>,State>),
	   bool (*(*is_done(State)))(std::pair<std::vector<Action>,State>),
	   std::vector<std::pair<std::vector<Action>,State>>
	       (*enumerate)(std::pair<std::vector<Action>,State>)){
*/
template<SearchStyle SS,typename State,typename Action>
auto mk_AI(std::function<
	      std::function<
	         int(std::pair<std::vector<Action>,State>)>
	      (State)> metric,
	   std::function<
	      std::function<
	         bool(std::pair<std::vector<Action>,State>)>
	      (State)> should_cut,
	   std::function<
	      std::function<
	         bool(std::pair<std::vector<Action>,State>)>
	      (State)> is_done,
	   std::function<
	      std::vector<std::pair<std::vector<Action>,State>>
	      (std::pair<std::vector<Action>,State>)>
	      enumerate){
  auto  AI=[=](State const & state)->std::vector<Action>{
        typedef std::vector<Action> Path;
	typedef std::pair<Path,State> Consideration;
	auto cutter=should_cut(state);
	auto success=is_done(state);
	priority_stack<Consideration,std::function<int(Consideration)>,false,SS==BFS>
	    possibilities(metric(state));
	possibilities.push({Path(),state});
	while(!possibilities.empty()){
	  auto trying=possibilities.pop();
	  DBG("- Checking if only one root\n");
	  if(possibilities.empty())
	    if(trying.first!=Path()){
	      DBG("-- Shortcutting\n");
	      // since all must descend from this, lets advance the board
	      return trying.first;
	    }
	  DBG("Enumerating\n");
	  for (auto opt : enumerate(trying)) {
	    DBG("-- Cutting\n");
	    if(cutter(opt))
	      continue;
	    DBG("-- Check if done\n");
	    if(success(opt))
	      return opt.first;
	    DBG("-- Push into possibilities");
	    possibilities.push(opt);
	  }
	}
	DBG("- No Path Found\n");
	return {};
  };
  return AI;
}

std::function<std::vector<Direction>(Snek const&)>Snek_AI(){
  typedef std::vector<Direction> Path;
  typedef std::pair<Path,Snek> Consideration;
  auto metric = [](Snek initial){
		  auto goal=initial.Food();
		  return [=](Consideration con)->int{
			   auto& path=con.first;
			   auto& game=con.second;
			   auto b=game.Body();
			   auto turns=count_turns(path);
			   auto wiggles=count_wiggles(path);
			   auto goal_dist=distance(b[0],goal,1);
			   auto goal_dist_smart=snek_aware_distance(game,goal);
			   auto depth=int(path.size());
			   auto unreachable=unreachable_count(b,b[0],
							      game.Size());
			   if(depth<3)
			     // escape the head
			     return -1;
			   if(goal_dist==goal_dist_smart)
			     return
			       goal_dist+depth
			       +unreachable;
			   // A* with a small weighting
			   return
			     goal_dist
			     +10*(goal_dist+depth)
			     +wiggles+turns
			     +unreachable;
			 };
		};
  auto cutter = [](Snek initial){
		  auto goal=initial.Food();
		  return [=](Consideration con)->bool{
			   auto&game =con.second;
			   auto body=game.Body();// remove copy here?
			   if(!game.Alive())
			     return true;
			   if (!reachable(body,body[0],body.back(),
					  game.Size()))
			     return true;
			   if(body[0]==goal)
			     if(wall_count(body,goal,game.Size())>2
				and distance(goal,body.back(),1)<3)
			       // If eating the food would (probably) cut us off, don't.
			       return true;
			   return false;
			 };
		};
  auto terminal=[](Snek initial){
		  auto goal=initial.Food();
		  return [=](Consideration con)->bool{
			   auto&game=con.second;
			   auto body=game.Body();// remove copy here?
			   if(body[0]==goal)
			     if(!(wall_count(body,goal,game.Size())>2
				  and distance(goal,body.back(),1)<3))
			       return true;
			   return false;
			 };
		};
  auto enumerator=[](Consideration con)->std::vector<Consideration>{
    std::vector<Consideration> ret;
    for (auto dir :
       {Direction::up, Direction::right, Direction::down, Direction::left}){
          Path p(con.first);
	  Snek g(con.second);
	  p.push_back(dir);
	  g.move(dir);
          ret.push_back({p,g});
    }
    return ret;
  };

  return mk_AI<BFS,Snek,Direction>(metric,cutter,terminal,enumerator);
}
