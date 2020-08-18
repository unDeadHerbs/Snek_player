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

#define DEBUG2 0
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
  return border.first*border.second - walls.size();
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
  int dist=distance(body[0],goal,1);
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

template<typename T,typename VAL,bool GREATER=true>
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
    auto ptr=list.begin();
    if((!GREATER and v<ptr->second) or
       (GREATER and v>ptr->second)){
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

struct Consideration{
  Path path;
  Snek game;
};

Path AI(Snek const & game){
  auto food=game.Food();
  CLEAR();
  auto metric=[](Point goal){
		return [=](Consideration con)->int{
			 auto b=con.game.Body();
			 auto turns=count_turns(con.path);
			 auto goal_dist=snek_aware_distance(con.game,goal);
			 auto depth=con.path.size();

			 if(depth<3)
			   // escape the head
			   return -200000;
			 // just A* for a moment
			 return 101*goal_dist+100*depth+105*turns;
		       };
	      };
  priority_stack<Consideration,decltype(metric(food)),false> possibilities(metric(food));
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
      Path p(trying.path);
      t_game.move(dir);
      p.push_back(dir);
      DBG("-- check alive point "<<dir<<"\n");
      if(!t_game.Alive())
	continue;
      DBG("-- check reachable point "<<dir<<"\n");
      if(true){//if(t_game.Body()[0]==food){
	// Two moves required to prevent cutting at food (where
	// everywhere is reachable but no move keeps that so).
	bool safe_dir=false;
	for (auto test_dir :
	       {Direction::up, Direction::right,
		  Direction::down, Direction::left})
	  for (auto test_dir2 :
		 {Direction::up, Direction::right,
		    Direction::down, Direction::left}){
	    Snek b_game(t_game);
	    b_game.move(test_dir);
	    if(b_game.Alive())
	      if(0==unreachable_count(b_game.Body(),
				      b_game.Body().front(),
				      b_game.Size())){
		b_game.move(test_dir2);
		if(b_game.Alive())
		  if(0==unreachable_count(b_game.Body(),
					  b_game.Body().front(),
					  b_game.Size())){
		    safe_dir=true;
		    break;
		  }
	      }
	    if(safe_dir)
	      break;
	  }
	if(!safe_dir)
	  continue;
      }

      if(t_game.Body()[0]==food)
	return p;
      /*
      if (!reachable(t_game.Body(),t_game.Body().front(),t_game.Body().back(),t_game.Size()))
	continue;
      DBG("-- check not-closed in point "<<dir<<"\n");
      if(t_game.Body()[0]==food)
	if(wall_count(t_game.Body(),food,t_game.Size())>2 and distance(food,t_game.Body().back(),1)<3)
	  // If eating the food would (probably) cut us off, don't.
	  continue;
	else
	  return p;
      */
      DBG("-- Adding consider point "<<dir<<"\n");
      DBG2(t_game.Body()[0],'.');
      possibilities.push({p,t_game});
    }
    DBG2(trying.game.Body()[0],',');
  }
  DBG("No Path Found\n");
  return {};
}
