#include "snek_game.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <deque>
#include <list>
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
      auto tail_dist=std::distance(intersect,walls.end());
      //auto head_dist=std::distance(walls.begin(),intersect);
      auto wait_dist=tail_dist;
      //if(head_dist>3)
      //wait_dist=std::min(tail_dist,head_dist);
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
      auto tail_dist=std::distance(intersect,walls.end());
      //auto head_dist=std::distance(walls.begin(),intersect);
      auto wait_dist=tail_dist;
      //if(head_dist>3)
      //wait_dist=std::min(tail_dist,head_dist);
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
      auto tail_dist=std::distance(intersect,walls.end());
      //auto head_dist=std::distance(walls.begin(),intersect);
      auto wait_dist=tail_dist;
      //if(head_dist>3)
      //wait_dist=std::min(tail_dist,head_dist);
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

int body_to_goal_dist(std::vector<Point> body,Point goal){
  int dist=distance(body[0],goal,1);
  for(auto segment:body)
    dist=std::min(dist,distance(segment,goal,1));
  return dist;
}

int wall_count(Snek const & game){
  int c=0;
  for (auto dir :
	 {Direction::up, Direction::right, Direction::down, Direction::left}){
    auto g=game;
    g.move(dir);
    if(!g.Alive())
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
  //auto init_back=game.Body().back();
  auto food=game.Food();
  CLEAR();
  auto metric=[](Point goal){
		return [=](Consideration con)->int{
			 auto b=con.game.Body();
			 auto turns=count_turns(con.path);
			 auto goal_dist=snek_aware_distance(con.game,goal);
			 auto goal_lin_dist=distance(b[0],goal,1);
			 auto tail_dist=snek_aware_distance(con.game,b.back());
			 auto tail_lin_dist=distance(b[0],b.back(),1);
			 auto depth=con.path.size();
			 auto is_simple_path=(goal_dist==goal_lin_dist);

			 if(depth<3)
			   // escape the head
			   return -200000+depth;
			 if(is_simple_path)
			   // take simple path with small preference to developed paths
			   return -100000+10.1*goal_dist+10*depth;
			 // TODO: (-1)*size.first * size.second?
			 if(!reachable(b,b[0],goal,con.game.Size()))
			   // follow tail until better
			   return -500+tail_dist-depth;
			 if(body_to_goal_dist(b,goal)<goal_dist)
			   // follow tail
			   return -500+tail_dist-depth;

			 int follow_tail_if_near      =  -80 * ((tail_dist<5)
								* !is_simple_path * (b.size()>goal_dist));
			 int distance_guess           =   40 * (goal_dist+depth);
			 int less_distance_if_wiggles =  -30 * goal_dist*(turns>10)*(turns<b.size()/4);
			 int follow_walls             =   30 * wall_count(con.game);
			 // TODO: Make follow_walls a property of the path not the head.
			 // int avoid_small_bubbles   =   20 * bubble_count(con.game);
			 int unprefer_undeveloped     =   15 * goal_dist;
			 int prefer_developed_paths   =  -10 * depth;
			 int prefer_lots_of_turns     =  -15 * turns*(turns>5);
			 int prefer_less_turns        =   10 * (turns/(depth/20+1)
								+b[0].first!=goal.first+b[0].second!=goal.second);
			 int when_lost_find_tail      =   -1 * (tail_lin_dist+tail_dist);
			 int developed_simple_path    =    1 * goal_dist * (goal_dist==goal_lin_dist);

			 return
			   distance_guess+prefer_less_turns+when_lost_find_tail
			   +prefer_developed_paths +follow_walls+prefer_lots_of_turns
			   +less_distance_if_wiggles+follow_tail_if_near+unprefer_undeveloped
			   +developed_simple_path;
		       };
	      };
  priority_stack<Consideration,decltype(metric(food)),false> possibilities(metric(food));
  possibilities.push({{},game});
#if 0
  priority_stack<Consideration,decltype(metric(init_back)),false> possibilities_tail(metric(init_back));
  possibilities_tail.push({{},game});
  DBG("-- Seed with path to tail\n");
  while(!possibilities_tail.empty()){
    auto trying=possibilities_tail.pop();
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
      if(t_game.Body()[0]==food)
	return p; // Just in case.
      if(t_game.Body()[0]==init_back)
	goto search;
      DBG("-- Adding consider point "<<dir<<"\n");
      DBG2(t_game.Body()[0],'.');
      possibilities.push({p,t_game});
      possibilities_tail.push({p,t_game});
    }
    DBG("- Adding considered point\n");
    DBG2(trying.game.Body()[0],',');
  }
 search:
  DBG("-- Starting search Proper\n");
#endif
  while(!possibilities.empty()){
    /*
    // If top 20 ideas all have the same start, use that.
    Path init_check(possibilities[0].path);
    if(possibilities.size()>20){
      for(int dep=0;dep<20
		     and !init_check.empty();++dep)
	init_check=shared_front(init_check,
				possibilities[dep].path);
      if(!init_check.empty())
	;//return init_check;
    }
    */

    auto trying=possibilities.pop();
    if(possibilities.empty())
      if(trying.path.size()){ // since all must descend from this, lets advance the board
	DBG("-- Short Cutting\n");
	return trying.path;
      }
    if(!possibilities.empty())
    if(trying.path.size()>5+possibilities[0].path.size()
       and trying.path.size()>20){
      Path ret;
      for(int p=0;p<20;p++)
	ret.push_back(trying.path[p]);
      return ret;
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
      if(t_game.Body()[0]==food)
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
