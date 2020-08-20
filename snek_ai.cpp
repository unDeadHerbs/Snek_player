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


/*
 * While the snake is short do something else that works.  Once the
 * snake is a little longer (more than 2 times the longer side
 * perhaps) make a storage of zig-zags at the far left end of the
 * space, each time the snake leaves the space it will collect apples
 * moving rightwards in the space (with some degree of leftward motion
 * allowed for easy apples).  Then when it gets to the right end of
 * the space it will move along the end wall to the top, move along to
 * the left wall (grabbing apples that are directly down/up along the
 * way).  Once at the left wall the snake will make a stack of
 * zig-zags to re-capture the tail.  The depth of this stack will be
 * until the tail has reached the right wall, so that it will be out
 * of the way for the next pass.  This may need modifying if the
 * down/up grabs are in the way or not.  It may also be plausible for
 * the snake to leave the stack while the tail is only half way to the
 * right wall if it will reach the right wall by the time the head
 * reaches it.  This will require either some probabilities or some
 * stats to tune.
 */
//std::function<std::vector<Direction>(Snek const&)>Snek_AI_zags_storage(){}

/*
 * This is a very fast to calculate and simple metric.  Similar to how
 * one can solve most mazes by following the right wall, this snake
 * assumes that its body is a maze and follows the right wall.  There
 * are two exceptions to this: 1, if the apple is within one turn the
 * snake will go take it; and 2, if a move would trap the snake (put
 * its head in a room without its tail) then the snake won't take it.
 * Since the entire path to the next apple is calculated before moving
 * this second rule means that a small tree is explored.
 *
 * TODO: Make a variant of this that builds small zigzag piles in square-
 * ish free spaces.  This will reduce the size of and help destroy the
 * structures that this algorithm tends to make, which are slow to navigate.
 * (I'm not sure that the zags will be faster than the structures, since they
 * will also need navigating, but I'm hoping that they will be avoidable and can
 * be ignored/rebuilt more frequently, making them more ephemeral and not needing
 * of avoidance.  It may be the case that the interlocking structures decrease
 * the average time to the next apple.)
 */
std::function<std::vector<Direction>(Snek const&)>Snek_AI_go_right(){
  typedef std::vector<Direction> Path;
  typedef std::pair<Path,Snek> Consideration;
  auto metric = [](Snek initial){
		  auto goal=initial.Food();
		  return [=](Consideration con)->int{
			   auto& path=con.first;
			   auto& game=con.second;
			   auto b=game.Body();
			   auto goal_dist=distance(b[0],goal,1);
			   auto goal_dist_smart=snek_aware_distance(game,goal);
			   auto depth=int(path.size());
			   // If a path exists, take it.
			   if(goal_dist==goal_dist_smart)
			     return
			       goal_dist+depth;
			   // Else take the path as enumerated.
			   return int(game.Size().first*game.Size().second)+
			     depth/int(game.Size().first*game.Size().second);
			   // If the path is longer than the map, check for alternatives.
			 };
		};
  auto cutter = [](Snek){
		  return [](Consideration con)->bool{
			   auto&game =con.second;
			   auto body=game.Body();// remove copy here?
			   if(!game.Alive())
			     return true;
			   if (!reachable(body,body[0],body.back(),
					  game.Size()))
			     if(body.size()<game.Size().first*game.Size().second-1)
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
			     return true;
			   return false;
			 };
		};
  auto enumerator=[](Consideration con)->std::vector<Consideration>{
    std::vector<Consideration> ret;
    if(con.first.size()==0){
      for (auto dir :
	     {Direction::up, Direction::right, Direction::down, Direction::left}){
	Path p(con.first);
	Snek g(con.second);
	p.push_back(dir);
	g.move(dir);
	ret.push_back({p,g});
      }
    }else switch(con.first.back()){
      case Direction::none:
      case Direction::up:
	for (auto dir :
	       {Direction::left, Direction::up,Direction::right}){
	  Path p(con.first);
	  Snek g(con.second);
	  p.push_back(dir);
	  g.move(dir);
	  ret.push_back({p,g});
	}
	break;
      case Direction::right:
	for (auto dir :
	       {Direction::up, Direction::right, Direction::down}){
	  Path p(con.first);
	  Snek g(con.second);
	  p.push_back(dir);
	  g.move(dir);
	  ret.push_back({p,g});
	}
	break;
      case Direction::down:
	for (auto dir :
	       {Direction::right, Direction::down, Direction::left}){
	  Path p(con.first);
	  Snek g(con.second);
	  p.push_back(dir);
	  g.move(dir);
	  ret.push_back({p,g});
	}
	break;
      case Direction::left:
	for (auto dir :
	       {Direction::down, Direction::left, Direction::up}){
	  Path p(con.first);
	  Snek g(con.second);
	  p.push_back(dir);
	  g.move(dir);
	  ret.push_back({p,g});
	}
	break;
      }
    return ret;
  };

  return mk_AI<DFS,Snek,Direction>(metric,cutter,terminal,enumerator);
}

std::function<std::vector<Direction>(Snek const&)>Snek_AI_no_pockets(){
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

/*
std::function<std::vector<Direction>(Snek const&)> Snek_AI_modalish(){
  typedef std::vector<Direction> Path;
  typedef std::pair<Path,Snek> Consideration;
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
			   return -500+tail_dist+depth*10/b.size();
			 if(body_to_goal_dist(b,goal)<goal_dist)
			   // follow tail
			   return -500+tail_dist-depth;

			 int follow_tail_if_near      =  -80 * ((tail_dist<5)
								* !is_simple_path * (b.size()>goal_dist));
			 int distance_guess           =   40 * (goal_dist+depth);
			 int less_distance_if_wiggles =  -30 * goal_dist*(turns>10)*(turns<b.size()/4);
			 int follow_walls             =   30 * wall_count(b,b[0],con.game.Size());
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
      if (!reachable(t_game.Body(),t_game.Body().front(),t_game.Body().back(),t_game.Size()))
	continue;
      DBG("-- check not-closed in point "<<dir<<"\n");
      if(t_game.Body()[0]==food)
	if(wall_count(t_game.Body(),food,t_game.Size())>2 and distance(food,t_game.Body().back(),1)<3)
	  // If eating the food would (probably) cut us off, don't.
	  continue;
	else
	  return p;
      DBG("-- Adding consider point "<<dir<<"\n");
      DBG2(t_game.Body()[0],'.');
      possibilities.push({p,t_game});
    }
    DBG2(trying.game.Body()[0],',');
  }
  DBG("No Path Found\n");
  return {};
}
*/
