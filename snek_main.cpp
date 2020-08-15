#include "snek_game.hpp"
#include <optional>
#include <queue>
#include <fstream>
#include <cmath>
#include <map>
#include <algorithm>

#define DEBUG1 0
#if DEBUG1
std::ofstream log;
#define DBG(X) do{log<<X<<std::flush;}while(0)
#else
#define DBG(X) do{}while(0)
#endif

#define DEBUG2 1
#if DEBUG2
#include "Console-IO/ioconsole.hpp"
std::queue<Point> dots;
#define DBG2(X) do{					\
    if(udh::cio[(X).first][(X).second]==' '){		\
      dots.push(X);					\
      udh::cio[(X).first][(X).second]='.';		\
      udh::cio<<std::flush;				\
    }							\
  }while(0)
#define CLEAR() do{					\
    while(dots.size()){					\
      auto d=dots.front();				\
      dots.pop();					\
      udh::cio[d.first][d.second]=' ';	\
      udh::cio<<std::flush;				\
    }							\
  }while(0)
#else
#define DBG2(X) do{}while(0)
#define CLEAR() do{}while(0)
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
  auto quick_explore = val.first.size()<3;
  auto to_many_turns = (1+turns>1+(turns>3)*turns*(1+(turns>5)*turns)*(1+(turns>7)*turns));
  auto is_close = (md<3) && (contention[val.second.Body()[0]]<3);
  auto contentiousness = pow(contention[val.second.Body()[0]],2);

  auto distance_cost=heuristic_distance+current_distance;
  auto dislikability=to_many_turns+(1+!quick_explore)+contentiousness+(1+!is_close);

  return distance_cost*dislikability;
  
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+(1+turns>1+(turns>3)*turns*(1+(turns>5)*turns)*(1+(turns>7)*turns))*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*((md>2) && (contention[val.second.Body()[0]]<3));
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+(1+turns>1+(turns>3)*turns*(1+(turns>5)*turns)*(1+(turns>7)*turns))*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*((md>2) && (contention[val.second.Body()[0]]<3));
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+(1+turns>1+(turns>3)*turns*(1+(turns>5)*turns))*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*((md>2) && (contention[val.second.Body()[0]]<3));
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+(1+turns>1+turns>3*turns*(1+turns>5*turns))*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*(md>2 && contention[val.second.Body()[0]]<3);
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+(1+turns>1+turns>3*turns)*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*(md>2 && contention[val.second.Body()[0]]<3);
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+(1+turns>3*4)*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*(md>2 && contention[val.second.Body()[0]]<3);
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+count_turns(val.first)*pow(contention[val.second.Body()[0]],2)*(md>5)+val.first.size()*(md>2 && contention[val.second.Body()[0]]<3);
  return (md2*(1+(md>5)*4)*(val.first.size()>2))+count_turns(val.first)*contention[val.second.Body()[0]]*(md>5)+val.first.size()*(md>2 && contention[val.second.Body()[0]]<3 );
  return (md2*(1+(md2>5)*4)*(val.first.size()>2))+count_turns(val.first)*contention[val.second.Body()[0]]*(md2>5)+val.first.size()*(md2>2 && contention[val.second.Body()[0]]<3 );
  return (md2*(1+(md2>5)*4)*(val.first.size()>2))+count_turns(val.first)*contention[val.second.Body()[0]]*(md2>5)+val.first.size();
  return (md2*(1+(md2>5)*4)*(val.first.size()>2))+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]]*(md2>5)+val.first.size();
  return (md2*(1+(md2>5)*4)*(val.first.size()>2))+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]];
  return (md*(1+(md>5)*4))+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]];
  return (md+(md>5)*4)+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]];
  return (metric_distance(val.second)+3)+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]];
  return std::max<int>(metric_distance(val.second),
		       std::max(count_turns(val.first)-3,1
				)*val.first.size())
    *contention[val.second.Body()[0]];
  return std::max<int>(metric_distance(val.second),count_turns(val.first)*val.first.size())*contention[val.second.Body()[0]];
  return std::min<int>(metric_distance(val.second)*2,count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]]);
  return metric_distance(val.second)*2+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]];
  return metric_distance(val.second)+count_turns(val.first)*val.first.size()*contention[val.second.Body()[0]];
  return metric_distance(val.second)+count_turns(val.first)*val.first.size();
  return metric_distance(val.second)+count_turns(val.first)*contention[val.second.Body()[0]];
  return metric_distance(val.second)+val.first.size()*contention[val.second.Body()[0]];
  return metric_distance(val.second)+val.first.size()+contention[val.second.Body()[0]];
  return pow(metric_distance(val.second),1)*2+pow(val.first.size()/8,2);
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
