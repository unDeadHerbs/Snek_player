#include "snek_game.hpp"
#include <optional>
#include <queue>
#include <fstream>

std::ofstream log;
#define DBG(X) do{log<<X<<std::flush;}while(0)


using Direction = Snek::Direction;

bool tail_reachable(Snek const& s){
  return true; // TODO
}

int metric_distance(Snek const & s){
  if(!tail_reachable(s))return 10000; // a big number
  return distance(s.Body()[0],s.Food());// distance to food
}

typedef std::queue<Direction> Path;
Path Astar(Snek s){
  DBG("- In Astar\n");
  typedef std::pair<Path,Snek> Consideration;
  auto comp=[](Consideration const & lhs,Consideration const & rhs){
	      return (metric_distance(lhs.second)*2+lhs.first.size())
		    >(metric_distance(rhs.second)*2+rhs.first.size());
	    };
  std::priority_queue<Consideration,std::vector<Consideration>,decltype(comp)> possibilities(comp);
  possibilities.push({{},s});
  // check if would kill
  DBG("- Made empty Tries\n");
  for(;;){
    DBG("-- have "<<possibilities.size()<<" options\n");
    auto current=possibilities.top();
    DBG("-- Top has "<<current.first.size()<<" steps\n");
    DBG("-- Top has "<<metric_distance(current.second)<<" to go\n");
    for(auto dir:{Direction::up,Direction::right,Direction::down,Direction::left}){
      //DBG("--- Trying option "<<dir<<"\n");
      Snek ss(current.second);
      ss.move(dir);
      if(ss.Alive()){
	//DBG("---- Adding Option\n");
	auto p=current.first;
	p.push(dir);
	DBG("---- Adding Path Length "<<p.size()<<"\n");
	if(ss.Body()[0]==s.Food()){
	  DBG("---- Found\n");
	  return p;
	}
	possibilities.push({p,ss});
      }
    }
  }
    
  //if(!ss.Alive())return {};
  // return happiness?
  // return 1;
}

int main() {
  log = std::ofstream("trying.log",std::ios::out|std::ios::trunc);
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
