#include "snek_game.hpp"
#include "snek_ai.hpp"

#define VISUAL 1

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

#if VISUAL
#include "Console-IO/ioconsole.hpp"

void visual_simulator() {
#if DEBUG1
  log_file = std::ofstream("trying.log", std::ios::out | std::ios::trunc);
#endif
  DBG("Starting Visual Simulator\n");
  Snek s(udh::cio.size());
  s.drawWalls(udh::cio);
  s.updateDisplay(udh::cio);
  DBG("Call to AI\n");
  auto p = AI(s);
  DBG("Got a path of length " << p.size() << "\n");
  while (s.Alive()) {
    if (!p.size()){
      DBG("\n\nNext Food\n");
      p = AI(s);
      DBG("Moving\n");
      DBG("Current fill = "<<s.Body().size()<<"/"<<s.Size().first*s.Size().second<< "\n");
    }
    if(!p.size()){
      DBG("No Path\n");
      break;
    }
    s.move(p.front());
    p.erase(p.begin(),p.begin()+1); // pop front
    s.updateDisplay(udh::cio);
  }
}

#else

enum Termination {Success,Crash,Death,Giveup};

std::pair<Termination,int> size_simulator_single(Point size) {
#if DEBUG1
  log_file = std::ofstream("trying.log", std::ios::out | std::ios::trunc);
#endif
  DBG("Starting Size Simulator ("<<size.first<<","<<size.second<<")\n");
  Snek s(size);
  DBG("Call to AI\n");
  auto p = AI(s);
  DBG("Got a path of length " << p.size() << "\n");
  try{
  while (s.Alive()) {
    if (!p.size()){
      DBG("\n\nNext Food\n");
      p = AI(s);
      DBG("Moving\n");
      DBG("Current fill = "<<s.Body().size()<<"/"<<s.Size().first*s.Size().second<< "\n");
    }
    if(!p.size()){
      DBG("No Path\n");
      break;
    }
    s.move(p.front());
    p.erase(p.begin(),p.begin()+1); // pop front
  }
  if(!s.Alive())
    return {Death,s.Body().size()};
  if(s.Body().size()==((size.first*size.second)&~1))
    // The last square isn't fillable in an odd grid.
    return {Success,s.Body().size()};
  else
    return {Giveup,s.Body().size()};
  }catch(std::exception e){
    return {Crash,s.Body().size()};
  }
}

#include <iostream>

void size_simulator(Point max_size){
    for(int x=2;x<max_size.first;x++)
        for(int y=x;y<max_size.second;y++){
            auto res=size_simulator_single({x,y});
            std::cout<<"("<<x<<","<<y<<") = {"<<res.first<<","<<res.second<<"}"<<std::endl;
        }
}

#endif

int main(){
#if VISUAL
  visual_simulator();
#else
  size_simulator({7,7});
#endif
  return 0;
}
