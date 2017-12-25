#pragma once

#include <SDL.h>

namespace jones {
  
  class Screen {
  public:
    Screen();
    
    void initialize();
    void release();
    void showMain();

  private:
    void processEvent();
    void renderScreen();

    SDL_Event mEvents;
    bool mIsRunning;
  };
}
