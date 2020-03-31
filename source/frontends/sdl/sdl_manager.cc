//
// MIT License
//
// Copyright 2019-2020
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include <SDL2/SDL.h>
#include <boost/core/ignore_unused.hpp>
#include <cstdint>

#include "log.hh"
#include "sdl_manager.hh"

using namespace jones::sdl;

namespace {

void sdl_push_quit_event() {
  SDL_Event event;
  event.type = SDL_QUIT;
  SDL_PushEvent(&event);
}

void sdl_shutdown() {
  SDL_PumpEvents();
  SDL_FlushEvents(SDL_USEREVENT, SDL_LASTEVENT);
  SDL_Quit();
}

} // namespace

sdl_manager::~sdl_manager() {
  uninitialize();
}

void sdl_manager::initialize() {
  if (is_running_) {
    LOG_DEBUG << "screen is already being shown";
    return;
  }
  for (auto const component : components_) {
    component->initialize();
  }
  running_thread_ = std::make_unique<std::thread>([this]() {
    is_running_ = true;
    while (is_running_) {
      process_events();
    }
  });
}

void sdl_manager::uninitialize() {
  is_running_ = false;
  sdl_push_quit_event();
  if (running_thread_ != nullptr) {
    running_thread_->join();
    running_thread_ = nullptr;
  }
  for (auto component : components_) {
    component->uninitialize();
  }
  components_.clear();
  sdl_shutdown();
}

void sdl_manager::process_events() {
  SDL_Event events;
  while (SDL_PollEvent(&events)) {
    for (auto const component : components_) {
      component->on_event(events);
    }
    if (events.type == SDL_QUIT) {
      is_running_ = false;
      break;
    }
  }
}

auto sdl_manager::add_component(sdl_component *const listener) -> void {
  components_.push_back(listener);
}

auto sdl_manager::remove_component(sdl_component *const listener) -> void {
  (void)std::remove(components_.begin(), components_.end(), listener);
}
