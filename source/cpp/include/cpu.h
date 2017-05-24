#ifndef INCLUDE_CPU_H
#define INCLUDE_CPU_H

namespace jones {

  class Cpu {
  public:

    Cpu(const void* baseAddr);

    void reset();

    void run();
    
    void* getProgramCounter();
  };
}

#endif // INCLUDE_CPU_H
