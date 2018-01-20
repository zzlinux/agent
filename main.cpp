#include <iostream>
#include "thread/ThreadController.h"
using namespace hitcrt;
int main() {
    std::cout << "Hello, World!" << std::endl;
    ThreadController thread;
    thread.init();
    thread.run();
    return 0;
}