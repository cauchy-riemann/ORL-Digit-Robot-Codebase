#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <cstdlib>
#include <limits>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


int main ()
  {
       for (int i = 0; i < 100; ++i)
         {
           std::fstream ofs("test.txt", std::fstream::trunc);
           ofs  << i;
           ofs.close();
         }
       return 0;
  }
