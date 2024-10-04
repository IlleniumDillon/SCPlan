#include <iostream>
#include <sys/unistd.h>

#include "uvEmbMaster.h"

int main() 
{
    UvEmbMaster uvEmbMaster;
    char line[64] = {0};
    for (int i = 0; i < 10; i ++)
    {
        if (uvEmbMaster.read_regCh(line, 64) > 0)
        {
            std::cout << "Read: " << line << std::endl;
        }
        sleep(1);
    }

    return 0;
}