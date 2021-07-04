#include "ota-bootload-ncp.h"

int main()
{
    printf("start app \n");
    if(emberAfOtaBootloadCallback())
    {
        //printf("restart ncp \n");
        //exitNcp();
    }
    return 1;
}

