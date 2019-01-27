#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

#include <Chimera/chimera.hpp>
#include <Chimera/config.hpp>
#include <Chimera/threading.hpp>
#include <Chimera/utilities.hpp>

using namespace Chimera::Threading;


int main(void)
{
    ChimeraInit();

    #ifdef DEBUG
    //InitializeSamplingProfiler();
    InitializeInstrumentingProfiler();
    #endif

    startScheduler();

    /* Should never reach here as scheduler should be running */
    for(;;)
    {

    }
}


