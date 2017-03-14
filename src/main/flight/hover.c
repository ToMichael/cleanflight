#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "hover.h"

void rxHover(void){
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    
}
