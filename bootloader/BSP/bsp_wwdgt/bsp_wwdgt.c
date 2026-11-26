/** @file bsp_wwdgt.c */
#include "bsp_wwdgt.h"
#include "gd32f30x.h"
#include "gd32f30x_wwdgt.h"

ret_t bsp_wwdgt_init(void)
{
    rcu_periph_clock_enable(RCU_WWDGT);
    wwdgt_config(0x7FU, 0x5FU, WWDGT_CFG_PSC_DIV8);
    wwdgt_enable();
    return RET_OK;
}

ret_t bsp_wwdgt_feed(void)
{
    wwdgt_counter_update(0x7FU);
    return RET_OK;
}
