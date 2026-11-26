/** @file bsp_fwdgt.c */
#include "bsp_fwdgt.h"
#include "gd32f30x_fwdgt.h"

ret_t bsp_fwdgt_init(void)
{
    fwdgt_write_enable();
    fwdgt_config(3124U, FWDGT_PSC_DIV128);
    fwdgt_counter_reload();
    fwdgt_enable();
    return RET_OK;
}

ret_t bsp_fwdgt_feed(void)
{
    fwdgt_counter_reload();
    return RET_OK;
}
