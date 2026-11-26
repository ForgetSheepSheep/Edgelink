#include "boot_status.h"
#include "dev_lcd/dev_lcd.h"

#define BS_COLOR_BG     (0x0000U)
#define BS_COLOR_TITLE  (0xFFFFU)
#define BS_COLOR_TEXT   (0xE73FU)
#define BS_COLOR_MUTED  (0x9DBBU)
#define BS_COLOR_BORDER (0x1A0FU)
#define BS_COLOR_GREEN  (0x07B4U)

static uint8_t s_bs_ready = 0U;

void boot_status_init(void)
{
    ret_t ret;

    ret = dev_lcd_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_Warning("boot status: LCD bus init failed ret=%ld", (long)ret);
        return;
    }

    ret = dev_lcd_init_controller();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_Warning("boot status: LCD controller init failed ret=%ld",
                    (long)ret);
        return;
    }

    s_bs_ready = 1U;
    (void)dev_lcd_clear(BS_COLOR_BG);
}

uint8_t boot_status_ready(void)
{
    return s_bs_ready;
}

void boot_status_show(const char *title,
                      const char *line1,
                      const char *line2)
{
    if (s_bs_ready == 0U)
    {
        return;
    }

    (void)dev_lcd_clear(BS_COLOR_BG);

    if (title != NULL)
    {
        (void)dev_lcd_draw_text_scaled(20U, 70U, title, 2U,
                                       BS_COLOR_TITLE, BS_COLOR_BG);
    }
    if (line1 != NULL)
    {
        (void)dev_lcd_draw_text(20U, 150U, line1,
                                BS_COLOR_TEXT, BS_COLOR_BG);
    }
    if (line2 != NULL)
    {
        (void)dev_lcd_draw_text(20U, 180U, line2,
                                BS_COLOR_MUTED, BS_COLOR_BG);
    }

    (void)dev_lcd_draw_text(20U, 440U, "EDGELINK BOOTLOADER",
                            BS_COLOR_MUTED, BS_COLOR_BG);
}

void boot_status_progress(uint32_t percent, const char *label)
{
    char percent_text[8U];
    int length;
    uint16_t fill_width;

    if (s_bs_ready == 0U)
    {
        return;
    }
    if (percent > 100U)
    {
        percent = 100U;
    }

    length = snprintf(percent_text, sizeof(percent_text),
                      "%lu%%", (unsigned long)percent);
    if ((length < 0) || ((uint32_t)length >= sizeof(percent_text)))
    {
        return;
    }

    if (label != NULL)
    {
        (void)dev_lcd_draw_text(20U, 230U, label,
                                BS_COLOR_TEXT, BS_COLOR_BG);
    }
    (void)dev_lcd_draw_text(270U, 230U, percent_text,
                            BS_COLOR_TEXT, BS_COLOR_BG);

    (void)dev_lcd_fill_rect(20U, 260U, 300U, 282U, BS_COLOR_BORDER);
    fill_width = (uint16_t)((276UL * percent) / 100UL);
    if (fill_width > 0U)
    {
        (void)dev_lcd_fill_rect(22U, 262U,
                                (uint16_t)(22U + fill_width - 1U), 280U,
                                BS_COLOR_GREEN);
    }
}
