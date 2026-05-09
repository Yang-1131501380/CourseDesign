#include <rthw.h>
#include <rtthread.h>

#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "uart_multi.h"
#include "usart.h"

extern void SystemClock_Config(void);

void SysTick_Handler(void)
{
    rt_interrupt_enter();
    rt_tick_increase();
    rt_interrupt_leave();
}

void rt_hw_board_init(void)
{
    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / RT_TICK_PER_SECOND);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0x0F, 0);

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

#ifdef RT_USING_CONSOLE
void rt_hw_console_output(const char *str)
{
    static const rt_tick_t timeout = RT_TICK_PER_SECOND / 10;
    const char *p = str;
    rt_tick_t start = rt_tick_get();

    while (*p != '\0')
    {
        char ch = *p++;

        if (ch == '\n')
        {
            while (UartMulti_Write(UART_MULTI_CH1, (const rt_uint8_t *)"\r", 1) != HAL_OK)
            {
                if ((rt_tick_get() - start) > timeout)
                {
                    return;
                }
            }
        }

        while (UartMulti_Write(UART_MULTI_CH1, (const rt_uint8_t *)&ch, 1) != HAL_OK)
        {
            if ((rt_tick_get() - start) > timeout)
            {
                return;
            }
        }
    }
}
#endif
