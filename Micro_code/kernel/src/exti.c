#include <exti.h>
#include <encoder.h>
#include <gpio.h>
#include <printk.h>
#include <rcc.h>
#include <nvic.h>
#include <unistd.h>
#include <stdint.h>
#include <motor_driver.h>

/** @brief EXTI register map. */
struct exti
{
    volatile uint32_t imr;   /**< 00 Interrupt Mask Register */
    volatile uint32_t emr;   /**< 04 Event Mask Register */
    volatile uint32_t rtsr;  /**< 08 Rising trigger Selection */
    volatile uint32_t ftsr;  /**< 0C Falling trigger Selection */
    volatile uint32_t swier; /**< 10 Software Interrupt Event Register */
    volatile uint32_t pr;    /**< 14 Pending Register */
};

/** @brief System Config register map. */
struct syscfg
{
    volatile uint32_t memrmp;  /**< 00 Memory Remap */
    volatile uint32_t pmc;     /**< 04 Peripheral mode configuration */
    volatile uint32_t exti[4]; /**< 08-14 External interrupt configuration */
    volatile uint32_t cmpcr;   /**< 20 Compensation cell control */
};

#define EXTI_BASE ((struct exti *)0x40013C00)
#define SYSCFG_BASE ((struct syscfg *)0x40013800)

#define BITS_PER_EXTI 4

#define RCC_APB2_SYSCFG_EN (1 << 14)

#define EXTI0_IRQ        6
#define EXTI1_IRQ        7
#define EXTI2_IRQ        8
#define EXTI3_IRQ        9
#define EXTI4_IRQ        10
#define EXTI9_5_IRQ      23
#define EXTI15_10_IRQ    40

uint32_t btn1_status = 0, btn2_status = 0;

void enable_exti(gpio_port port, uint32_t channel, uint32_t edge)
{
    //enable sys config
    struct rcc_reg_map *rcc = RCC_BASE;
    rcc->apb2_enr |= RCC_APB2_SYSCFG_EN;

    //port mapping
    uint32_t idx = channel / 4;
    uint32_t shift = (channel % 4) * 4;

    SYSCFG_BASE->exti[idx] &= ~(0xF << shift);   // clear
    SYSCFG_BASE->exti[idx] |=  (port << shift);  // set port mapping

    //edge logic
    if (edge == RISING_EDGE || edge == RISING_FALLING_EDGE)
        EXTI_BASE->rtsr |= (1 << channel);
    else
        EXTI_BASE->rtsr &= ~(1 << channel);

    if (edge == FALLING_EDGE || edge == RISING_FALLING_EDGE)
        EXTI_BASE->ftsr |= (1 << channel);
    else
        EXTI_BASE->ftsr &= ~(1 << channel);

    //unmask interrupts
    EXTI_BASE->imr |= (1 << channel);

    //enable nvic irq
    uint8_t irq = exti_channel_to_irq(channel);
    nvic_irq(irq, IRQ_ENABLE);
}

void disable_exti(uint32_t channel)
{
    EXTI_BASE->imr &= ~(1 << channel);
}

void exti_clear_pending_bit(uint32_t channel)
{
    EXTI_BASE->pr |= (1 << channel);
}

// Note: Add your IRQ handlers here

uint8_t exti_channel_to_irq(uint32_t channel)
{
    if (channel <= 4)
        return EXTI0_IRQ + channel;     // EXTI0,1,2,3,4 all separate

    if (channel <= 9)
        return EXTI9_5_IRQ;        // EXTI5..9 share one IRQ

    return EXTI15_10_IRQ;          // EXTI10..15 share one IRQ
}

//0-4
void EXTI0_IRQHandler(void)
{
    if (EXTI_BASE->pr & (1 << 0)) {
        exti_clear_pending_bit(0);
        encoder_irq_handler_left();
    }
}

void EXTI1_IRQHandler(void)
{
    if (EXTI_BASE->pr & (1 << 1)) {
        exti_clear_pending_bit(1);
        encoder_irq_handler_right();
    }
}

void EXTI2_IRQHandler(void)
{
    if (EXTI_BASE->pr & (1 << 2)) {
        exti_clear_pending_bit(2);
        // encoder_irq_handler_left();
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI_BASE->pr & (1 << 3)) {
        exti_clear_pending_bit(3);
        // encoder_irq_handler_left();
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI_BASE->pr & (1 << 4)) {
        exti_clear_pending_bit(4);
        encoder_irq_handler_left();
    }
}

//5-9
void EXTI9_5_IRQHandler(void)
{
    uint32_t pr = EXTI_BASE->pr;

    if (pr & (1 << 5)) {
        exti_clear_pending_bit(5);
        // encoder_irq_handler_left();
    }

    if (pr & (1 << 6)) {
        exti_clear_pending_bit(6);
        // encoder_irq_handler_left();
    }

    if (pr & (1 << 7)) {
        exti_clear_pending_bit(6);
        // BUTTON 2
        // gpio_clr(GPIO_A, 5);
        gpio_clr(GPIO_B, 6);

        if (btn2_status) {
            btn2_irq_handler();
            btn2_status = 0;
        } else {
            btn2_status = 1;
        }   
    }

    if (pr & (1 << 8)) {
        exti_clear_pending_bit(6);
        // encoder_irq_handler_left();
    }

    if (pr & (1 << 9)) {
        exti_clear_pending_bit(6);
        // BUTTON 1
        // gpio_set(GPIO_A, 5);
        gpio_set(GPIO_B, 6);

        if (btn1_status) {
            btn1_irq_handler();
            btn1_status = 0;
        } else {
            btn1_status = 1;
        }
    }
}

//10-15
void EXTI15_10_IRQHandler(void)
{
    uint32_t pr = EXTI_BASE->pr;

    if (pr & (1 << 10)) {
        exti_clear_pending_bit(10);
        encoder_irq_handler_right();
    }

    if (pr & (1 << 11)) {
        exti_clear_pending_bit(11);
        // encoder_irq_handler_right();
    }

    if (pr & (1 << 12)) {
        exti_clear_pending_bit(12);
        // encoder_irq_handler_right();
    }

    if (pr & (1 << 13)) {
        exti_clear_pending_bit(13);
        // encoder_irq_handler_right();
    }

    if (pr & (1 << 14)) {
        exti_clear_pending_bit(14);
        // encoder_irq_handler_right();
    }

    if (pr & (1 << 15)) {
        exti_clear_pending_bit(15);
        // encoder_irq_handler_right();
    }
}