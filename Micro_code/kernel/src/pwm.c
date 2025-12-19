#include <printk.h>
#include <pwm.h>
#include <rcc.h>
#include <unistd.h>
#include <motor_driver.h>

/** @brief TIM1 register map */
struct tim1 {
    volatile uint32_t cr1;     /**< 00 Control Register 1 */
    volatile uint32_t cr2;     /**< 04 Control Register 2 */
    volatile uint32_t smcr;    /**< 08 Slave Mode Control */
    volatile uint32_t dier;    /**<*< 0C DMA/Interrupt Enable */
    volatile uint32_t sr;      /**< 10 Status Register */
    volatile uint32_t egr;     /**< 14 Event Generation */
    volatile uint32_t ccmr[2]; /**< 18-1C Capture/Compare Mode */
    volatile uint32_t ccer;    /**< 20 Capture/Compare Enable */
    volatile uint32_t cnt;     /**< 24 Counter Register */
    volatile uint32_t psc;     /**< 28 Prescaler Register */
    volatile uint32_t arr;     /**< 2C Auto-Reload Register */
    volatile uint32_t rcr;     /**< 30 Repetition Counter Register */
    volatile uint32_t ccr[4];  /**< 34-40 Capture/Compare */
    volatile uint32_t bdtr;    /**< 44 Break and Dead-Time Register */
    volatile uint32_t dcr;     /**< 48 DMA Control Register */
    volatile uint32_t dmar;    /**< 4C DMA address for full transfer Register */
};

/** @brief TIM2-5 register map */
struct tim2_5 {
    volatile uint32_t cr1;        /**< 00 Control Register 1 */
    volatile uint32_t cr2;        /**< 04 Control Register 2 */
    volatile uint32_t smcr;       /**< 08 Slave Mode Control */
    volatile uint32_t dier;       /**< 0C DMA/Interrupt Enable */
    volatile uint32_t sr;         /**< 10 Status Register */
    volatile uint32_t egr;        /**< 14 Event Generation */
    volatile uint32_t ccmr[2];    /**< 18-1C Capture/Compare Mode */
    volatile uint32_t ccer;       /**< 20 Capture/Compare Enable */
    volatile uint32_t cnt;        /**< 24 Counter Register */
    volatile uint32_t psc;        /**< 28 Prescaler Register */
    volatile uint32_t arr;        /**< 2C Auto-Reload Register */
    volatile uint32_t reserved_1; /**< 30 */
    volatile uint32_t ccr[4];     /**< 34-40 Capture/Compare */
    volatile uint32_t reserved_2; /**< 44 */
    volatile uint32_t dcr;        /**< 48 DMA Control Register */
    volatile uint32_t dmar; /**< 4C DMA address for full transfer Register */
    volatile uint32_t or ;  /**< 50 Option Register */
};

extern uint8_t IS_COMP; //0 means PWM mode, 1 means in basic compare

#define TIM1_BASE (struct tim1 *) 0x40010000 // TODO: fill out address for TIMER 1

struct tim2_5* const timer_base[] = {(void *)0x0,   // N/A - Don't fill out
                                     (void *)0x0,   // N/A - Don't fill out
                                     (void *)0x40000000,    // TODO: fill out address for TIMER 2
                                     (void *)0x40000400,    // TODO: fill out address for TIMER 3
                                     (void *)0x40000800,    // TODO: fill out address for TIMER 4
                                     (void *)0x40000C00};   // TODO: fill out address for TIMER 5

#define ARPE (1U << 7)
#define CCMR_OCM_PWM (6U)
#define CCMR_OCPE (1U << 3)
#define MOE (1U << 15)

#define CCMR_OFFSET(channel) ((channel - 1) * 8)
#define CCMR_IND(channel) ((channel <= 2) ? 0 : 1)
#define CCER_CCxE(channel) (1U << ((channel - 1) * 4))
#define CCER_CCxNE(channel) (1U << (((channel - 1) * 4) + 2))

void timer_start_pwm(uint32_t period, uint32_t duty_cycle, uint32_t timer, uint32_t channel) {
    if (timer < 1 || timer > 5 || channel < 1 || channel > 4) {
        printk("ERROR: Invalid timer or channel.\n");
        return;
    }
  struct rcc_reg_map *rcc = RCC_BASE;
  volatile uint32_t *cr1_ptr, *psc_ptr, *arr_ptr, *ccmr_ptr_base, *ccer_ptr, *ccr_ptr_base, *egr_ptr;
  volatile uint32_t *bdtr_ptr = NULL;
  
  if (timer == 1) {
    struct tim1 *TIM1 = TIM1_BASE;
    rcc->apb2_enr |= 1U;

    cr1_ptr = &TIM1->cr1;
    psc_ptr = &TIM1->psc;
    arr_ptr = &TIM1->arr;
    ccmr_ptr_base = TIM1->ccmr;
    ccer_ptr = &TIM1->ccer;
    ccr_ptr_base = TIM1->ccr;
    bdtr_ptr = &TIM1->bdtr;
    egr_ptr = &TIM1->egr;
  } else {
    struct tim2_5 *tim = timer_base[timer];
    rcc->apb1_enr |= (1 << (timer-2));

    cr1_ptr = &tim->cr1;
    psc_ptr = &tim->psc;
    arr_ptr = &tim->arr;
    ccmr_ptr_base = tim->ccmr;
    ccer_ptr = &tim->ccer;
    ccr_ptr_base = tim->ccr;
    egr_ptr = &tim->egr;
  }

  //set prescaler and period
  *psc_ptr = 16; //was prescalar -1
  *arr_ptr = period;

  //set arpe reload
  *cr1_ptr |= ARPE;

  uint32_t ccmr_ind = CCMR_IND(channel);
  uint32_t ccmr_offset = CCMR_OFFSET(channel);

  //clear ccmr for channel
  ccmr_ptr_base[ccmr_ind] &= ~(7U << (ccmr_offset+4));

  //set OCxM to PWM mode 2
  ccmr_ptr_base[ccmr_ind] |= (CCMR_OCM_PWM << (ccmr_offset+4));

  //set OCPE
  ccmr_ptr_base[ccmr_ind] |= (CCMR_OCPE << ccmr_offset);

  //set duty cycle
  ccr_ptr_base[channel-1] = duty_cycle;

  //enable event generation
  *egr_ptr |= 1;

  if (IS_COMP) {
    *ccer_ptr |= CCER_CCxNE(channel);
  }

  //enable CCxE
  *ccer_ptr |= CCER_CCxE(channel);

  if (timer == 1 && bdtr_ptr != NULL) {
    *bdtr_ptr |= MOE;
  }

  //enable counter
  *cr1_ptr |= 1;
  return;
}

void timer_disable_pwm(uint32_t timer, uint32_t channel) {
  if (timer < 1 || timer > 5 || channel < 1 || channel > 4) {
    return;
  }

  if (timer == 1) {
    struct tim1 *TIM1 = TIM1_BASE;
    TIM1->ccer &= ~CCER_CCxE(channel);
    if (IS_COMP) {
      TIM1->ccer &= ~CCER_CCxNE(channel);
    }
    TIM1->bdtr &= ~MOE;
    TIM1->cr1 &= ~1U;

  } else {
    struct tim2_5 *tim = timer_base[timer];
    tim->ccer &= ~CCER_CCxE(channel);
    if (IS_COMP) {
      tim->ccer &= ~CCER_CCxNE(channel);
    }
    tim->cr1 &= ~1U;
  }
}

void timer_set_duty_cycle(uint32_t timer, uint32_t channel, uint32_t duty_cycle) {
  if (timer < 1 || timer > 5 || channel < 1 || channel > 4) {
    return;
  }

  if (timer == 1) {
    struct tim1 *TIM1 = TIM1_BASE;
    TIM1->ccr[channel - 1] = duty_cycle;
  } else {
    struct tim2_5 *tim = timer_base[timer];
    tim->ccr[channel - 1] = duty_cycle;
  }
}

