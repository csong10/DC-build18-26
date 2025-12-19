.cpu cortex-m4
.syntax unified
.thumb

.section .text

.thumb_func
.global enter_user_mode
enter_user_mode:
  ldr r0, =__psp_stack_top
  msr PSP, r0
  b _crt0

.thumb_func
.global __def_idle_fn_spin
__def_idle_fn_spin:
  wfi
  b __def_idle_fn_spin