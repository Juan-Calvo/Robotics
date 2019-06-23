#ifndef Timer_config_h
#define Timer_config_h
  #define TIMER_DIVIDER 80
  void IRAM_ATTR interrupt_handler(void *param);
  static void timer_init_1 (uint32_t  timer_interval_sec);
  void IRAM_ATTR nextpoints(int x, int y,int z,int w,int u);
  void msteps();
  void clcsteps();
#endif
