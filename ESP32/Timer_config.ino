/***********************************************************************
 *   TIMER INTERRUPT | BRESENHAM ALGORITHM                             *
 ***********************************************************************/
#include "Robot.h"
int prue=0;
TB6560 TB6560_1 = TB6560(STEP_PIN_1, DIR_PIN_1, EN_PIN_1);
TB6560 TB6560_2 = TB6560(STEP_PIN_2, DIR_PIN_2, EN_PIN_2);
TB6560 TB6560_3 = TB6560(STEP_PIN_3, DIR_PIN_3, EN_PIN_3);
TB6560 TB6560_4 = TB6560(STEP_PIN_4, DIR_PIN_4, EN_PIN_4);
TB6560 TB6560_5 = TB6560(STEP_PIN_5, DIR_PIN_5, EN_PIN_5);
TB6560 Drivers[5] ={TB6560_1, TB6560_2, TB6560_3,TB6560_4,TB6560_5};

template <typename T>
inline T  GetMax2 (T  a, T b) {
  return (a < b ? b:a);
}

template <typename T>
inline T  absVal (T  a) {
  return (a > 0 ? a:-a);
}

struct Bresenham
{
    int x0=0,y0=0,z0=0,w0=0,u0=0,x1,y1,z1,w1,u1;
    int dx,dy,dz,dw,du,dm, i=0;
    int step_a=0;
    int dir=0;
    int sx=1,sy=1,sz=1,sw=1,su=1;
};

Bresenham br;
POINTS Segment2; 

void IRAM_ATTR interrupt_handler(void* param){
  if(prue == 0){
    Drivers[0].inicio();
    Drivers[1].inicio();
    Drivers[2].inicio();
    Drivers[3].inicio();
    Drivers[4].inicio(); 
  }
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed     = 1;
  TIMERG0.wdt_wprotect = 0;
  prue=1;
  msteps();
  if (br.i-- < 0 ){
    br.x0 = br.y0 =br.z0 = br.w0=br.u0 =br.dx=br.dy = br.dz= br.dw =br.du=0;
    if(TCP_POINTS_Readings!=0){
      if (xQueueReceive( TCP_POINTS_Readings , &Segment2 ,(TickType_t)0 )==pdPASS){
      nextpoints(Segment2.getx(),Segment2.gety(),Segment2.getz(),Segment2.getw(),Segment2.getu());
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, Segment2.gett());
      }
    }
  } 
  br.x1 -= br.dx; if (br.x1 < 0) { br.x1 += br.dm; br.step_a |= 0b00001; br.x0 += br.sx;} 
  br.y1 -= br.dy; if (br.y1 < 0) { br.y1 += br.dm; br.step_a |= 0b00010; br.y0 += br.sy; }
  br.z1 -= br.dz; if (br.z1 < 0) { br.z1 += br.dm; br.step_a |= 0b00100; br.z0 += br.sz; }
  br.w1 -= br.dw; if (br.w1 < 0) { br.w1 += br.dm; br.step_a |= 0b01000; br.w0 += br.sw; }
  br.u1 -= br.du; if (br.u1 < 0) { br.u1 += br.dm; br.step_a |= 0b10000; br.u0 += br.su; }
 
  clcsteps();

  TIMERG0.int_clr_timers.t0 = 1;
  TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
}

void msteps(){
  if(br.step_a & 0b00001){ Drivers[0].Step(); }
  if(br.step_a & 0b00010){ Drivers[1].Step(); }
  if(br.step_a & 0b00100){ Drivers[2].Step(); }
  if(br.step_a & 0b01000){ Drivers[3].Step(); }
  if(br.step_a & 0b10000){ Drivers[4].Step(); }
  br.step_a = 0;
}

void clcsteps(){
  Drivers[0].RStep();
  Drivers[1].RStep();
  Drivers[2].RStep();
  Drivers[3].RStep();
  Drivers[4].RStep();
}
void IRAM_ATTR nextpoints(int x, int y, int z,int w,int u){
  
  br.dx = absVal(x); if (x < 0) { Drivers[0].Sdir(); }
                     else       { Drivers[0].Rdir(); }
  br.dy = absVal(y); if (y < 0) { Drivers[1].Sdir(); }
                     else       { Drivers[1].Rdir(); }
  br.dz = absVal(z); if (z < 0) { Drivers[2].Sdir(); }
                     else       { Drivers[2].Rdir(); }
  br.dw = absVal(w); if (w < 0) { Drivers[3].Sdir(); }
                     else       { Drivers[3].Rdir(); }
  br.du = absVal(u); if (u < 0) { Drivers[4].Sdir(); }
                     else       { Drivers[4].Rdir(); }
  
  br.x0 = br.y0 =br.z0=br.w0=br.u0= 0;
  
  br.dm = GetMax2 (br.dx, br.dy);
  br.dm =GetMax2 (br.dm, br.dz);
  br.dm =GetMax2 (br.dm, br.dw);
  br.dm = br.i =GetMax2 (br.dm, br.du);
  
  br.x1 = br.y1 =br.z1 =br.w1=br.u1=br.dm/2;
}

static void timer_init_1 (uint32_t  timer_interval_sec){
  timer_config_t config;
  config.alarm_en = TIMER_ALARM_EN;
  config.counter_en = TIMER_PAUSE;
  config.intr_type = TIMER_INTR_LEVEL;
  config.counter_dir = TIMER_COUNT_UP;
  config.auto_reload = 1;
  config.divider = TIMER_DIVIDER;
  timer_init(TIMER_GROUP_0 ,TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_interval_sec);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0,interrupt_handler, 0, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, TIMER_0);
}
