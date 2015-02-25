#ifndef HW_STUFF_H
#define HW_STUFF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include "typedefs.h"

enum EHWAxisPower
{
    pwr_min = 0,
    pwr_med,
    pwr_high,
    pwr_max
};


// Always update from the real hw_stuff.h
#define SYSTEM_T_10MS_COUNT      500            // 20us timer -> 500 systicks / 10ms

#define TIMER_SYSTEM            TIM1

// Dummy stuff for simulation
struct STIM1
{
    uint16 SR;
    uint16 CRL;
};

#define TIM_FLAG_Update     0
// -----------


void InitHW(void);

void HW_ASSERT();


void LED_On( int i );
void LED_Off( int i );


bool BtnGet_Emerg();
bool BtnGet_Resume();
bool BtnGet_Home();

void HW_SetDirX_Plus();
void HW_SetDirX_Minus();
void HW_SetDirY_Plus();
void HW_SetDirY_Minus();
void HW_SetDirZ_Plus();
void HW_SetDirZ_Minus();
void HW_SetDirA_Plus();
void HW_SetDirA_Minus();

void HW_StepClk_X();
void HW_StepClk_Y();
void HW_StepClk_Z();
void HW_StepClk_A();

void HW_StepClk_Reset();
void HW_ResetClk_X();
void HW_ResetClk_Y();
void HW_ResetClk_Z();
void HW_ResetClk_A();



void StepDBG_Accelerations( int phase, int sense );     // 0 - beginnig, 1 - const, 2 - end;  sense: 1 - accel, 0 - decel
void StepDBG_LineSegment( struct SStepCoordinates *c1, struct SStepCoordinates *c2 );
void StepDBG_SegmentFinished( void );
void StepDBG_TickCount();

// just a wrapper solution
void main_entry(uint32 *stack_top);
void main_loop(void);

void StepTimerIntrHandler(void);

void HWDBG( int val );

#define __disable_interrupt()       do {  } while(0)
#define __enable_interrupt()        do {  } while(0)


#ifdef __cplusplus
 }
#endif


#endif // HW_STUFF_H
