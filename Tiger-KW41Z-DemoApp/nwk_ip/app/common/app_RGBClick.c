//////////////////////////////////////////////////////////////
// Copyright(c) 2016, Volansys Technologies
//
// Description:
/// \file app_RGBClick.c
/// \brief This is the source file for mikroBus 4x4 RGB click module 
///	configuration.
///
//
// Author Volansys Technologies
//
//////////////////////////////////////////////////////////////

/*============================================================
Include Files
=============================================================*/
#include "app_RGBClick.h"
#include "fsl_os_abstraction.h"
#include "fsl_gpio.h"

#if RGB_CLICK_ENABLE && VT_KW41Z_MENP

/*============================================================
Global Variables
=============================================================*/
//int i;
int n = 200;
unsigned long DiodeArray[MAX_SIZE];

/*============================================================
Static Function Declaration
=============================================================*/
static void APP_ProcessRGBClickCmd(uint8_t *pCommand, uint8_t dataLen);
static void setColor(uint32_t red, uint32_t green, uint32_t blue);

/*============================================================
Function definations
=============================================================*/

static inline void sleep_us (){
  unsigned int i=0;
  for( i = 0 ; i < 500; i++) {
  __asm("NOP");
  __asm("NOP");
  __asm("NOP");
  __asm("NOP");
  }
  
}

static inline void RGBLed_ResetDelay() {
    sleep_us();
}

static inline void RGBLed_InitDiode(unsigned long ARGBColor, unsigned long * AdiodeArray) {
  *AdiodeArray = (ARGBColor & 0x000000FF) | ((ARGBColor >> 16) << 8) | ((ARGBColor >> 8) << 16);
}

static inline void RGBLed_SetColor(unsigned long * AdiodeArray) {
  if (( (*AdiodeArray) & (1ul << 23)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 22)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 21)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 20)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 19)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 18)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 17)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 16)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 15)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 14)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 13)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 12)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 11)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 10)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 9)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 8)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 7)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 6)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 5)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 4)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 3)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 2)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 1)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
  if (( (*AdiodeArray) & (1ul << 0)) == 0) {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");

  }
  else {
	  ((GPIO_Type *)GPIOC_BASE)->PSOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  ((GPIO_Type *)GPIOC_BASE)->PCOR = 0x80000;
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
	  __asm("NOP");
  }
}

static inline void RGBLed_SetDiode(char ANum, unsigned long AColor, unsigned long * AdiodeArray) {
  char i;

  /* This whole operation is time critical and shall be Atomic.
   * It must not be interrupted or delayed, as it affect the timing required
   * for the RGB click
   */
   OSA_InterruptDisable();
  
   for (i = 0; i < MAX_SIZE; i++) {
    if (i == (ANum - 1)) {
      RGBLed_InitDiode(AColor, &AdiodeArray[i]);
      RGBLed_SetColor(&AdiodeArray[i]);
    }
    else {
      RGBLed_SetColor(&AdiodeArray[i]);
    }
  }
  RGBLed_ResetDelay();
  
  OSA_InterruptEnable();
}

void RGBLed_Init() {
  GpioOutputPinInit (&RGBPins[0],1);
  BlankScreen ();
}

inline void Delay_time() {
  unsigned int temp=0;
  for (temp=1; temp<=n; temp++)
  sleep_us();
}

// Start snake
void Snake(unsigned long RGBColor) {
  RGBLed_SetDiode(4, RGBColor, DiodeArray);  // Turn on diode 4 with the desired color
  Delay_time();
  RGBLed_SetDiode(3, RGBColor, DiodeArray);  // Turn on diode 3 with the desired color
  Delay_time();
  RGBLed_SetDiode(2, RGBColor, DiodeArray);  // Turn on diode 2 with the desired color
  Delay_time();
  RGBLed_SetDiode(1, RGBColor, DiodeArray);  // Turn on diode 1 with the desired color
  Delay_time();
  RGBLed_SetDiode(5, RGBColor, DiodeArray);  // Turn on diode 5 with the desired color
  Delay_time();
  RGBLed_SetDiode(9, RGBColor, DiodeArray);  // Turn on diode 9 with the desired color
  Delay_time();
  RGBLed_SetDiode(13, RGBColor, DiodeArray); // Turn on diode 13 with the desired color
  Delay_time();
  RGBLed_SetDiode(14, RGBColor, DiodeArray); // Turn on diode 14 with the desired color
  Delay_time();
  RGBLed_SetDiode(15, RGBColor, DiodeArray); // Turn on diode 15 with the desired color
  Delay_time();
  RGBLed_SetDiode(16, RGBColor, DiodeArray); // Turn on diode 16 with the desired color
  Delay_time();
  RGBLed_SetDiode(12, RGBColor, DiodeArray); // Turn on diode 12 with the desired color
  Delay_time();
  RGBLed_SetDiode(8, RGBColor, DiodeArray);  // Turn on diode 8 with the desired color
  Delay_time();
  RGBLed_SetDiode(7, RGBColor, DiodeArray);  // Turn on diode 7 with the desired color
  Delay_time();
  RGBLed_SetDiode(6, RGBColor, DiodeArray);  // Turn on diode 6 with the desired color
  Delay_time();
  RGBLed_SetDiode(10, RGBColor, DiodeArray); // Turn on diode 10 with the desired color
  Delay_time();
  RGBLed_SetDiode(11, RGBColor, DiodeArray); // Turn on diode 11 with the desired color
}
// Return Snake
 void Snake_return (unsigned long RGBColor)  {
  RGBLed_SetDiode(11, RGBColor, DiodeArray); // Turn on diode 11 with the desired color
  Delay_time();
  RGBLed_SetDiode(10, RGBColor, DiodeArray); // Turn on diode 10 with the desired color
  Delay_time();
  RGBLed_SetDiode(6, RGBColor, DiodeArray);  // Turn on diode 6 with the desired color
  Delay_time();
  RGBLed_SetDiode(7, RGBColor, DiodeArray);  // Turn on diode 7 with the desired color
  Delay_time();
  RGBLed_SetDiode(8, RGBColor, DiodeArray);  // Turn on diode 8 with the desired color
  Delay_time();
  RGBLed_SetDiode(12, RGBColor, DiodeArray); // Turn on diode 12 with the desired color
  Delay_time();
  RGBLed_SetDiode(16, RGBColor, DiodeArray); // Turn on diode 16 with the desired color
  Delay_time();
  RGBLed_SetDiode(15, RGBColor, DiodeArray); // Turn on diode 15 with the desired color
  Delay_time();
  RGBLed_SetDiode(14, RGBColor, DiodeArray); // Turn on diode 14 with the desired color
  Delay_time();
  RGBLed_SetDiode(13, RGBColor, DiodeArray); // Turn on diode 13 with the desired color
  Delay_time();
  RGBLed_SetDiode(9, RGBColor, DiodeArray);  // Turn on diode 9 with the desired color
  Delay_time();
  RGBLed_SetDiode(5, RGBColor, DiodeArray);  // Turn on diode 5 with the desired color
  Delay_time();
  RGBLed_SetDiode(1, RGBColor, DiodeArray);  // Turn on diode 1 with the desired color
  Delay_time();
  RGBLed_SetDiode(2, RGBColor, DiodeArray);  // Turn on diode 2 with the desired color
  Delay_time();
  RGBLed_SetDiode(3, RGBColor, DiodeArray);  // Turn on diode 3 with the desired color
  Delay_time();
  RGBLed_SetDiode(4, RGBColor, DiodeArray);  // Turn on diode 4 with the desired color
  Delay_time();
}

// Fill all the diodes with one color
void FillScreen(unsigned long RGBColor) {
  unsigned int temp=0;
  
  for (temp=1; temp<=16; temp++) {
    RGBLed_SetDiode(temp, RGBColor, DiodeArray);
  }
}

// Fill all the diodes with one color
void BlankScreen() {
  unsigned int temp=0;
  
  for (temp=1; temp<=16; temp++) {
    RGBLed_SetDiode(temp, 0x00, DiodeArray);
  }
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapRGBCb(bool_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP RGB Click message.
\brief  It performs the required operations and sends back a CoAP ACK message.

\param  [in]    sessionStatus   status for CoAP session
\param  [in]    pData           pointer to CoAP message payload
\param  [in]    pSession        pointer to CoAP session
\param  [in]    dataLen         length of CoAP payload

\return         void
***************************************************************************************************/
void APP_CoapRGBCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    extern const uint32_t gCoapMaxOptions;
  
    /* Process the command only if it is a POST method */
    if ((pData) && (sessionStatus == gCoapSuccess_c) && (pSession->code == gCoapPOST_c))
    {
        APP_ProcessRGBClickCmd(pData, dataLen);
    }
    
    /* Send the reply if the status is Success or Duplicate */
    if ((gCoapFailure_c != sessionStatus) && (gCoapConfirmable_c == pSession->msgType))
    {
        /* Send CoAP ACK */
        COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_ProcessRGBClickCmd(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to process a RGB Click module command (rgb with brightness).

\param  [in]    pCommand   command data
\param  [in]    dataLen    data length

\return         void
***************************************************************************************************/
static void APP_ProcessRGBClickCmd(uint8_t *pCommand, uint8_t dataLen)
{
    char* p = (char *)pCommand;
    uint8_t cmdId = 255;
    uint8_t i=0;
    uint8_t redValue = 0, greenValue = 0, blueValue = 0;
    appDeviceState_t appState = gDeviceState_AppLedRgb_c;
    
    if(dataLen > 3)
    {
        cmdId = NWKU_atoi(p);
        p++;
        switch(cmdId)
        {
          case RGBColorOperation:
            shell_printf("Setting RGB Pattern\r\n");
            
            //Read Red, Green and Blue Color Value
            redValue = *p;
            p++;
            greenValue = *p;
            p++;
            blueValue = *p;
            shell_printf("R = %2x\tG = %2x\tB = %2x\t\r\n",redValue,greenValue,blueValue);
            setColor(redValue,greenValue,blueValue);
            break;
          default:
            shell_printf("Invalid Click module command\r\n");
        }
    }
    else
    {
        shell_printf("Invalid CoAP Payload received\r\n");
    }
    
}

static void setColor(uint32_t red, uint32_t green, uint32_t blue)
{
    uint32_t RGB_value = 0;
    
    shell_printf("[setColor] Red 0x%x, Green 0x%x, Blue 0x%x.\r\n",
                 red, green, blue);
    
    if (red > MAX_ALLOWED_BRIGHTNESS)
    {
        red = MAX_ALLOWED_BRIGHTNESS;
    }
    if (green > MAX_ALLOWED_BRIGHTNESS)
    {
        green = MAX_ALLOWED_BRIGHTNESS;
    }
    if (blue > MAX_ALLOWED_BRIGHTNESS)
    {
        blue = MAX_ALLOWED_BRIGHTNESS;
    }
    
    RGB_value = (red << 16) | (green << 8) | blue;
    shell_printf("[setColor] RGB 0x%06x,\r\n",RGB_value);

    FillScreen(RGB_value);
}

#endif /* RGB_CLICK_ENABLE */
