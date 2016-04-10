// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
#define SENSOR  (*((volatile unsigned long *)0x4002401C)) // accessing PE2-PE1–PE0
#define LIGHT   (*((volatile unsigned long *)0x400050FC)) // accesses PB5–PB0
#define LIGHT_P (*((volatile unsigned long *)0x40025028)) // accesses PF1 and PF3
	
	struct State {
  unsigned long Out;  // 6-bit pattern to output
  unsigned long Time; // delay in 10ms units 
  unsigned long Next[8];}; // next state for inputs 0,1,2,3
  typedef const struct State STyp;
	
	#define GoW 0  		// go west
	#define WWS 1	    // Wait from West to South
	#define WWS1 2	    // Wait from West to South
	#define WWP 3			// Wait from West to Pedestrian
	#define GoS 4			// Wait from West to Pedestrian
	#define WSW 5			// Wait from South to West
	#define WSW1 6			// Wait from South to West
	#define WSP 7			// Wait from South to Pedestrian
	#define GoP 8			// Go Pedestrian
	#define WPW 9			// Wait from Pedestrian to West
	#define GoP1W 10		// Wait from Pedestrian to West flashing red
	#define GoP2W 11   // Wait from Pedestrian to West flashing red
	#define GoP3W 12	// Wait from Pedestrian to West flashing red
	#define GoP4W 13	// Wait from Pedestrian to West flashing red
	#define GoP5W 14	// Wait from Pedestrian to West flashing red
	#define GoP6W 15	// Wait from Pedestrian to West flashing red
	#define WPS   16  // Wait from Pedestrian to South
	#define GoP1S 17	// Wait from Pedestrian to South flashing red
	#define GoP2S 18	// Wait from Pedestrian to South flashing red
	#define GoP3S 19	// Wait from Pedestrian to South flashing red
	#define GoP4S 20	// Wait from Pedestrian to South flashing red
	#define GoP5S 21	// Wait from Pedestrian to South flashing red
	
	STyp FSM[22]={
 {0x8C,10,{GoW,GoW,WWS,WWS,WWP,WWP,WWS,WWS}}, //0
 {0x94,10,{WWS1,WWS1,WWS1,WWS1,WWS1,WWS1,WWS1,WWS1}},//1
 {0xA2,10,{GoS,GoS,GoS,GoS,GoS,GoS,GoS,GoS}},//1
 {0x94,10,{GoP,GoP,GoP,GoP,GoP,GoP,GoP,GoP}},//2
 {0xA1, 10,{GoS,WSW,GoS,WSW,WSP,WSP,WSP,WSP}},//3
 {0xA2, 10,{WSW1,WSW1,WSW1,WSW1,WSW1,WSW1,WSW1,WSW1}},//4
 {0x94, 10,{GoW,GoW,GoW,GoW,GoW,GoW,GoW,GoW}},//4
 {0xA2, 10,{GoP,GoP,GoP,GoP,GoP,GoP,GoP,GoP}},//5
 {0x64, 10,{GoP,WPW,WPS,WPW,GoP,WPW,WPS,WPW}},//6
 {0xA4, 5,{GoP1W,GoP1W,GoP1W,GoP1W,GoP1W,GoP1W,GoP1W,GoP1W}},//7
 {0x24, 5,{GoP2W,GoP2W,GoP2W,GoP2W,GoP2W,GoP2W,GoP2W,GoP2W}},//8
 {0xA4, 5,{GoP3W,GoP3W,GoP3W,GoP3W,GoP3W,GoP3W,GoP3W,GoP3W}},//9
 {0x24, 5,{GoP4W,GoP4W,GoP4W,GoP4W,GoP4W,GoP4W,GoP4W,GoP4W}},//10
 {0xA4, 5,{GoP5W,GoP5W,GoP5W,GoP5W,GoP5W,GoP5W,GoP5W,GoP5W}},//11
  {0x24, 5,{GoP6W,GoP6W,GoP6W,GoP6W,GoP6W,GoP6W,GoP6W,GoP6W}},//12
 {0xA4, 5,{GoW,GoW,GoW,GoW,GoW,GoW,GoW,GoW}},//13
 {0xA4, 5,{GoP1S,GoP1S,GoP1S,GoP1S,GoP1S,GoP1S,GoP1S,GoP1S}},//14
 {0x24, 5,{GoP2S,GoP2S,GoP2S,GoP2S,GoP2S,GoP2S,GoP2S,GoP2S}},//15
 {0xA4, 5,{GoP3S,GoP3S,GoP3S,GoP3S,GoP3S,GoP3S,GoP3S,GoP3S}},//16
 {0x24, 5,{GoP4S,GoP4S,GoP4S,GoP4S,GoP4S,GoP4S,GoP4S,GoP4S}},//17
 {0xA4, 5,{GoP5S,GoP5S,GoP5S,GoP5S,GoP5S,GoP5S,GoP5S,GoP5S}},//18
 {0x24, 10,{GoS,GoS,GoS,GoS,GoS,GoS,GoS,GoS}}//19
 	};
	
	unsigned long S;  // index to the current state 
	unsigned long Input;
	unsigned short var=0;
	//unsigned long arr[20];
	volatile unsigned long delay; 	

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void SysTick_Wait10ms(unsigned long );
	void SysTick_Init(void);
void SysTick_Wait(unsigned long delay);

// ***** 3. Subroutines Section *****

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
 EnableInterrupts();
	
	//*********************
	
	SysTick_Init();   // Program 10.2
  SYSCTL_RCGC2_R |= 0x32;      // 1) B E F
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
	

	
  GPIO_PORTE_AMSEL_R &= ~0x03; // 3) disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE1-0
	
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 PF1
	GPIO_PORTF_AMSEL_R &= ~0x0A; // 3) disable analog function on PF3 PF1
  GPIO_PORTF_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTF_DIR_R |= 0x0A;    // 5) outputs on PF3 PF1
  GPIO_PORTF_AFSEL_R &= ~0x0A; // 6) regular function on PF3 PF1
  GPIO_PORTF_DEN_R |= 0x0A;    // 7) enable digital on PF3 PF1
	


	//*********************
	
	S = GoW; 
  while(1){
		LIGHT = FSM[S].Out & 0x3F;  // set lights
		//var =  FSM[S].Out;
		if(FSM[S].Out & 0x80)
		{LIGHT_P = 0x02;
		} // red
		
		else if(FSM[S].Out & 0x40)
		{LIGHT_P = 0x08;} // green
		//else if((FSM[S].Out & ~0x80) && (FSM[S].Out & ~0x40) )
			else if(FSM[S].Out & ~0xC0)
		{LIGHT_P = 0x00;}
    
    SysTick_Wait10ms(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input]; 
  }
}

//********* SYsTICK conf
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}
