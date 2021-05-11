#include "includes.h"

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define  N_TASKS        3

#define do 57892
#define re 58725
#define mi 59468
#define fa 59809
#define sol 60433
#define la 60990
#define si 61080
#define udo 61714

const int alarmsong[100] = { mi, re, do, re, mi, mi, mi, 
							re, re, re, mi, sol, sol, 
							mi, re, do, re, mi, mi, mi,
							re, re, mi, re, do};
							
const unsigned char digit[13] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f, 0x40, 0x00}; // -, notrhing 
const unsigned char fnd_sel[4] = { 0x08,0x04,0x02,0x01 };
const unsigned char dot = 0x80;
const unsigned int led_scale[8] = {0xFF, 0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x02, 0x01};  
const int shift[4] = {0x01, 0x02, 0x04};

OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];
OS_EVENT	  *Mbox;


volatile INT8U	FndNum;

void  task1(void *data);
void  task2(void *data);
void  task3(void *data);

unsigned short read_adc(void);
void init_adc(void);
void display_fnd(int c);
void write_twi_1byte_nopreset(unsigned char reg, unsigned char data);
void write_twi_0byte_nopreset(unsigned char reg);
void InitI2C();
int ReadTemperature(void);
//int ReadTemperature(void);
//void  FndTask(void *data);
//void  FndDisplayTask(void *data);


/////////
int i=0;
unsigned char num[3];
unsigned char temptest[3];
int state=ON; // 주파수 발생시키기 위해 소리를 끄고 킬 변수
volatile int count=0;
volatile int sec=0; volatile int min=0; volatile int time=0;

int alarm=-1;
int term=0;
int alarm_s=0; int alarm_m=0;
int illu;
volatile int flag=0;
////////


////////
OS_EVENT* mbox_alarm; // master task를 전달하기 위한 mail box
OS_EVENT* mbox_illu;
OS_EVENT* alarm_end;
OS_FLAG_GRP *ground;  
OS_EVENT *sem;
INT8U err;
///////


ISR(INT5_vect){ //toggle
	flag++; _delay_ms(300); flag%=2; 
	OSFlagPost(ground, 0xFF, OS_FLAG_CLR , &err);
	OSFlagPost(ground, shift[flag], OS_FLAG_SET, &err);
}

 
ISR (TIMER2_OVF_vect){  

if (count==61){
	sec++; 
	if(sec==60) { min++; sec=0; }
	count=0;}
else count++;
time=sec+min*100;
}



ISR (TIMER1_OVF_vect){  
	int pre=illu;
	illu=read_adc()/200; // 5초마다 조도 센싱 
	if (pre!=illu) OSMboxPost(mbox_illu, &illu);
	//PORTA=led_scale[jodo/200];
}
  
 
int sound =0; int scnt=0;

ISR (TIMER3_OVF_vect){  
 if (state==ON) { 
 	PORTB=0x00; // state on일경우 buzzer 출력값 1 설정
	state=OFF;} // state 상태전환 
 else {
 	PORTB=0x10; // state off일 경우 buzzer 출력값 0 설정
	state=ON; } // state 상태전환

	if (scnt<900) TCNT3=alarmsong[(sound%25)]; // 현재 index에 해당하는 소리 출력
	else { scnt=0; sound++;	}
	scnt++;
}

 
 
int main (void){

  OSInit();
  OS_ENTER_CRITICAL();
  TCCR0 = 0x07;
  TIMSK = _BV(TOIE0);
  TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
  OS_EXIT_CRITICAL();
	
  // register set
  // LED
  DDRA = 0xff;
  // FND
  DDRC = 0xff;
  DDRG = 0x0f;
  // buzzer
  DDRB=0x10;
  // switch 
  DDRE=0x00;
  // overflow
  TCCR0=0x03; 
  TIMSK=0x44; // overflow 되도록 설정	
  
  // TIMER OVF2
  TCCR2=0x05; // 7에서작동안함 prescaler = 128  
  
  //TIMER OVF1
  TCCR1B=4;
  TCNT1=3036;
//  TIMSK|=0x40;
  //TIMER OVF3
  //TCCR3B=0x01;
  ETIMSK|=((1<<TOIE3));  
  // interrupt
  EIMSK=0x20; // Intertrupt 4,5 활성화 (0011 0000)
  sei();
  ////////////////////////////////
  mbox_alarm = OSMboxCreate((void*)0); 
  mbox_illu = OSMboxCreate((void*)0); 
  alarm_end = OSMboxCreate((void*)0); 
  ground = OSFlagCreate((OS_FLAGS)0x01, &err); // s_grp의 Event Flag 생성
  ///
  
  OSTaskCreate(task1, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 0);
  OSTaskCreate(task2, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 1);
  OSTaskCreate(task3, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 2);

  OSStart();

  return 0;
}



void  task1(void *data){
	init_adc();
	int alarming;
	int set_alarm=1;
	int light;
	int alarm_flag=(void*)0;
	while(1){
		OSFlagPend(ground, 0x01, OS_FLAG_WAIT_SET_ALL, 0, &err);
		
		display_fnd(time);
		
		if((light=OSMboxAccept(mbox_illu))!=0x00) { PORTA=led_scale[*(int*)light]; }
		if((alarming=OSMboxAccept(mbox_alarm))!=0x00) { alarm_flag=1; set_alarm=*(int*)alarming;}
		if (alarm_flag && time==set_alarm) { TCCR3B=1; OSMboxPost(alarm_end, &set_alarm); alarm_flag=0; }   
		if (TCCR3B && ((PINE&0x10)==0x00) ) TCCR3B=0; // 알람 울리고 테스크2에도 알려줘야됌  
		if ((time!=0) && (time%300==0)){
		OSFlagPost(ground, 0xFF, OS_FLAG_CLR , &err);
		OSFlagPost(ground, 0x04, OS_FLAG_SET, &err);}
	
}
}



 
void  task2(void *data){
	int term=0;
	int flag=0;
	int set_alarm;
	while(1){
	OSFlagPend(ground, 0x02, OS_FLAG_WAIT_SET_ALL, 0, &err);
	display_fnd(alarm_s+alarm_m*100);
	if(OSMboxAccept(alarm_end)!=0x00) { term=0; flag=0; alarm_s=0; alarm_m=0; }
	if ((EIMSK==0x00)&&((PINE&0x20)==0x00)) term=1;
	if (((PINE&0x10)==0x00)&&(term==0)) {
		EIMSK=0x00;
		_delay_ms(200); alarm_s++;  alarm_s%=60;  flag=0;
		}

	if ((term==1)&&((PINE&0x10)==0x00)){
		_delay_ms(200); alarm_m++; alarm_m%=60; flag=1;
		 }
		 
	if (flag&&((PINE&0x20)==0x00)) {
	
	set_alarm=alarm_s+alarm_m*100;
	OSMboxPost(mbox_alarm, &set_alarm);
	EIMSK=0x20; term=0;}
}

}

//온도  
void  task3(void *data){  
  InitI2C();
  write_twi_1byte_nopreset(ATS75_CONFIG_REG, 0x00); // 9비트, Normal
  write_twi_0byte_nopreset(ATS75_TEMP_REG);
  int value; int i;
  unsigned char value_int, value_deci, num[4];
  while(1){ 
  	OSFlagPend(ground, 0x04, OS_FLAG_WAIT_SET_ALL, 0, &err);
  	 value = ReadTemperature();
  	  if((value & 0x8000) != 0x8000)  // Sign 비트 체크
		num[0] = 11;
    else
    {
		num[0] = 12;
		value = (~value)-1;   // 2’s Compliment
    }

    value_int = (unsigned char)((value & 0x7f00) >> 8);
    value_deci = (unsigned char)(value & 0x00ff);
    num[1] = (value_int / 10) % 10;
    num[2] = value_int % 10;
    num[3] = ((value_deci & 0x80) == 0x80) * 5; 
    
    int k=350;
	while(k){
    for(i=0; i<4; i++)
    {
        PORTC = digit[num[i]];
        PORTG = fnd_sel[i];
        if(i==2) PORTC |= 0x80;
        _delay_ms(2);
    } k--;}
    
	  OSFlagPost(ground, shift[0], OS_FLAG_SET, &err);
  }
}
 
 
 
 
 void display_fnd(int c){
	num[3]=digit[(c)%10];  
	num[2]=digit[(c/10)%10];  
	num[1]=digit[(c/100)%10]+dot;  
	num[0]=digit[(c/1000)]; 
	for(i=0; i<4; i++){
	PORTC=num[i];
	PORTG=fnd_sel[i]; 
	_delay_us(2500);
}

}



void init_adc(){
	ADMUX=0x00;
	ADCSRA = 0x87;
}


unsigned short read_adc(){
	unsigned char adc_low,adc_high;
	unsigned short value;
	ADCSRA |= 0x40; // ADC start conversion, ADSC = '1'
	while((ADCSRA & (0x10)) != 0x10); // ADC 변환 완료 검사
	adc_low=ADCL;
	adc_high=ADCH;
	value = (adc_high <<8) | adc_low;
	
	return value;
}


int ReadTemperature(void)
{
	int value;

	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	TWDR = 0x98 + 1; //TEMP_I2C_ADDR + 1
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while(!(TWCR & _BV(TWINT)));

	//온도센서는 16bit 기준으로 값을 가져오므로
	//8비트씩 2번을 받아야 한다.
	value = TWDR; 
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	value = ((value<< 8)|TWDR);
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);

	//value >>= 8;
	TIMSK = (value >= 33) ? TIMSK | _BV(TOIE2): TIMSK & ~_BV(TOIE2);

	return value;
}


void write_twi_1byte_nopreset(unsigned char reg, unsigned char data)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || ((TWSR & 0xf8) != 0x08 &&   (TWSR & 0xf8) != 0x10)); // ACK를 기다림
	TWDR = ATS75_ADDR | 0;  // SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);  // SLA+W 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18); 
	TWDR = reg;    // aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);  // aTS75 Reg 값 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWDR = data;    // DATA 준비
	TWCR = (1 << TWINT) | (1 << TWEN);  // DATA 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
}

void write_twi_0byte_nopreset(unsigned char reg)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || ((TWSR & 0xf8) != 0x08 &&   (TWSR & 0xf8) != 0x10));  // ACK를 기다림
	TWDR = ATS75_ADDR | 0; // SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);  // SLA+W 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18); 
	TWDR = reg;    // aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);  // aTS75 Reg 값 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
}

void InitI2C()
{
    PORTD = 3; 						// For Pull-up override value
    SFIOR &= ~(1 << PUD); 			// PUD
    TWSR = 0; 						// TWPS0 = 0, TWPS1 = 0
    TWBR = 32;						// for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);	// TWEA = Ack pulse is generated
									// TWEN = TWI 동작을 가능하게 한다
}


