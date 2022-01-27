/** \mainpage Autorzy: Maciej Szewczyk, Krzysztof Tomañski
	 Za³o¿enia:
	- termometr wyposa¿ony jest w wyœwietlacz siedmiosegmentowy LED, diody LED, przyciski oraz czujnik
	temperatury typu DS1820,
	- termometr realizuje pomiar bie¿¹cej temperatury z rozdzielczoœci¹ 0.1?C, rejestracjê, w zadanym okresie
	czasu, temperatury maksymalnej i minimalnej oraz dodatkowo sygnalizacjê przekroczenia ustawionych
	progów temperatury (dolnego lub górnego),
	- na wyœwietlaczu LED powinna byæ wyœwietlana temperatura bie¿¹ca, a po naciœniêciu przycisku tempe-
	ratura maksymalna, nastêpnie minimalna (ka¿da z wartoœci dostêpna jest przez czas ok. 3 sek., po czym
	wyœwietlana jest znów temperatura bie¿¹ca, rodzaj wyœwietlanej informacji sygnalizowany jest diodami
	LED), jeden z przycisków nale¿y wykorzystaæ do zerowania zarejestrowanych do tej pory temperatur
	maksymalnej i minimalnej, kolejny przycisk pozwala przejœæ do trybu ustawiania temperatur progowych
	(kolejno dolnego progu, nastêpnie górnego), których przekroczenie sygnalizowane jest diod¹ LED (wyga-
	szana podczas zerowania temperatur maksymalnej i minimalnej), zmiana ustawieñ wartoœci temperatur
	progowych mo¿liwa jest przy u¿yciu dwóch przycisków (z funkcj¹ autorepetycji) pozwalaj¹cych na ich
	zwiêkszanie lub zmniejszanie.
	- projekt powinien zawieraæ m.in. zadania realizuj¹ce: obs³ugê wyœwietlacza i diod LED (zadanie uaktywnia-
	ne przerwaniem pochodz¹cym od jednego z liczników/czasomierzy mikrokontrolera), obs³ugê klawiatury,
	pomiar temperatury bie¿¹cej, maksymalnej i minimalnej, obs³uga menu termometru; wymiana danych
	pomiêdzy zadaniami powinna siê odbywaæ z wykorzystaniem kolejek, semaforów
*/
/* Pod³¹czenie do procesora:
	PORTC - segmenty wyswietlacza LED
	PB0 ... PB3 - cyfry wyœwietlacza LED
	PA0 - dioda LED1 sygnalizuj¹ca wyœwietlanie temperatury bie¿¹cej
	PA1 - dioda LED2 sygnalizuj¹ca wyœwietlanie zarejestrowanej temperatury minimalnej
	PA2 - dioda LED3 sygnalizuj¹ca wyœwietlanie zarejestrowanej temperatury maksymalnej
	PA3 - dioda LED4 sygnalizuj¹ca ustawianie dolnego progu
	PA4 - dioda LED5 sygnalizuj¹ca ustawianie górnego progu
	PA5 - dioda LED6 sygnalizuj¹ca przekroczenie jednego z ustawionych progów
	PD0 - przycisk do zmiany, która temperatura ma byæ wyœwietlana
	PD1 - przycisk zerowania zarejestrowanych temperatur
	PD2 - przycisk do wejœcia do trybu ustawiania temperatur progowych
	PD3 - przycisk do zwiêkszania temperatury progowej
	PD4 - przycisk do zmniejszania temperatury progowej
	PD5 - magistrala 1-wire, czujnik temperatury DS18B20
*/


#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ds18x20.h"


///priorytet zadania do obs³ugi czujnika temperatury
#define DS18B20_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
///priorytet zadania obs³uguj¹cego diody led i przyciski
#define KEYS__LEDS_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )

#define NUMBER_OF_DIGITS 4
#define LED_digits		PORTB	
#define LED_D0			PB0	
#define LED_D1			PB1
#define LED_D2			PB2
#define LED_D3			PB3
#define LED_segments	PORTC

///przycisk do zmiany, która temperatura ma byæ wyœwietlana
#define KEY1	(1<<PD0)
///przycisk zerowania zarejestrowanych temperatur
#define KEY2	(1<<PD1)
///przycisk do wejœcia do trybu ustawiania temperatur progowych
#define KEY3	(1<<PD2)
///przycisk do zwiêkszania temperatury progowej
#define KEY4	(1<<PD3)
///przycisk do zmniejszania temperatury progowej
#define KEY5	(1<<PD4)

#define MODE_TEMP_ACT 0
#define MODE_TEMP_MIN 1
#define MODE_TEMP_MAX 2
#define MODE_TEMP_ALARM_MIN 3
#define MODE_TEMP_ALARM_MAX 4

#define LED_PORT PORTA
///dioda LED1 sygnalizuj¹ca wyœwietlanie temperatury bie¿¹cej
#define LED1 (1<<PA0)
///dioda LED2 sygnalizuj¹ca wyœwietlanie zarejestrowanej temperatury minimalnej
#define LED2 (1<<PA1)
///dioda LED3 sygnalizuj¹ca wyœwietlanie zarejestrowanej temperatury maksymalnej
#define LED3 (1<<PA2)
///dioda LED4 sygnalizuj¹ca ustawianie dolnego progu
#define LED4 (1<<PA3)
///dioda LED5 sygnalizuj¹ca ustawianie górnego progu
#define LED5 (1<<PA4)
///dioda LED6 sygnalizuj¹ca przekroczenie jednego z ustawionych progów
#define LED6 (1<<PA5)

/**
    Tablica konwersji wartosci BIN na kod wskaznika siedmiosegmentowego
 */
const uint8_t seg7[] PROGMEM = { 0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
	                             0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111};


/**
    Zmienne wykorzystywane do obslugi wyswietlacza LED
 */
volatile uint8_t LED_buf[NUMBER_OF_DIGITS];
volatile uint8_t LED_ptr;

uint8_t mode=MODE_TEMP_ACT,first_temp;	
int16_t temp_act, temp_min, temp_max, temp_alarm_min=500, temp_alarm_max=800;

///semafor ustawiany w przerwaniu co 2s, co powoduje rozpoczêcie pomiaru temperatury
static xSemaphoreHandle Tim2s;

///wyœwietlanie temperatury
static void prvDisplayTemp(int16_t val);
static void prvDisplayTemp(int16_t val)
{
	int16_t val_temp;
	if( val<0 ){
		LED_buf[3] = 0x80;
		val_temp = val * (-1);
	}
	else { 
		LED_buf[3] = 0; 
		val_temp = val;
		}
	if( val_temp<100 ){ LED_buf[2] = 0; }
		else{ LED_buf[2] = pgm_read_byte(&seg7[val_temp/100]); }
	LED_buf[1] = 0x80 | pgm_read_byte(&seg7[(val_temp/10)%10]);
	LED_buf[0] = pgm_read_byte(&seg7[val_temp%10]);
}


///inicjalizacja portów
static void prvInitHardware(void);
static void prvInitHardware(void)
{
	// porty wyswietlacza LED
	DDRB |= 0x0F;
	PORTB &= ~0x0F;
	DDRC = 0xFF;
	PORTC = 0x00;

	//pull up pinów pod³¹czonych do przycisków
	PORTD|=KEY1|KEY2|KEY3|KEY4|KEY5;

	//diody led
	DDRA|=LED1|LED2|LED3|LED4|LED5|LED6;
	LED_PORT |=(LED1|LED2|LED3|LED4|LED5|LED6);

	// inicjalizacja licznika Timer0 i przerwania F_CPU=16MHz
    TIMSK |= (1<<OCIE0);
    OCR0 = 30; //2ms
	// Prescaler = 1024 
    TCCR0 |= (1<<WGM01) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	
	//wykrycie czujnika na magistrali
	search_sensors(); 
}

///przerwanie do obs³ugi wyœwietlacza siedmiosegmentowego oraz odmierzania czasu
ISR(TIMER0_COMP_vect) 
{
	static uint16_t t = 0;
	
	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// obsluga odmierzania czasu 1s
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if (t++ > 500)
	{
		t = 0;
		xSemaphoreGiveFromISR(Tim2s, &xHigherPriorityTaskWoken);
	}
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// obsluga wyswietlacza siedmiosegmentowego LED
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ((++LED_ptr) > NUMBER_OF_DIGITS-1) LED_ptr = 0;
	LED_digits |= 0x0F;
	LED_segments = ~LED_buf[LED_ptr]; 
	LED_digits &=~(1<<LED_ptr);
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if (xHigherPriorityTaskWoken == pdTRUE) taskYIELD();
}

///obs³uga przycisków i diod LED
static void vTaskKeysLed(void *pvParameters);
static void vTaskKeysLed(void *pvParameters)
{
	static uint8_t KBD1,KBD2,press,time_switch_mode,step;
	for( ;; )
	{
			KBD1=PIND & 0x1F;
		if( (KBD1 !=0x1F) && (KBD1 == KBD2) ){
				if (press==0)
				{
					switch ((~KBD1)&0x1F)
					{
						case KEY1:	//zmiana wyœwietlanej temperatury
							if( mode==MODE_TEMP_ACT ){ mode = MODE_TEMP_MIN; time_switch_mode=60; }
							else{ mode=MODE_TEMP_ACT; time_switch_mode=0; }
						break;
						case KEY2:	//zerowanie zarejestrowanych temperatur
							temp_min=temp_act; temp_max=temp_act;
						break;
						case KEY3:	//wejœcie do trybu ustawiania progu dolnego, kolejne wciœniêcie - ustawianie progu górnego
							if( mode == MODE_TEMP_ALARM_MIN ){ mode = MODE_TEMP_ALARM_MAX; }
							else{ mode = MODE_TEMP_ALARM_MIN; }
						break;
					
								
					}
					press=1;
				}	
					if( (step==0) || (step>20) ){
						switch ((~KBD1)&0x1F)
						{			
							case KEY4:	//zmniejszenie wartoœci progowej
								if( mode == MODE_TEMP_ALARM_MIN ){ temp_alarm_min--; }
								if( mode == MODE_TEMP_ALARM_MAX ){ temp_alarm_max--; }
							break;
							case KEY5:	//zwiêkszenie wartoœci progowej
								if( mode == MODE_TEMP_ALARM_MIN ){ temp_alarm_min++; }
								if( mode == MODE_TEMP_ALARM_MAX ){ temp_alarm_max++; }
							break;						
						}						
					}
					if(step<21){ step++; }
				}
			
		if(KBD1==0x1F){ press=0; step=0; }
		KBD2=KBD1;
		
		if( time_switch_mode ){ time_switch_mode--; }
		else{
			if( mode == MODE_TEMP_MIN ){ mode=MODE_TEMP_MAX; time_switch_mode = 60; }
			else if( mode == MODE_TEMP_MAX ){ mode=MODE_TEMP_ACT; }
		}

		switch( mode ){
			case MODE_TEMP_ACT:
				LED_PORT&=~LED1;	LED_PORT |=LED2|LED3|LED4|LED5; prvDisplayTemp(temp_act);
			break;
			case MODE_TEMP_MIN:
				LED_PORT&=~LED2;	LED_PORT |=LED1|LED3|LED4|LED5; prvDisplayTemp(temp_min);
			break;
			case MODE_TEMP_MAX:
				LED_PORT&=~LED3;	LED_PORT |=LED1|LED2|LED4|LED5; prvDisplayTemp(temp_max);
			break;
			case MODE_TEMP_ALARM_MIN:
				LED_PORT&=~LED4;	LED_PORT |=LED1|LED2|LED3|LED5; prvDisplayTemp(temp_alarm_min);
			break;
			case MODE_TEMP_ALARM_MAX:
				LED_PORT&=~LED5;	LED_PORT |=LED1|LED2|LED3|LED4; prvDisplayTemp(temp_alarm_max);
			break;
		}
		if( (temp_act<temp_alarm_min) || (temp_act>temp_alarm_max) ){ LED_PORT&=~LED6; }
		else{ LED_PORT |=LED6; }

		vTaskDelay( 50 / portTICK_RATE_MS );
	}
}
///pomiar temperatury
static void vTaskMeasTemp(void *pvParameters);
static void vTaskMeasTemp(void *pvParameters)
{
	uint8_t sign, integer, fraction;
	for( ;; )
	{
		if (xSemaphoreTake(Tim2s, portMAX_DELAY)){
			DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );
			vTaskDelay( 1000/portTICK_RATE_MS );
			if( DS18X20_OK == DS18X20_read_meas(gSensorIDs[0], &sign, &integer, &fraction) ) {
				temp_act = ((uint16_t)integer*10) + fraction;
				if(sign==1){ temp_act*=-1; }
				if ( first_temp==0 )
				{
					temp_min=temp_act;
					temp_max=temp_act;
					first_temp=1;
				}
				else{
				if( temp_act < temp_min ){ temp_min = temp_act; }
				if( temp_act > temp_max ){ temp_max = temp_act; }					
				}

			}
			else {
			}	
		}
		
	}
}

/*-----------------------------------------------------------*/

void main(void)
{
	prvInitHardware();

	vSemaphoreCreateBinary(Tim2s);

	xTaskCreate( vTaskMeasTemp, 
	             (const int8_t*) "vTaskMeasTemp",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 DS18B20_TASK_PRIORITY,
				 NULL);
				 
	xTaskCreate( vTaskKeysLed,
				(const int8_t*) "vTaskKeysLed",
				configMINIMAL_STACK_SIZE,
				NULL,
				KEYS__LEDS_TASK_PRIORITY,
				NULL);

	vTaskStartScheduler();

	for(;;);
}
/*-----------------------------------------------------------*/
