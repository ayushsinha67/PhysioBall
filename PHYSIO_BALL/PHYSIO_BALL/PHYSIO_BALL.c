/*
 * PHYSIOBALL
 * Manipal Institute of Technology, 2012
 * Authors: Ayush, Rajanya, Abhijit
 */

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include "adc.h"
#include "adc.c"
#include "lcd_hd44780.h"
#include "lcd_hd44780.c"

#define CHK(x,b) (x&b)
#define CLR(x,b) (x&=~b)
#define SET(x,b) (x|=b)
#define TOG(a,b) (a^=b)

#define PORT_BUZZER     PORTD
#define PIN_BUZZER      PD0

#define PORT_BUTTONS    PINC
#define INC             PC0
#define DEC             PC1
#define START           PC2
#define STOP            PC3
#define DATA			PC4

#define BUZZ(x)		SET( PORT_BUZZER, 1<<PIN_BUZZER ); _delay_ms(x); CLR( PORT_BUZZER, 1<<PIN_BUZZER );

/* GLOBAL SRAM VARIABLES */
uint16_t val1, val2, val3;
uint8_t i = 0;
uint8_t ch = 0;
uint8_t thres = 0;
uint16_t count = 0;
uint16_t sram_count[6] = {0,0,0,0,0,0};
uint16_t sram_thres[6] = {0,0,0,0,0,0};

/* EEPROM VARIABLES */
uint16_t EEMEM ee_count[6];
uint16_t EEMEM ee_thres[6];

/* FUNCTION PROTOTYPES */
void StartScreen(void);
void SetThreshold(void);
void Counter(void);
void BarGraph( uint8_t max );
void GetData();
void WriteData( uint16_t new_count, uint16_t new_thres);
void DispData();

/* -------------- MAIN ------------------- */
int main(void)
{
	/* Enable Watchdog Timer */
	//wdt_enable(WDTO_1S);	// 1 Sec WDT	
	
	/* Initialize PORTS */ 
    DDRA &= ~(1<<PA0);  // ADC PORT
    DDRD |= (1<<0);     // Buzzer
    DDRC = 0x00;        // Input Pins
    PORTC = 0xFF;       // Pull Up

    /* Initialize LCD */
    lcd_init(LCD_DISP_ON);
    lcd_clrscr(); 	
	
	/* Initialize LCD */
	Init_ADC();
	
	wdt_reset();	// Reset WDT 
   
    while(1){
		
		StartScreen();	// Menu
		BUZZ(200);
		
		if( ch == 1){
			
			SetThreshold();
			BUZZ(200);
			Counter();		
		}
		
		else if( ch == 2 ){
			
			GetData();	// EEPROM read
			DispData();		
		}
	
		wdt_reset();	// Reset WDT
		
    }

    return 0;
}

/* ------------ FUNCTIONS -------------------- */
void StartScreen(void)
{
	BUZZ(200);
	lcd_home();
	lcd_clrscr();
	lcd_sets(0,0,"PhysioBall");
	ch = 0;	// Reset Choice
	i = 0;
	
	do{	/* Menu Selection Routine */

		if( i <= 5){
			_delay_ms(200);
			lcd_command(LCD_MOVE_DISP_RIGHT );
		}
		else if( i > 5 && i <= 11 ){
			_delay_ms(200);
			lcd_command(LCD_MOVE_DISP_LEFT);
		}
		else{
			i = -1;
			lcd_home();
		}
		
		i++;
		
		if( !CHK( PORT_BUTTONS, 1<<START ) )	// Check User Selection
			ch = 1;
		else if( !CHK( PORT_BUTTONS, 1<<DATA ) )
			ch = 2;
		
		wdt_reset();	// Reset WDT
			
	} while( ch==0 );	
	
	lcd_home();
}
//-----------------------------------------------
void SetThreshold(void)
{

    while( CHK( PORT_BUTTONS, 1<<START) ){ // Wait till start button is pressed

        lcd_sets(0,0,"THRESHOLD:      ");
        lcd_set_int(11,0,thres); // Display Threshold value
        lcd_setc(14,0,'%');

        if( !CHK( PORT_BUTTONS, 1<<INC) && (thres<100) ){   // INCREMENT BUTTON PRESSED
            _delay_ms(1);
            thres++;
        }

        if( !CHK( PORT_BUTTONS, 1<<DEC) && (thres>0) ){   // INCREMENT BUTTON PRESSED
            _delay_ms(1);
            thres--;
        }

        BarGraph( (uint8_t)( ( (float)thres / 100.0 ) * 16.0 ) );  // BAR GRAP
		
		wdt_reset();	// Reset WDT
    }
}

//--------------------------------------

void Counter(void)
{
    count = 0;
    uint8_t chk = 0;

    while( CHK( PORT_BUTTONS, 1<<STOP) ){ // Wait till start button is pressed

        val1 = ADC_read(0); // Read from channel 0
        val2 = (uint16_t)( ( (float)val1 / 1023.0 ) * 100.0 ); // in terms on 100

        BarGraph( (uint8_t)( ( (float)val1 / 1023.0 ) * 16.0 ) );  // BAR GRAP

        lcd_sets(0,0,"COUNT:           ");
        lcd_set_int(7,0,count);       // Display Count value
        lcd_set_int(12,0,val2);
        lcd_setc(15,0,'%');

        if( chk == 1 ){				// Prevent Repeated Count above threshold
            if( val2 >= thres )
                continue;
            else
                chk = 0;
        }

        else if( val2 >= thres){
            BUZZ(40);
            chk = 1;
            count++; // Increment Counter
        }
		
		wdt_reset();	// Reset WDT
    }
	
	GetData();
	WriteData(count, thres);

}

//---------------------------------------
void BarGraph( uint8_t max)
{
    uint8_t j;

    for( j = 0; j<16; j++ ){
        if( j<max ){
            lcd_setc(j,1,(char)0xFF);
        }

        else{
            lcd_setc(j,1,' ');
        }
    }
}

// ---------------------------------------
void GetData()
{
	eeprom_read_block((void*)&sram_count, (const void*)&ee_count, 6); // Read count from EEPROM
	eeprom_read_block((void*)&sram_thres, (const void*)&ee_thres, 6); // Read thres from EEPROM
}

//---------------------------------------
void WriteData( uint16_t new_count, uint16_t new_thres )
{
	uint8_t j;
	for( j = 5; j>0; j-- ){
		sram_count[j] = sram_count[j-1];	 // Shift count data 	
	}
	
	for( j = 5; j>0; j-- ){
		sram_thres[j] = sram_thres[j-1];	 // Shift thres data 	
	}
	
	sram_count[0] = new_count;	 			 // Write new count
	sram_thres[0] = new_thres;				 // Write new threshold	
	
	eeprom_write_block((const void*)&sram_count, (void*)&ee_count, 6);	// Write count to EEPROM
	eeprom_write_block((const void*)&sram_thres, (void*)&ee_thres, 6);	// Write thres to EEPROM
}

//------------------------------------------
void DispData()
{
	uint8_t screen = 1;
	lcd_clrscr();
	
	while( CHK( PORT_BUTTONS, 1<<STOP) ){
	
		if( screen == 1 ){			// Data: 1-3
			
			lcd_setc(15,1,0x3E);			// display pointer
			lcd_set_int(1,1,sram_count[0]); // display count
			lcd_set_int(6,1,sram_count[1]);
			lcd_set_int(11,1,sram_count[2]);
			lcd_set_int(1,0,sram_thres[0]);	// display threshold
			lcd_set_int(6,0,sram_thres[1]);
			lcd_set_int(11,0,sram_thres[2]);	
			lcd_setc(3,0,'%');				// display percent	
			lcd_setc(8,0,'%');
			lcd_setc(13,0,'%');		
		}	
		
		else if( screen == 2 ){		// Data: 4-6
			
			lcd_setc(15,1,0x3E);			// display pointer
			lcd_setc(0,1,0x3C);
			lcd_set_int(1,1,sram_count[3]);	// display count
			lcd_set_int(6,1,sram_count[4]);
			lcd_set_int(11,1,sram_count[5]);	
			lcd_set_int(1,0,sram_thres[3]);	// display threshold
			lcd_set_int(6,0,sram_thres[4]);
			lcd_set_int(11,0,sram_thres[5]);		
			lcd_setc(3,0,'%');				// display percent	
			lcd_setc(8,0,'%');
			lcd_setc(13,0,'%');
		}
		
		if ( !CHK( PORT_BUTTONS, 1<<INC) && ( screen < 2 ) ){
			_delay_ms(10);
			BUZZ(40);
			screen++;
			lcd_clrscr();
		}
		
		else if ( !CHK( PORT_BUTTONS, 1<<DEC) && ( screen > 1 ) ){
			_delay_ms(10);
			BUZZ(40);
			screen--;
			lcd_clrscr();
		}
		
		wdt_reset();	// reset WDT
		
	}	
	
}