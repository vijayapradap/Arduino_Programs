/*
 * File:   N_SPT.c
 * Author: Neuron RD
 *
 * Created on October 12, 2018, 3:50 PM
 */


// PIC32MX270F256B Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <p32xxxx.h>
#include <proc/p32mx270f256b.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/attribs.h>

#define SYSCLK 40000000L

//char sec,min,hour,hr,date,month,year;
//int ap, msgcount=0;

char gps_buff[500], gsm_buff[200], content[100], Location[50];
unsigned char gps=0, gsm=0, gsm_flag=0, gps_flag=0, rec=0, T_set=0;
char n_s, e_w;
unsigned char utctime[12], latitude[12], longitude[12];
float _latitude, _longitude, _lat_deg, _long_deg;

float meter, kilometer;
float f_speed, t_speed, s_speed;
static float overspeed = 50.0;
static float max_speed = 120.0;

char number[11], result[100];
char MD_Jio[11]={"7904649539"};
char SPT_No[11]={"9600209509"};
//char VSPT_No[11]={"8124255031"};
char VSPT_No[11]={"9865542903"};
char customer_number[11]={"9865542903"};

unsigned int dSec = 0, Sec = 0, Min = 0, Hour = 0;
static unsigned int S_Min = 30, S_Hour = 0, T_off=0;

//char atm_1=00, ath_1=00, atm_2=00, ath_2=00, atm_3=00, ath_3=00;
//char afl_1[20]={"00.0000,000.0000"}, afl_2[20]={"00.0000,000.0000"}, afl_3[20]={"00.0000,000.0000"};
//char atap_1='A', atap_2='A', atap_3='A', Fix_1=0, Fix_2=0, Fix_3=0;
//unsigned int ap_1=0, ap_2=0, ap_3=0, ampm=0, ch=0;

void __delay_ms(unsigned int x)
{
    T2CON = 0x8000;
    while(x--)
    {
        TMR2=0;
        while(TMR2<SYSCLK/1000);
    }
}
/*void I2CInit(void)
{
    I2C2CONbits.DISSLW = 1;
    I2C2CONbits.ON = 1;
    I2C2BRG = 0x02c;
}
void I2CStart()
{
    I2C2CONbits.SEN = 1;
    while(I2C2CONbits.SEN);
}
void I2CStop()
{
    I2C2CONbits.PEN = 1;
    while(I2C2CONbits.PEN);
}
void I2CRestart()
{
    I2C2CONbits.RSEN = 1;
    while(I2C2CONbits.RSEN);
}
void I2CAck()
{
    I2C2CONbits.ACKDT = 0;
    I2C2CONbits.ACKEN = 1;
    while(I2C2CONbits.ACKEN);
}
void I2CNak()
{
    I2C2CONbits.ACKDT = 1;
    I2C2CONbits.ACKEN = 1;
    while(I2C2CONbits.ACKEN);
}
void I2CSend(unsigned char dat)
{
    I2C2TRN = dat;
    while(I2C2STATbits.TRSTAT);
}
char I2CRead()
{
    I2C2CONbits.RCEN = 1;
    while(!I2C2STATbits.RBF);
    return I2C2RCV;
}
void RTC_Read()
{
    unsigned char I2CData1[20];
	unsigned char i;
	
	I2CStart();
	I2CSend(0xD0);
	I2CSend(0x00);
	I2CRestart();
	I2CSend(0xD1);
	for(i=8;i>0;i--)
	{
		I2CData1[i] = I2CRead();
			
        hour=I2CData1[6];
        hr=I2CData1[6] & 0x1F;
        ap=I2CData1[6] & 0x20;
        min=I2CData1[7];
        sec=I2CData1[8];

        date=I2CData1[4];
        month=I2CData1[3];
        year=I2CData1[2];

		if(i-1)
			I2CAck(); 
		else 
			I2CNak();
            
        if(ap)
            ampm=1;
        else
            ampm=0;
        
        if(hour==0x71 && min==0x59)
        {
            msgcount=0;
            Fix_1=1;
            Fix_2=1;
            Fix_3=1;
        }
	}
	I2CStop();
}
void RTC_Write(unsigned char min, unsigned char hour, unsigned char day, unsigned char date, unsigned char mon, unsigned char year)
{
    I2CStart();
	I2CSend(0xD0);
	I2CSend(0x00);
    I2CSend(0x00);
    I2CSend(min);
    I2CSend(hour);
    I2CSend(day);
    I2CSend(date);
    I2CSend(mon);
    I2CSend(year);
    I2CSend(0x00);
    I2CSend(0x00);
	I2CStop();
}*/
float speed_calc(float Lat1, float Long1, float Lat2, float Long2)
{
    kilometer=0.0;
    meter=0.0;
    
    float R=6371000;
    
	Lat1 = Lat1 * M_PI / 180.0;
	Long1 = Long1 * M_PI / 180.0;

	Lat2 = Lat2 * M_PI / 180.0;
	Long2 = Long2 * M_PI / 180.0;
    
    float _lat=(float)Lat2-Lat1;
    float _long=(float)Long2-Long1;
    
    float a=(float)sin(_lat/2)*sin(_lat/2);
    float b=(float)cos(Lat1)*cos(Lat2);
    float c=(float)sin(_long/2)*sin(_long/2);
    
    float abc=(float)a+b*c;
    
    float cba=(float)2*atan2(sqrt(abc),sqrt(1-abc));
    
    meter=(float)R*cba;
    kilometer=(float)meter/1000.0;
    
    unsigned int time_s = 5;
    float speed_mps=(float)meter / time_s;
    float speed_kmph=(float)speed_mps * 3600.0;
    return speed_kmph/1000.0;
}
float convert_to_degrees(float NMEA_lat_long)
{
    float minutes, dec_deg, decimal;
	int degrees;
    
    degrees = (int)(NMEA_lat_long/100.00);
    minutes = NMEA_lat_long - degrees*100.00;
    dec_deg = minutes / 60.00;
    decimal = degrees + dec_deg;
    return decimal;
}
void Get_GPS_Location()
{
    unsigned int i=0, j=0, k=0;
    
    memset(utctime,0,strlen(utctime));
    memset(gps_buff,0,strlen(gps_buff));
    memset(latitude,0,strlen(latitude));
    memset(longitude,0,strlen(longitude));
    __delay_ms(5);
    gps=0;
    
    CHECK:
    
    U2STASET = 0x1000;
    U2MODESET = 0x8000;
    IEC1SET = 0X400000;
    
    __delay_ms(1000);
    
    U2MODECLR = 0x8000;
    U2STACLR = 0x1000;
    IEC1CLR = 0X400000;
    
    if(strstr(gps_buff,"$GPGGA"))
    {
        i=0;
        while((gps_buff[i]!='\0') && (k==0))
        {
            if(gps_buff[i] == 'G')
            {
                i++;
                if(gps_buff[i] == 'G')
                {
                    i++;
                    if(gps_buff[i] == 'A')
                    {
                        i++;
                        if((gps_buff[i] == ',') && (k == 0))
                        {
                            i++;
                            j=0;
                            while(j<10)
                            {
                                utctime[j] = gps_buff[i];
                                i++;
                                j++;
                            }
                            i++;
                            j=0;
                            while(j<11)
                            {
                                latitude[j]=gps_buff[i];
                                i++;
                                j++;
                            }
                            i++;
                            j=0;
                            while(j<12)
                            {
                                longitude[j]=gps_buff[i];
                                i++;
                                j++;
                            }
                            k=1;
                        }
                    }
                }
            }
            i++;
        }
    }
    else if(strstr(gps_buff,"$GPRMC"))
    {
        i=0;
        while((gps_buff[i]!='\0') && (k==0))
        {
            if(gps_buff[i] == 'R')
            {
                i++;
                if(gps_buff[i] == 'M')
                {
                    i++;
                    if(gps_buff[i] == 'C')
                    {
                        i++;
                        if((gps_buff[i] == ',') && (k == 0))
                        {
                            i++;
                            j=0;
                            while(j<11)
                            {
                                utctime[j] = gps_buff[i];
                                i++;
                                j++;
                            }
                            i++;
                            i++;
                            j=0;
                            while(j<11)
                            {
                                latitude[j]=gps_buff[i];
                                i++;
                                j++;
                            }
                            i++;
                            i++;
                            j=0;
                            while(j<12)
                            {
                                longitude[j]=gps_buff[i];
                                i++;
                                j++;
                            }
                            k=1;
                        }
                    }
                }
            }
            i++;
        }
    }
    else
    {
        gps=0;
        memset(gps_buff,0,strlen(gps_buff));
        __delay_ms(5);
        goto CHECK;
    }
    
    if(strstr(latitude,",N"))
        n_s='N';
    if(strstr(latitude,",S"))
        n_s='S';
    if(strstr(longitude,",E"))
        e_w='E';
    if(strstr(longitude,",W"))
        e_w='W';
    
    if(strstr(latitude,",,") || strstr(longitude,",,"))
    {
        _latitude = 0;
        _longitude = 0;
    }
    else
    {
        _latitude = atof(latitude);
        _longitude = atof(longitude);
    }
}
void UART2_Init()
{
    U2RXR = 2;                                      // RB1
    
    U2MODE = 0;
    U2STA = 0x1000;
    U2BRG = 259;
    U2MODESET = 0x8000;
    
    IEC1bits.U2RXIE = 0;
    IFS1bits.U1RXIF = 0;
    IPC9bits.U2IP = 0;
    IPC9bits.U2IP = 1;
    IPC9bits.U2IS = 2;
    
    U2MODECLR = 0x8000;
    U2STACLR = 0x1000;
    IEC1CLR = 0X400000;
}

void timer_init()
{
    T1CON = 0x8030;
    PR1 = 31250;
    
    IEC0CLR = 0x0010;
    IFS0CLR = 0x0010;
    IPC1CLR = 0x001f;
    IPC1SET = 0x0010;
    IPC1SET = 0x0000;
    IEC0SET = 0x0010;
}
void UART1_Init()
{
    U1RXR = 3;                                      // RB13
    RPB15R = 1;                                     // RB15
    
    U1MODE = 0;
    U1STA = 0x1400;
    U1BRG = 259;
    U1MODESET = 0x8000;
    
    IEC1CLR = 0X0100;
    IFS1bits.U1RXIF = 0;
    IPC8CLR = 0x001f;
    IPC8SET = 0x0010;
    IPC8SET = 0x0001;
    IEC1SET = 0X0100;
}
void U1_Serial_write(char ch)
{
    U1STAbits.UTXEN = 1;    
    while(U1STAbits.UTXBF);
    U1TXREG = ch;
}
void U1_Serial_print(unsigned char *string)
{    
    while(*string!='\0')
    {
       U1_Serial_write(*string++);
    }
}
void U1_Serial_println(char *string)
{
   U1_Serial_print(string);
   U1_Serial_write(0x0D);
   U1_Serial_write(0x0A);
}
void U1_Setup_messaging()
{
    unsigned int i;
    
    for(i=0;i<5;i++)
    {   
        U1_Serial_println("AT");
        __delay_ms(1000);
        if(strstr(gsm_buff,"OK"))
        {
            memset(gsm_buff,0,strlen(gsm_buff));
            break;
        }
        else
        {
            memset(gsm_buff,0,strlen(gsm_buff));
            gsm=0;
        }
    }
    
    U1_Serial_println("AT+IPR=9600");
    __delay_ms(1000);
    U1_Serial_println("AT+CMGDA=\"DEL ALL\"");
   __delay_ms(1000);
    U1_Serial_println("AT+CLIP=1");
    __delay_ms(2000);
}
void U1_Start_message(char *Phone_number)
{
    U1_Serial_println("AT+CMGF=1");
    __delay_ms(2000);
    U1_Serial_print("AT+CMGS=\"0");
    U1_Serial_print(Phone_number);
    U1_Serial_print("\"");
    U1_Serial_write(0x0D);
    __delay_ms(1000);
}
void U1_Send_content(char *content)
{
    U1_Serial_print(content);
}
void U1_Send_message(char *content)
{
    U1_Serial_print(content);
    U1_Serial_write(0x1A);
}
void U1_Delete_SMS()
{
    U1_Serial_println("AT+CMGDA=\"DEL ALL\"");
   __delay_ms(1000);
}
void U1_Msg_Received()
{
    memset(number,0,strlen(number));
    unsigned int i=0, j=0;
    gsm=0;
    
    U2MODECLR = 0x8000;
    U2STACLR = 0x1000;
    IEC1CLR = 0X400000;
    
    if(strstr(gsm_buff,"RING"))
    {
        U1STACLR = 0x1000;
        IEC1CLR = 0X0100;
        
        for(i=21,j=0;i<=30;i++,j++)
            number[j]=gsm_buff[i];
        
        if(strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No))
        {
            U1_Serial_println("ATH");
            __delay_ms(2000);
            
            rec=1;
        }
        else
        {
            U1_Serial_println("ATH");
            __delay_ms(2000);
        }
    }
    if(strstr(gsm_buff,"CMT"))
    {
        gsm=0;
        
        U1_Serial_println("AT+CMGR=1");
        __delay_ms(1000);
        
        U1STACLR = 0x1000;
        IEC1CLR = 0X0100;
        
        for(i=36,j=0;j<10;i++,j++)
            number[j]=gsm_buff[i];
        
        for(i=75,j=0;j<100;i++,j++)
            result[j]=gsm_buff[i];
        
        __delay_ms(10);
        
        if((strstr(result,"SPTLOC")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {            
            rec=1;
        }
        else if(strstr(result,"PTW") && (strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            memset(customer_number,0,strlen(customer_number));
            __delay_ms(10);
            
            for(i=3,j=0;j<10;i++,j++)
                customer_number[j]=result[i];
            
            sprintf(content,"CUSTOMER NUMBER IS: %s",customer_number);
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(3000);
            
            U1_Start_message(customer_number);
            U1_Send_message("WELCOME TO NEURON FAMILY");
            __delay_ms(1000);
        }
        else if((strstr(result,"TON")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            S_Hour=((result[3]-0x30)*10)+((result[4]-0x30)*1);
            S_Min=((result[5]-0x30)*10)+((result[6]-0x30)*1);
            Hour=0; Min=0;
            
            T_off=0;
            
            sprintf(content,"AUTO FREQUENT LOCATION MESSAGE TIME INTERVAL: %d HOURS, %d MINUTES",S_Hour,S_Min);
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(1000);
        }
        else if((strstr(result,"TOFF")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            U1_Start_message(number);
            U1_Send_message("AUTO FREQUENT LOCATION MESSAGE TIME INTERVAL OFF");
            __delay_ms(1000);
            
            T_off=1;
        }
        else if((strstr(result,"SPD")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            if(f_speed!=0 && f_speed<max_speed)
                sprintf(content,"Now Your Vehicle Speed is : %.3f km/hour",f_speed);
            else if(t_speed!=0 && t_speed<max_speed)
                sprintf(content,"Now Your Vehicle Speed is : %.3f km/hour",t_speed);
            else
                strcpy(content,"Now Your Vehicle Speed is : 00.000 km/hour");
            
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(1000);
        }
        else if((strstr(result,"OSS")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            unsigned char val[5];
            
            for(i=0,j=3;j<6;i++,j++)
                val[i]=result[j];
            __delay_ms(10);
            
            overspeed=atof(val);
            
            sprintf(content,"Over Speed Set : %.3f km/hour\r",overspeed);
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(1000);
        }
        else if((strstr(result,"SET")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            sprintf(content,"1. Over Speed Set : %.3f km/hour\r\r2. Auto Frequent Location Time : %d Hours %d Minutes",overspeed,S_Hour,S_Min);
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(1000);
        }
        /*else if((strstr(result,"TD")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {           
            unsigned char time[10],date[10];
            for(i=0,j=2;j<6;i++,j++)
                time[i]=result[j]-0x30;
            
            for(i=0,j=7;j<14;i++,j++)
                date[i]=result[j]-0x30;
            
            I2CStart();
            I2CSend(0xD0);
            I2CSend(0x00);
            I2CSend(0x00);
            I2CSend(((time[2]<<4)+time[3]));
            if(result[6]=='A')
            {
                I2CSend(((time[0]<<4)+time[1])|0x40);
            }
            if(result[6]=='P')
            {
                I2CSend(((time[0]<<4)+time[1])|0x60);
            }
            I2CSend(0x01);
            I2CSend(((date[0]<<4)+date[1]));
            I2CSend(((date[2]<<4)+date[3]));
            I2CSend(((date[4]<<4)+date[5]));
            I2CSend(0x00);
            I2CSend(0x00);
            I2CStop();
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message("TIME AND DATE SETTNG COMMAND RECEIVED");
            __delay_ms(1000);
        }
        else if((strstr(result,"AT")) && (strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            unsigned char val_1[4];
            unsigned char val_2[4];
            unsigned char val_3[4];
            
            i=0,j=0;
            
            for(i=0,j=2;j<6;i++,j++)
                val_1[i]=result[j]-0x30;
            
            atap_1=result[6];
            
            ath_1=((val_1[0]*10)+val_1[1]);
            atm_1=((val_1[2]*10)+val_1[3]);
            
            if(atap_1=='P')
                ap_1=1;
            if(atap_1=='A')
                ap_1=0;
            
            for(i=0,j=7;j<11;i++,j++)
                val_2[i]=result[j]-0x30;
            
            atap_2=result[11];
            
            ath_2=((val_2[0]*10)+val_2[1]);
            atm_2=((val_2[2]*10)+val_2[3]);
            
            if(atap_2=='P')
                ap_2=1;
            if(atap_2=='A')
                ap_2=0;
            
            for(i=0,j=12;j<16;i++,j++)
                val_3[i]=result[j]-0x30;
            
            atap_3=result[16];
            
            ath_3=((val_3[0]*10)+val_3[1]);
            atm_3=((val_3[2]*10)+val_3[3]);
            
            if(atap_3=='P')
                ap_3=1;
            if(atap_3=='A')
                ap_3=0;
            
            sprintf(content,"ADVANDCED FIXED TIME SETTING COMMAND RECEIVED\rTIME_1: %d:%d %cM\rTIME_2: %d:%d %cM\rTIME_3: %d:%d %cM",ath_1,atm_1,atap_1,ath_2,atm_2,atap_2,ath_3,atm_3,atap_3);
            __delay_ms(50);
            
            Fix_1=1,Fix_2=1,Fix_3=1;
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(1000);
        }
        else if((strstr(result,"AFL")) && (strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            for(i=0,j=5;j<23;i++,j++)
                afl_1[i]=result[j];
                
            for(i=0,j=25;j<43;i++,j++)
                afl_2[i]=result[j];
            
            for(i=0,j=45;j<63;i++,j++)
                afl_3[i]=result[j];
                
            __delay_ms(10);
            
            U1_Start_message(number);
            U1_Send_message("ADVANCED FIXED LOCATION SETTING COMMAND RECEIVED");
            __delay_ms(1000);
        }
        else if((strstr(result,"SS")) && (strstr(number,customer_number) || strstr(number,MD_Jio) || strstr(number,SPT_No) || strstr(number,VSPT_No)))
        {
            char content[200];
            sprintf(content,"ADVANCED FIXED TIME SETTINGS\rTIME_1: %d:%d%cM\rTIME_2: %d:%d%cM\rTIME_3: %d:%d%cM\rADVANCED FIXED LOCATION SETTINGS\rLOC_1: %s\rLOC_2: %s\rLOC_3: %s",ath_1,atm_1,atap_1,ath_2,atm_2,atap_2,ath_3,atm_3,atap_3,afl_1,afl_2,afl_3);
            __delay_ms(20);
            
            U1_Start_message(number);
            U1_Send_message(content);
            __delay_ms(1000);
        }*/
        else
        {
            __delay_ms(100);
        }
    }
    
    U1_Delete_SMS();
    
    U1STASET = 0x1000;
    IEC1SET = 0X0100;
    
    memset(result,0,strlen(result));
    memset(gsm_buff,0,strlen(gsm_buff));
    gsm=0;
}
void __ISR(_TIMER_1_VECTOR, IPL4SOFT) Timer1IntHandler(void)
{
	dSec++;
    
	if(dSec > 4)
	{
        dSec = 0;
        Sec++;
        
        if(Sec > 59)
        {
            Sec = 0;
            Min++;
            
            if(Min > 59)
            {
                Min = 0;
                Hour++;
            }
        }
    }
    IFS0CLR = 0x0010;
}
void __ISR(_UART_1_VECTOR,IPL5SOFT) UART1InterruptHandler(void)
{
    if (IFS1bits.U1RXIF == 1)
    {
        while(!U1STAbits.URXDA);
        gsm_buff[gsm] = U1RXREG;
        gsm++;

        if( (U1STAbits.OERR == 1) || (U1STAbits.PERR == 1) || (U1STAbits.FERR == 1) )
        {
            U1STAbits.OERR = 0;
            U1STAbits.PERR = 0;
            U1STAbits.FERR = 0;
        }
        IFS1bits.U1RXIF = 0;
        gsm_flag = 1;
    }
}
void __ISR(_UART_2_VECTOR,IPL6SOFT) UART2InterruptHandler(void)
{
    if (IFS1bits.U2RXIF == 1)
    {
        while(!U2STAbits.URXDA);
        gps_buff[gps] = U2RXREG;
        gps++;

        if( (U2STAbits.OERR == 1) || (U2STAbits.PERR == 1) || (U2STAbits.FERR == 1) )
        {
            U2STAbits.OERR = 0;
            U2STAbits.PERR = 0;
            U2STAbits.FERR = 0;
        }
        IFS1bits.U2RXIF = 0;
    }
}
void main()
{
    ANSELA=0;
    ANSELB=0;
    CM1CON=0;
    CM2CON=0;
    CM3CON=0;
    TRISA=0x0000;
    TRISB=0x2012;
    PORTA=0x0000;
    PORTB=0x0000;
    
    char latbuff[15], longbuff[15], loc[30];
    unsigned int i=0, j=0, Ocs=0, EMG=0, m_msg=1;
    float Lat1 = 0.0, Long1 = 0.0, Lat2 = 0.0, Long2 = 0.0, Lat3 = 0.0, Long3 = 0.0;
    
    //I2CInit();
    
    INTEnableSystemMultiVectoredInt();
    
    UART1_Init();
    UART2_Init();
    
    //RTC_Write(0x00, 0x49, 0x05, 0x27, 0x09, 0x18);
    
    U1_Setup_messaging();
    
    U1_Start_message(customer_number);
    U1_Send_message("WELCOME TO NEURON FAMILY");
    __delay_ms(1000);
    
    timer_init();
    
    while(1)
    {
        //RTC_Read();
        
        Get_GPS_Location();
        
        Lat1=convert_to_degrees(_latitude);
        Long1=convert_to_degrees(_longitude);
        
        if(PORTBbits.RB5==1)
            EMG=1;
        
        if(gsm_flag==1)
        {
            U1_Msg_Received();
            gsm_flag=0;
        }
        else
            __delay_ms(5000);
        
        Get_GPS_Location();
        
        Lat2=convert_to_degrees(_latitude);
        Long2=convert_to_degrees(_longitude);
        
        if(PORTBbits.RB5==1)
            EMG=1;
        
        if(Lat1!=0 && Long1!=0 && Lat2!=0 && Long2!=0)
        {
            f_speed=0;
            
            f_speed = speed_calc(Lat1,Long1,Lat2,Long2);
            
            if(f_speed<overspeed && t_speed>overspeed && Ocs==1)
            {
                U1_Start_message(customer_number);
                U1_Send_message("Now Your Vehicle Speed is Below Over Speed Limit");
                __delay_ms(2000);
                Ocs=0;
            }
            else if(gsm_flag==1)
            {
                U1_Msg_Received();
                gsm_flag=0;
            }
            else
                __delay_ms(5000);
        }
        else
        {
            if(gsm_flag==1)
            {
                U1_Msg_Received();
                gsm_flag=0;
            }
            else
                __delay_ms(5000);
        }
        
        Get_GPS_Location();
        
        Lat3=convert_to_degrees(_latitude);
        Long3=convert_to_degrees(_longitude);
        
        if(PORTBbits.RB5==1)
            EMG=1;
        
        if(Lat2!=0 && Long2!=0 && Lat3!=0 && Long3!=0)
        {
            t_speed=0;
            
            t_speed = speed_calc(Lat2,Long2,Lat3,Long3);
            
            if(f_speed>overspeed && t_speed>overspeed && t_speed<max_speed && Ocs==0)
            {
                Ocs=1;
                sprintf(content,"Alert!\r\r\rNow Your Vehicle Speed is above Over Speed Limit\r\rNow Your Vehicle Speed is: %.3f km/hour",t_speed);
                __delay_ms(10);
                
                U1_Start_message(customer_number);
                U1_Send_message(content);
                __delay_ms(2000);
            }
            if(t_speed<overspeed && t_speed<max_speed && Ocs==1)
            {
                U1_Start_message(customer_number);
                U1_Send_message("Now Your Vehicle Speed is Below Over Speed Limit");
                __delay_ms(2000);
                Ocs=0;
            }
        }
        
        int a_1, b_1, c_1, d_1, e_1, f_1;
        int la_1, la_2;
        int a_2, b_2, c_2, d_2, e_2, f_2;
        int lo_1, lo_2, lo_3, lo_4;

        int integer_lat=(int)Lat3;
        int fract_int_lat=((int)(Lat3*10000)%10000);

        la_1=integer_lat/10;
        la_2=integer_lat%10;
        
        a_1=fract_int_lat%10;
        b_1=fract_int_lat/10;
        c_1=b_1%10;
        d_1=b_1/10;
        e_1=d_1%10;
        f_1=d_1/10;

        int integer_long=(int)Long3;
        int fract_int_long=((int)(Long3*10000)%10000);

        lo_1=integer_long/10;
        lo_2=integer_long%10;
        lo_3=lo_1%10;
        lo_4=lo_1/10;
        
        a_2=fract_int_long%10;
        b_2=fract_int_long/10;
        c_2=b_2%10;
        d_2=b_2/10;
        e_2=d_2%10;
        f_2=d_2/10;
        
        sprintf(Location,"https://www.google.co.in/maps/place/%d%d.%d%d%d%d%c,%d%d%d.%d%d%d%d%c",la_1,la_2,f_1,e_1,c_1,a_1,n_s,lo_4,lo_3,lo_2,f_2,e_2,c_2,a_2,e_w);
        __delay_ms(5);
        
        if((strstr(Location,"00.") || strstr(Location,"000.")) || (!strstr(Location,"N") && !strstr(Location,"S")) || (!strstr(Location,"E") && !strstr(Location,"W")))
            strcpy(Location,"https://www.google.co.in/maps/place/00.0000N,000.0000E");
        __delay_ms(5);
        
        if(gsm_flag==1)
        {
            U1_Msg_Received();
            gsm_flag=0;
        }
        if(PORTBbits.RB5==1 || EMG==1)
        {
            EMG=0;
            U1_Start_message(customer_number);
            U1_Send_content("Alert!\r\rYour Child is in Problem,\r\rNow Your Child is Available at:\r\r");
            U1_Send_message(Location);
            __delay_ms(1000);
        }
        if((Hour == S_Hour && Min == S_Min && T_off==0) || (rec == 1) || (Min == m_msg))
        {
            if((Hour == S_Hour && Min == S_Min && rec == 0) || (Min == m_msg))
            {
                U1_Start_message(customer_number);
                U1_Send_message(Location);
                __delay_ms(1000);
                Min = 0;
                Hour = 0;
                m_msg = 60;
            }
            else
            {
                U1_Start_message(number);
                U1_Send_message(Location);
                __delay_ms(1000);
                rec=0;
            }
        }
        /*if(Fix_1 == 1 || Fix_2 == 1 || Fix_3 == 1)
        {
            sprintf(latbuff,"%d.%d%d%d",integer_lat,f_1,e_1,c_1);
            __delay_ms(5);

            sprintf(longbuff,"%d.%d%d%d",integer_long,f_2,e_2,c_2);
            __delay_ms(5);
        }
        if(ath_1 == hr && atm_1 == min && ap_1 == ap && Fix_1 == 1)
        {
            if(strstr(afl_1,latbuff) && strstr(afl_1,longbuff))
                Fix_1=0;
            else
            {
                U1_Start_message(customer_number);
                U1_Send_message("Not Reached First Fixed Location");
                __delay_ms(1000);
                Fix_1=0;
            }
        }
        if(ath_2 == hr && atm_2 == min && ap_2 == ap && Fix_2 == 1)
        {
            if(strstr(afl_2,latbuff) && strstr(afl_2,longbuff))
                Fix_2=0;
            else
            {
                U1_Start_message(customer_number);
                U1_Send_message("Not Reached Second Fixed Location");
                __delay_ms(1000);
                Fix_2=0;
            }
        }
        if(ath_3 == hr && atm_3 == min && ap_3 == ap && Fix_3 == 1)
        {
            if(strstr(afl_3,latbuff) && strstr(afl_3,longbuff))
                Fix_3=0;
            else
            {
                U1_Start_message(customer_number);
                U1_Send_message("Not Reached Third Fixed Location");
                __delay_ms(1000);
                Fix_3=0;
            }
        }*/
    }
}