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

#include <p32xxxx.h>
#include <proc/p32mx270f256b.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/attribs.h>

#define SYSCLK 40000000

char gps_buff[500], gsm_buff[200], content[100], Location[50];
unsigned char gps=0, gsm=0, gsm_flag=0, gps_flag=0, rec=0, T_set=0;
char n_s=0, e_w=0;
unsigned char utctime[12], latitude[12], longitude[12];
float _latitude, _longitude, _lat_deg, _long_deg;

float meter, kilometer;
float f_speed, t_speed, s_speed;
static float overspeed = 50.0;
static float max_speed = 120.0;

char number[11], result[100];
char MD_Jio[11]={"7904649539"};
char SPT_No[11]={"9600209509"};
char VSPT_No[11]={"8124255031"};
//char VSPT_No[11]={"9865542903"};
char customer_number[11]={"7904649539"};

unsigned int dSec = 0, Sec = 0, Min = 0, Hour = 0;
static unsigned int S_Min = 30, S_Hour = 0, T_off=0;

void __delay_ms(unsigned int x)
{
    T2CON = 0x8000;
    while(x--)
    {
        TMR2=0;
        while(TMR2<SYSCLK/1000);
    }
}
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
    
    __delay_ms(500);
    
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
        memset(gps_buff,0,strlen(gps_buff));
        memset(latitude,0,strlen(latitude));
        memset(longitude,0,strlen(longitude));
        gps=0;
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
    TRISA=0x0000;
    TRISB=0x2012;
    PORTA=0x0000;
    PORTB=0x0000;
	
    unsigned int i=0, j=0, Ocs=0, EMG=0, m_msg=1;
    float Lat1 = 0.0, Long1 = 0.0, Lat2 = 0.0, Long2 = 0.0, Lat3 = 0.0, Long3 = 0.0;
    
    INTEnableSystemMultiVectoredInt();
    
    UART1_Init();
    UART2_Init();
    
    U1_Setup_messaging();
    
    U1_Start_message(customer_number);
    U1_Send_message("WELCOME TO NEURON FAMILY");
    __delay_ms(1000);
    
    timer_init();
    
    while(1)
    {
        if(PORTBbits.RB5==1)
            EMG=1;
        
        Get_GPS_Location();
        
        Lat1=convert_to_degrees(_latitude);
        Long1=convert_to_degrees(_longitude);
        
        if((Lat1>=90 && Lat1>0) || (Long1>180 && Long1>0))
        {
            Lat1=0;
            Long1=0;
        }
        
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
        
        if((Lat2>=90 && Lat2>0) || (Long2>180 && Long2>0))
        {
            Lat2=0;
            Long2=0;
        }
        
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
        
        if((Lat3>=90 && Lat3>0) || (Long3>180 && Long3>0))
        {
            Lat3=0;
            Long3=0;
        }
        
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
        
        int a,e;
        a=(int)(Long3);
        e=a/100;
            
        if(e==0)
            sprintf(Location,"https://www.google.co.in/maps/place/%.4f%c,%d%.4f%c",Lat3,n_s,e,Long3,e_w);
        else
            sprintf(Location,"https://www.google.co.in/maps/place/%.4f%c,%.4f%c",Lat3,n_s,Long3,e_w);
        __delay_ms(5);
            
        if((strstr(Location,"000.") || strstr(Location,"00.")) || (((!strstr(Location,"N")) && (!strstr(Location,"S"))) || ((!strstr(Location,"E")) && (!strstr(Location,"W")))))
            strcpy(Location,"https://www.google.co.in/maps/place/00.0000,000.0000");
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
                m_msg = 70;
            }
            else
            {
                U1_Start_message(number);
                U1_Send_message(Location);
                __delay_ms(1000);
                rec=0;
            }
        }
    }
}
