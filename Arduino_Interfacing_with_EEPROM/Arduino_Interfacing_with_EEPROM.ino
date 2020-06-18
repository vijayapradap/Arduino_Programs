#define TSTBY   600               //Standby pulse time 600 uSec
#define TSS     10                //Start header setup time 10 uSec
#define TWC_W   5                 //Write cycle time (byte or page) Write, WRSR commands 5 mSec
#define TE      20                //Bit Period 10 to 100 uSec
#define THDR    5                 //Start header low pulse time 5 uSec

const char IO_Pin = 2;            //IO Pin for Write and Read
const char READ = 0x03;           //Read data from memory array beginning at specified address
const char CRRD = 0x06;           //Read data from current location in memory array
const char WRITE = 0x6C;          //Write data to memory array beginning at specified address
const char WREN = 0x96;           //Set the write enable latch (enable write operations)
const char WRDI = 0x91;           //Reset the write enable latch (disable write operations)
const char RDSR = 0x05;           //Read STATUS register
const char WRSR = 0x6E;           //Write STATUS register
const char ERAL = 0x6D;           //Write ‘ 0x00 ’ to entire array
const char SETAL = 0x67;          //Write ‘ 0xFF ’ to entire array
const char dev_addr = 0xA0;       //Device address
const char str_hdr = 0x55;        //Start Header command

/* 
 *  EEPROM Acknowledgement Master Sent HIGH and Slave Sent HIGH Pulse
 */
inline void Ack_MAK_SAK() {
  pinMode(IO_Pin, OUTPUT);
  digitalWrite(IO_Pin,HIGH);
  _delay_us(TE);
  pinMode(IO_Pin, INPUT);
  if(digitalRead(IO_Pin) == 1) {
    //Serial.println("Slave Acknowledgement Received");
    _delay_us(TE);
  }

  /*          OR   
   *   while(digitalRead(IO_Pin) == 1);
   *   _delay_us(TE);
   */
}

/* 
 *  EEPROM Acknowledgement Master Sent LOW and Slave Sent HIGH Pulse
 */
inline void Ack_NOMAK_SAK() {
  pinMode(IO_Pin, OUTPUT);
  digitalWrite(IO_Pin,LOW);
  _delay_us(TE);
  pinMode(IO_Pin, INPUT);
  if(digitalRead(IO_Pin) == 1) {
    //Serial.println("Slave Acknowledgement Received");
    _delay_us(TE);
  }

  /*          OR   
   *   while(digitalRead(IO_Pin) == 1);
   *   _delay_us(TE);
   */
}

/* 
 *  EEPROM Data Transfer
 */
void eeprom_tx(char data) {
  pinMode(IO_Pin, OUTPUT);
  /*
   * It's sent MSB first
   * 
   * for (int i=7; i>=0;--i) {
    (data & (1 << i))?digitalWrite(IO_Pin,HIGH):digitalWrite(IO_Pin,LOW);
    _delay_us(TE);
  }*/

  /*  LSB First  */

  for (int i=0; i<8;i++) {
    (data & (1 << i))?digitalWrite(IO_Pin,HIGH):digitalWrite(IO_Pin,LOW);
    _delay_us(TE);
  }
}

/* 
 *  EEPROM Data Receive
 */
char eeprom_rx() {
  char data;
  pinMode(IO_Pin, INPUT);
  for(int i=0;i<8;i++) {
    if(digitalRead(IO_Pin) == HIGH)
      data = ((data<<1) | 1);
    else
      data = (data<<1);
    _delay_us(TE);
  }
  return data;
}

/* 
 *  EEPROM Header Address
 */
void header_address() {
  eeprom_tx(str_hdr);
  digitalWrite(IO_Pin,HIGH);
  _delay_us(TE);

  pinMode(IO_Pin, INPUT);
  _delay_us(TE);

  eeprom_tx(dev_addr);
  Ack_MAK_SAK();
}

/* 
 *  EEPROM First Start
 */
void first_start() {
  pinMode(IO_Pin, OUTPUT);
  digitalWrite(IO_Pin,HIGH);
  _delay_us(TSTBY);

  digitalWrite(IO_Pin,LOW);
  _delay_us(THDR);

  header_address();
}

/* 
 *  EEPROM Consecutive Start
 */
void consecutive_start() {
  Ack_NOMAK_SAK();
  pinMode(IO_Pin, OUTPUT);
  digitalWrite(IO_Pin,HIGH);
  _delay_us(TSS);
  digitalWrite(IO_Pin,LOW);
  _delay_us(THDR);

  header_address();
}

/* 
 *  EEPROM Write Enable
 */
void write_enable() {
  //consecutive_start();
  eeprom_tx(WREN);
  Ack_NOMAK_SAK();
}

/* 
 *  EEPROM Write Disable
 */
void write_disable() {
  //consecutive_start();
  eeprom_tx(WRDI);
  Ack_NOMAK_SAK();
}

/* 
 *  EEPROM Write Data to EEPROM Memory
 */
void write_data(uint8_t address, char *data) {
  uint8_t leng = strlen(data);
  //consecutive_start();
  eeprom_tx(WRITE);
  Ack_MAK_SAK();
  eeprom_tx(address);
  Ack_MAK_SAK();
  eeprom_tx(0x00);
  Ack_MAK_SAK();
  for(int i=0;*data != '\0';i++) {
    eeprom_tx(*data++);
    if((i+1) < leng)
      Ack_MAK_SAK();
  }
  Ack_NOMAK_SAK();
  _delay_ms(TWC_W);
}

/* 
 *  EEPROM Read Data to EEPROM Memory
 */
void read_data(uint8_t address, uint8_t leng, char *rec_buff) {
  //consecutive_start();
  eeprom_tx(READ);
  Ack_MAK_SAK();
  eeprom_tx(address);
  Ack_MAK_SAK();
  eeprom_tx(0x00);
  Ack_MAK_SAK();
  for(int i=0;i<leng;i++) {
    rec_buff[i] = eeprom_rx();
    if((i+1) < leng)
      Ack_MAK_SAK();
  }
  Ack_NOMAK_SAK();
}

/* 
 *  EEPROM Erase All
 */
void erase_all() {
  first_start();
  write_enable();
  eeprom_tx(ERAL);
  Ack_NOMAK_SAK();
  _delay_us(TWC_W);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  first_start();
  write_enable();
}

void loop() {
  // put your main code here, to run repeatedly:
  char data[6] = {"12345"}, rec_buff[10];
  uint8_t addr = 1;
  
  write_data(addr, data);
  delay(2000);
  consecutive_start();
  read_data(addr, 5, rec_buff);

  Serial.print("EEPROM Received Data : ");
  Serial.println(rec_buff);
  while(1) {}
}
