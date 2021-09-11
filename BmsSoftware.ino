

/*

  Battery Management System Software
  5.07.2021
  YUNUS EMRE UZMEZ

*/

#include <Wire.h>                   /*Library for I2C*/
#include "MCP3421.h"                /*Library and object for MCP3421 A/D Converter*/
MCP3421 MCP = MCP3421();
#include <EEPROM.h>


/******************************************************************************/

void Hucre_Gerilimleri();                         /*Variable definitons for analog digital converter*/
float ham[31];
 int kontrol = 0;
int z = 0;
int k = 0;
unsigned int ConvertValue;
unsigned char i, Chan;
unsigned char ConvertValueL, ConvertValueH;
const int VClock = 10;
const int VDataIn = 11;
const int VDataOut = 12;
const int V1EOC = 2;
const int V1ChipSelect = 3;
const int V2EOC = 9;
const int V2ChipSelect = 48;
byte  Channel = 0;
int pause = 0;

float vref_hucre_gerilimleri = 5.02; // Devre üzerindeki 5V çıkışından okunan değere eşit olmalıdır.

/******************************************************************************/

const int hucre1Balance = A1;   /*Direct balance pin definitions */
const int hucre2Balance = A0;
const int hucre3Balance = 4;
const int hucre4Balance = 5;
const int hucre21Balance = 47;
const int hucre22Balance = 46;

const int muxS0 = 17;          /*Multiplexer balance pin definitions*/
const int muxS1 = 16;
const int muxS2 = 6;
const int muxS3 = 7;
const int muxEnable = 8;

/******************************************************************************/

float current = 0.0;         /*Variable definitons for MCP3421*/
long l1;
float currentValue = 0.0;
char st1[10];


/******************************************************************************/

const int temp1 = A5;       /*Temperature sensor pin definitions*/
const int temp2 = A2;
const int temp3 = A4;
const int temp4 = A6;

float sicakliklar[4] = {0};
float maxSicaklik  = 0.0;

/******************************************************************************/

const int chargeProtectionPin = 45;
const int dischargeProtectionPin = 44;
const int buzzerPin = 24;

/******************************************************************************/

float haberlesme[34];
void verileri_derle();
void verileri_yazdir();
int p = 0;

/******************************************************************************/

void Hucre_Gerilimleri_Kalibrasyon();
void Hucre_Gerilimleri_Enbuyuk();
void Hucre_Gerilimleri_Enkucuk();
void hucre_gerilimlerini_yaz();
void hucre_gerilim_farkini_yaz();

/******************************************************************************/

void direkt_balans_et(unsigned int hucreNo);
void mux_balans_et(unsigned int hucreNo);
void batarya_balans_et();
bool float_karsilastir(float a , float b, int threshold);

/******************************************************************************/

void mcp3421_oku();
float sicaklik_oku(unsigned int sensorPin);
void sicaklik_yazdir();

/******************************************************************************/

void over_voltage_protection();
void under_voltage_protection();
void temperature_protection();

/*************************************************************************/

char tx_buff[153];
uint8_t checksum = 0;

void convert_2_int(int sayi, uint8_t *temp);
uint8_t calculate_checksum(uint8_t *array, uint16_t len);
void send_data_checksum();

/***********************************************************************/

void calculate_remaining_energy();

unsigned long consumedEnergyWh;
unsigned long wattHour ;
unsigned long remainingEnergyWh;
unsigned long energyPercentageWh ;

bool timeFlag = 0;

unsigned long firstTime = 0;
unsigned long secondTime = 0;
unsigned long timeDiff = 0;
float sec = 0;

double totalEnergy = 2300;

double energy = 2300;

int eeAdress = 0;

int timerCounter = 0;

/**********************************************************************/

void init_timer_interrupt();

/**********************************************************************/


void setup() {

  pinMode(buzzerPin, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(dischargeProtectionPin, OUTPUT);
  pinMode(chargeProtectionPin, OUTPUT);

  pinMode(VClock, OUTPUT);
  pinMode(VDataIn, OUTPUT);
  pinMode(VDataOut, INPUT);
  pinMode(V1EOC, INPUT);
  pinMode(V1ChipSelect, OUTPUT);
  pinMode(V2EOC, INPUT);
  pinMode(V2ChipSelect, OUTPUT);


  pinMode(hucre1Balance, OUTPUT);    /*Balance pins are set to OUTPUT*/
  pinMode(hucre2Balance, OUTPUT);
  pinMode(hucre3Balance, OUTPUT);
  pinMode(hucre4Balance, OUTPUT);
  pinMode(hucre21Balance, OUTPUT);
  pinMode(hucre22Balance, OUTPUT);

  pinMode(muxS0, OUTPUT);          /*Multiplexer pins are set to OUTPUT*/
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);
  pinMode(muxEnable, OUTPUT);

  digitalWrite(dischargeProtectionPin,HIGH);

  Wire.begin();

  MCP.init(0x68, 3, 0);

  Serial.begin(115200);

  Serial.println("Start");

  Serial3.begin(115200);

  digitalWrite(13, HIGH);

  init_timer_interrupt();

  int  temp = 0;

  EEPROM.get(eeAdress,temp);

  if(temp > 0  && temp <= 2300){
    totalEnergy = temp;
  }
  else{
    totalEnergy = totalEnergy;
  }

  Serial.println(temp);
  
}

void loop() {

  Hucre_Gerilimleri();
  Hucre_Gerilimleri_Kalibrasyon();
  Hucre_Gerilimleri_Enbuyuk();
  Hucre_Gerilimleri_Enkucuk();
  Hucre_Gerilimleri_Toplami();

  batarya_balans_et();
  //hucre_gerilimlerini_yaz();
  //hucre_gerilim_farkini_yaz();

  mcp3421_oku();

  sicaklik_yazdir();

  calculate_remaining_energy();

  over_voltage_protection();
  under_voltage_protection();
  temperature_protection();

  //verileri_yazdir();

  verileri_derle();
  //send_data_checksum();
  
}


/********************************************************************************/

void over_voltage_protection() {

  if (ham[22] > float(4.20))
    digitalWrite(chargeProtectionPin, LOW);
  else
    digitalWrite(chargeProtectionPin, HIGH);

}

/********************************************************************************/

void under_voltage_protection() {

  if (ham[23] < float(2.60))
    digitalWrite(dischargeProtectionPin, LOW);

  else
    digitalWrite(dischargeProtectionPin, HIGH);

}

/********************************************************************************/

void temperature_protection() {


  if (maxSicaklik > float(55.00))
    digitalWrite(buzzerPin, HIGH);
  else
    digitalWrite(buzzerPin, LOW);


  //------------------------------------------------------------------------------//

  if (maxSicaklik > float(70.00))
    digitalWrite(dischargeProtectionPin, LOW);

  else
    digitalWrite(dischargeProtectionPin, HIGH);


}

/********************************************************************************/

void direkt_balans_et(unsigned int hucreNo) {
  if (hucreNo == 1) {
    digitalWrite(hucre1Balance, HIGH);
  }
  if (hucreNo == 2) {
    digitalWrite(hucre2Balance, HIGH);
  }
  if (hucreNo == 3) {
    digitalWrite(hucre3Balance, HIGH);
  }
  if (hucreNo == 4) {
    digitalWrite(hucre4Balance, HIGH);
  }
  if (hucreNo == 21) {
    digitalWrite(hucre21Balance, HIGH);
  }
  if (hucreNo == 22) {
    digitalWrite(hucre22Balance, HIGH);
  }
}

/********************************************************************************/

void mux_balans_et(unsigned int hucreNo) {
  switch (hucreNo) {
    case 5:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, HIGH);
      break;
    case 6:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, HIGH);
      break;
    case 7:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, HIGH);
      break;
    case 8:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, HIGH);
      break;
    case 9:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, LOW);
      break;
    case 10:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, LOW);
      break;
    case 11:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, LOW);
      break;
    case 12:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, LOW);
      break;
    case 13:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, LOW);
      break;
    case 14:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, LOW);
      break;
    case 15:
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, LOW);
      break;
    case 16:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, LOW);
      digitalWrite(muxS3, LOW);
      break;
    case 17:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, HIGH);
      break;
    case 18:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, LOW);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, HIGH);
      break;
    case 19:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, LOW);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, HIGH);
      break;
    case 20:
      digitalWrite(muxEnable, HIGH);
      digitalWrite(muxS0, HIGH);
      digitalWrite(muxS1, HIGH);
      digitalWrite(muxS2, HIGH);
      digitalWrite(muxS3, HIGH);
    default:
      break;
  }

}

/******************************************************************************/

void batarya_balans_et() {
  if (float_karsilastir(ham[0], ham[23], 15)) {
    direkt_balans_et(1);
  }

  if (float_karsilastir(ham[1], ham[23], 15)) {
    direkt_balans_et(2);
  }

  if (float_karsilastir(ham[2], ham[23], 15)) {
    direkt_balans_et(3);
  }

  if (float_karsilastir(ham[3], ham[23], 15)) {
    direkt_balans_et(4);
  }

  if (float_karsilastir(ham[4], ham[23], 15)) {
    mux_balans_et(5);
  }

  if (float_karsilastir(ham[5], ham[23], 15)) {
    mux_balans_et(6);
  }

  if (float_karsilastir(ham[6], ham[23], 15)) {
    mux_balans_et(7);
  }

  if (float_karsilastir(ham[7], ham[23], 15)) {
    mux_balans_et(8);
  }

  if (float_karsilastir(ham[8], ham[23], 15)) {
    mux_balans_et(9);
  }


  if (float_karsilastir(ham[9], ham[23], 15)) {
    mux_balans_et(10);
  }

  if (float_karsilastir(ham[10], ham[23], 15)) {
    mux_balans_et(11);
  }

  if (float_karsilastir(ham[11], ham[23], 15)) {
    mux_balans_et(12);
  }

  if (float_karsilastir(ham[12], ham[23], 15)) {
    mux_balans_et(13);
  }

  if (float_karsilastir(ham[13], ham[23], 15)) {
    mux_balans_et(14);
  }

  if (float_karsilastir(ham[14], ham[23], 15)) {
    mux_balans_et(15);
  }

  if (float_karsilastir(ham[15], ham[23], 15)) {
    mux_balans_et(16);
  }


  if (float_karsilastir(ham[16], ham[23], 15)) {
    mux_balans_et(17);
  }

  if (float_karsilastir(ham[17], ham[23], 15)) {
    mux_balans_et(18);
  }

  if (float_karsilastir(ham[18], ham[23], 15)) {
    mux_balans_et(19);
  }

  if (float_karsilastir(ham[19], ham[23], 15)) {
    mux_balans_et(20);
  }

  if (float_karsilastir(ham[20], ham[23], 15)) {
    direkt_balans_et(21);
  }

  if (float_karsilastir(ham[21], ham[23], 15)) {
    direkt_balans_et(22);
  }


}


/************************************************************************************/

void Hucre_Gerilimleri()
{
  for ( z = 0; z < 22; z ++)
  {
    if (z < 11) {
      Channel = z;
      digitalWrite(V2ChipSelect , 1);
      pause = 0;
      ConvertValueL = 0;
      ConvertValueH = 0;
      if (digitalRead(V1EOC == 0))
      {
        digitalWrite(VClock , 0);
        digitalWrite(V1ChipSelect , 1);
        delayMicroseconds(2);
        digitalWrite(V1ChipSelect , 0);
        delayMicroseconds(2);
        Channel = Channel << 4;
        for ( i = 0; i < 4; i ++)
        {
          Chan = Channel;
          Chan = Chan >> 7;
          digitalWrite(VDataIn, Chan & 0x01);
          delayMicroseconds(2);
          digitalWrite(VClock , 1);
          digitalWrite(VClock , 0);
          Channel = Channel << 1;
        }
        for ( i = 0; i < 6; i ++)
        {
          digitalWrite(VClock , 1);
          digitalWrite(VClock , 0);
        }
        digitalWrite(V1ChipSelect , 1);

        while ((!digitalRead(V1EOC == 0)) && (pause < 10))
        {
          delayMicroseconds(10);
          pause = pause + 1;
        }
        if (pause == 10)
        {
          return (0xFFFF);
        }
        else
        {
          delayMicroseconds(10);
          digitalWrite(VClock , 0);
          digitalWrite(V1ChipSelect , 1);
          delayMicroseconds(1);
          digitalWrite(V1ChipSelect , 0);
          delayMicroseconds(1);
          for ( i = 0; i < 2; i ++)
          {
            digitalWrite(VClock , 1);
            ConvertValueH <<= 1;
            if (digitalRead(VDataOut))
              ConvertValueH |= 0x1;
            digitalWrite(VClock , 0);
            delayMicroseconds(1);
          }
          for ( i = 0; i < 8; i ++)
          {
            digitalWrite(VClock , 1);
            ConvertValueL <<= 1;
            if (digitalRead(VDataOut))
              ConvertValueL |= 0x1;
            digitalWrite(VClock , 0);
            delayMicroseconds(1);
          }
          digitalWrite(V1ChipSelect , 1);
          ConvertValue = ConvertValueH;
          ConvertValue <<= 8;
          ConvertValue |= ConvertValueL;
        }
      }
    }

    if ( (z < 23) && (z > 10) ) {
      Channel = (z - 11);
      digitalWrite(V1ChipSelect , 1);
      pause = 0;
      ConvertValueL = 0;
      ConvertValueH = 0;
      if (digitalRead(V2EOC == 0))
      {
        digitalWrite(VClock , 0);
        digitalWrite(V2ChipSelect , 1);
        delayMicroseconds(2);
        digitalWrite(V2ChipSelect , 0);
        delayMicroseconds(2);
        Channel = Channel << 4;
        for ( i = 0; i < 4; i ++)
        {
          Chan = Channel;
          Chan = Chan >> 7;
          digitalWrite(VDataIn, Chan & 0x01);
          delayMicroseconds(2);
          digitalWrite(VClock , 1);
          digitalWrite(VClock , 0);
          Channel = Channel << 1;
        }
        for ( i = 0; i < 6; i ++)
        {
          digitalWrite(VClock , 1);
          digitalWrite(VClock , 0);
        }
        digitalWrite(V2ChipSelect , 1);

        while ((!digitalRead(V2EOC == 0)) && (pause < 10))
        {
          delayMicroseconds(10);
          pause = pause + 1;
        }
        if (pause == 10)
        {
          return (0xFFFF);
        }
        else
        {
          delayMicroseconds(10);
          digitalWrite(VClock , 0);
          digitalWrite(V2ChipSelect , 1);
          delayMicroseconds(1);
          digitalWrite(V2ChipSelect , 0);
          delayMicroseconds(1);
          for ( i = 0; i < 2; i ++)
          {
            digitalWrite(VClock , 1);
            ConvertValueH <<= 1;
            if (digitalRead(VDataOut))
              ConvertValueH |= 0x1;
            digitalWrite(VClock , 0);
            delayMicroseconds(1);
          }
          for ( i = 0; i < 8; i ++)
          {
            digitalWrite(VClock , 1);
            ConvertValueL <<= 1;
            if (digitalRead(VDataOut))
              ConvertValueL |= 0x1;
            digitalWrite(VClock , 0);
            delayMicroseconds(1);
          }
          digitalWrite(V2ChipSelect , 1);
          ConvertValue = ConvertValueH;
          ConvertValue <<= 8;
          ConvertValue |= ConvertValueL;
        }
      }
    }

    //ham[z] = (ConvertValue * vref_hucre_gerilimleri) / 1024.0 ;

    ham[z] =  ConvertValue ;

  }
}



/******************************************************************************/

void Hucre_Gerilimleri_Kalibrasyon()
{ if (ham[0] < 250)
  {
    ham[0] = 0 ;
  }
  else {
    ham[0] =  ((ham[0] * vref_hucre_gerilimleri) / 1024.0) + 0.03;  //1. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[1] < 250)
  {
    ham[1] = 0 ;
  }
  else {
    ham[1] =  ((ham[1] * vref_hucre_gerilimleri) / 1024.0) + 0.07;  //2. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[2] < 250)
  {
    ham[2] = 0 ;
  }
  else {
    ham[2] =  ((ham[2] * vref_hucre_gerilimleri) / 1024.0) - 0.08;  //3. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[3] < 250)
  {
    ham[3] = 0 ;
  }
  else {
    ham[3] =  ((ham[3] * vref_hucre_gerilimleri) / 1024.0) - 0.08;  //4. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[4] < 250)
  {
    ham[4] = 0 ;
  }
  else {
    ham[4] =  ((ham[4] * vref_hucre_gerilimleri) / 1024.0) + 0.04;  //5. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[5] < 250)
  {
    ham[5] = 0 ;
  }
  else {
    ham[5] =  ((ham[5] * vref_hucre_gerilimleri) / 1024.0) - 0.02;  //6. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[6] < 250)
  {
    ham[6] = 0 ;
  }
  else {
    ham[6] =  ((ham[6] * vref_hucre_gerilimleri) / 1024.0) - 0.19;  //7. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[7] < 250)
  {
    ham[7] = 0 ;
  }
  else {
    ham[7] =  ((ham[7] * vref_hucre_gerilimleri) / 1024.0) + 0.06;  //8. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[8] < 250)
  {
    ham[8] = 0 ;
  }
  else {
    ham[8] =  ((ham[8] * vref_hucre_gerilimleri) / 1024.0) - 0.11;  //9. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[10] < 250)
  {
    ham[10] = 0 ;
  }
  else {
    ham[10] = ((ham[10] * vref_hucre_gerilimleri) / 1024.0) + 0.03; //10. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[9] < 250)
  {
    ham[9] = 0 ;
  }
  else {
    ham[9] =  ((ham[9] * vref_hucre_gerilimleri) / 1024.0) + 0.13;  //11. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[11] < 250)
  {
    ham[11] = 0 ;
  }
  else {
    ham[11] = ((ham[11] * vref_hucre_gerilimleri) / 1024.0) + 0.15; //12. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[12] < 250)
  {
    ham[12] = 0 ;
  }
  else {
    ham[12] = ((ham[12] * vref_hucre_gerilimleri) / 1024.0) - 0.04; //13. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[13] < 250)
  {
    ham[13] = 0 ;
  }
  else {
    ham[13] = ((ham[13] * vref_hucre_gerilimleri) / 1024.0) - 0.08; //14. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[14] < 250)
  {
    ham[14] = 0 ;
  }
  else {
    ham[14] = ((ham[14] * vref_hucre_gerilimleri) / 1024.0) - 0.22; //15. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[15] < 250)
  {
    ham[15] = 0 ;
  }
  else {
    ham[15] = ((ham[15] * vref_hucre_gerilimleri) / 1024.0) - 0.13; //16. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[16] < 250)
  {
    ham[16] = 0 ;
  }
  else {
    ham[16] = ((ham[16] * vref_hucre_gerilimleri) / 1024.0) - 0.33; //17. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[17] < 250)
  {
    ham[17] = 0 ;
  }
  else
  {
    ham[17] = ((ham[17] * vref_hucre_gerilimleri) / 1024.0) - 0.11; //18. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[18] < 250)
  {
    ham[18] = 0 ;
  }
  else {
    ham[18] = ((ham[18] * vref_hucre_gerilimleri) / 1024.0) - 0.34; //19. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[19] < 250)
  {
    ham[19] = 0 ;
  }
  else {
    ham[19] = ((ham[19] * vref_hucre_gerilimleri) / 1024.0) - 0.37; //20. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[21] < 250)
  {
    ham[21] = 0 ;
  }
  else {
    ham[21] = ((ham[21] * vref_hucre_gerilimleri) / 1024.0) + 0.14; //21. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
  if (ham[20] < 250)
  {
    ham[20] = 0 ;
  }
  else {
    ham[20] = ((ham[20] * vref_hucre_gerilimleri) / 1024.0) + 0.28; //22. Hücre gerilimi
  }//////////////////////////////////////////////////////////////////////////////////////
}

/******************************************************************************/

void Hucre_Gerilimleri_Enbuyuk() {
  ham[22] = ham[0];
  for ( i = 0; i < 22; i++) {
    if (ham[i] > ham[22]) {
      ham[22] = ham[i];
    }
  }
}

/******************************************************************************/

void Hucre_Gerilimleri_Enkucuk() {
  ham[23] = ham[0];
  for ( i = 0; i < 22; i++) {
    if (ham[i] < ham[23]) {
      if (ham[i] > 0.1) {
        ham[23] = ham[i];
      }
    }
  }
}

/******************************************************************************/

void Hucre_Gerilimleri_Toplami() {
  ham[24] = 0;
  for ( i = 0; i < 22; i++) {
    ham[24] += ham[i];
  }
}

/******************************************************************************/

void hucre_gerilimlerini_yaz() {

  Serial3.print("[");
  for (k = 0; k < 25 ; k++) {
    Serial3.print(ham[k]);
    if (k < 24) {
      Serial3.print("  ");
    }
    delay (1);
  }
  Serial3.println("]");


}

/******************************************************************************/

void hucre_gerilim_farkini_yaz() {
  int c = 0;

  Serial.print("[");
  for (c = 0 ; c < 22 ; c++ ) {
    Serial.print(ham[c] - ham[23]);
    if (c < 21) {
      Serial.print("  ");
    }
  }
  Serial.println("]");

}

/******************************************************************************/

bool float_karsilastir(float a , float b, int threshold) {
  int temp = 0;

  temp = (a - b) * 100;

  if (temp > threshold)
    return 1;
  else
    return 0;
}

/******************************************************************************/

void mcp3421_oku() {


  while (MCP.ready() == 0);

  l1 = MCP.getLong();

  /*Serial.print("  ");
    sprintf(st1, "ld %ld", l1);
    Serial.print(st1);
    Serial.print(" ");
    Serial.print("Milivolt(Reference) : ");*/

  currentValue = MCP.getDouble() * 1000;

  current = (currentValue - 1004.00 ) * 20 * 0.001;

  if (timeFlag == 0) {
    firstTime = micros();
    timeFlag = 1;
  }
  else {
    secondTime = micros();
    timeFlag = 0;
    timeDiff = secondTime - firstTime;
    sec = timeDiff * 0.000001;
  }

  /*
    firstTime = micros();
    timeFlag = 1;
    Serial.print(firstTime);
    Serial.print("     ");
    Serial.print(secondTime);
    Serial.print("     ");
    Serial.println(sec);*/


    /*
    Serial3.print("l1 : ");
    Serial3.print(l1);
    Serial3.print(" currentValue : ");
    Serial3.print(currentValue);
    Serial3.print("  Current(Amper) : ");
    Serial3.println(current);*/


}

/***********************************************************************************/

void calculate_remaining_energy() {

  double batteryVoltage = ham[24];
  
  consumedEnergyWh = consumedEnergyWh + (batteryVoltage * current * sec);
  wattHour = consumedEnergyWh / 3600;
  remainingEnergyWh = totalEnergy - wattHour;
  energyPercentageWh = (remainingEnergyWh / energy) * 100.0;


  /*
    Serial.print(sec);
    Serial.print(" ");
    Serial.print(startingEnergy);
    Serial.print(" ");
    Serial.print(consumedEnergyWh);
    Serial.print(" ");
    Serial.print(wattHour);
    Serial.print(" ");
    Serial.print(remainingEnergyWh);
    Serial.print(" ");
    Serial.println(energyPercentageWh);
  */

}

/***********************************************************************************/

float sicaklik_oku(unsigned int sensorPin) {
  float sicaklik = 0.0;
  float average[20] = {0};
  float sum = 0.0;
  float cValue = 0.0;

  for (int m = 0 ; m < 20 ; m++) {
    average[m] = analogRead(sensorPin);
    delayMicroseconds(10);
  }

  sum = 0.0;

  for (int n = 0; n < 20 ; n++) {
    sum += average[n];
  }

  cValue = sum / 20;

  sicaklik = ((float)cValue / 1023) * 5000;
  sicaklik = sicaklik / 10.0;
  return sicaklik;
}

/******************************************************************************/

void sicaklik_yazdir() {
  sicakliklar[0] = sicaklik_oku(temp1);
  sicakliklar[1] = sicaklik_oku(temp2);
  sicakliklar[2] = sicaklik_oku(temp3);
  sicakliklar[3] = sicaklik_oku(temp4);

  maxSicaklik = sicakliklar[0];
  for (int f = 0; f < 4 ; f++) {
    if (sicakliklar[f] > maxSicaklik) {
      maxSicaklik = sicakliklar[f];
    }
  }

  /*
    Serial3.print('[');
      Serial3.print(sicakliklar[0]);
      Serial3.print("  ");
      Serial3.print(sicakliklar[1]);
      Serial3.print("  ");
      Serial3.print(sicakliklar[2]);
      Serial3.print("  ");
      Serial3.print(sicakliklar[3]);
      Serial3.print("  ");
      Serial3.print(maxSicaklik);
      Serial3.print(']');*/


}

/******************************************************************************/


void verileri_derle() {

  for (int i = 0; i < 16 ; i++) {
    haberlesme[i] = ham[i];
  }

  haberlesme[16] = ham[16];
  haberlesme[17] = ham[17];
  haberlesme[18] = ham[18];
  haberlesme[19] = ham[19];
  haberlesme[20] = ham[20];
  haberlesme[21] = ham[21];

  haberlesme[22] = ham[22];
  haberlesme[23] = ham[23];
  haberlesme[24] = ham[24];


  haberlesme[25] = sicakliklar[0];
  haberlesme[26] = sicakliklar[1];
  haberlesme[27] = sicakliklar[2];
  haberlesme[28] = sicakliklar[3];
  haberlesme[29] = maxSicaklik ;

  haberlesme[30] = current;
  haberlesme[31] = (float)wattHour;
  haberlesme[32] = (float)remainingEnergyWh;
  haberlesme[33] = (float)energyPercentageWh;

}

/******************************************************************************/

void verileri_yazdir() {
  for (int i = 0; i < 16 ; i++) {
    haberlesme[i] = ham[i];
  }

  haberlesme[16] = ham[16];
  haberlesme[17] = ham[17];
  haberlesme[18] = ham[18];
  haberlesme[19] = ham[19];
  haberlesme[20] = ham[20];
  haberlesme[21] = ham[21];

  haberlesme[22] = ham[22];
  haberlesme[23] = ham[23];
  haberlesme[24] = ham[24];


  haberlesme[25] = sicakliklar[0];
  haberlesme[26] = sicakliklar[1];
  haberlesme[27] = sicakliklar[2];
  haberlesme[28] = sicakliklar[3];
  haberlesme[29] = maxSicaklik ;

  haberlesme[30] = current;
  haberlesme[31] = (float)wattHour;
  haberlesme[32] = (float)remainingEnergyWh;
  haberlesme[33] = (float)energyPercentageWh;


  Serial.print("<");

  for (p = 0; p < 34 ; p++) {
    Serial.print(haberlesme[p]);
    if (p < 33) {
      Serial.print("|");
    }

  }

  Serial.println(">");

}


/******************************************************************************/

void convert_2_int(int sayi, uint8_t *temp)
{
  int sonuc = 0;
  int bolum = 0, kalan = 0;

  if (sayi < 0)
  {
    sayi = -sayi;
  }
  else if (sayi > 0) {
    sayi = sayi;
  }

  if (sayi < 10000 && sayi >= 1000)
  {
    bolum = sayi / 1000;
    kalan = sayi % 1000;

    *temp++ = bolum + '0';

    bolum = kalan / 100;
    kalan = kalan % 100 ;

    *temp++ = bolum + '0';

    bolum = kalan / 10;
    kalan = kalan % 10;

    *temp++ = bolum + '0';
    *temp++ = kalan + '0';
  }

  if (sayi < 1000 && sayi >= 100)
  {
    *temp++ = '0';

    bolum = sayi / 100;
    kalan = sayi % 100 ;

    *temp++ = bolum + '0';

    bolum = kalan / 10;
    kalan = kalan % 10;

    *temp++ = bolum + '0';
    *temp++ = kalan + '0';
  }

  if (sayi < 100 && sayi >= 10)
  {
    *temp++ = '0';
    *temp++ = '0';

    bolum = sayi / 10;
    kalan = sayi % 10;

    *temp++ = bolum + '0';
    *temp++ = kalan + '0';
  }
  
  if (sayi < 10)
  {
    *temp++ =  '0';
    *temp++ =  '0';
    *temp++ =  '0';
    *temp++ = sayi + '0';
  }
}


/******************************************************************************/

uint8_t calculate_checksum(uint8_t *array, uint16_t len) {
  uint8_t rem = 0x41;
  uint16_t  i = 1, j = 0;

  for (i = 1; i < len; i++) {

    rem = rem ^ array[i];

    for (j = 0; j < 8; j++) {

      if (rem & 0x80) {  // if leftmost (most significant) bit is set
        rem = (rem << 1) ^ 0x07;
      }
      else {
        rem = rem << 1;
      }

    }

  }

  return rem;
}

/******************************************************************************/


void send_data_checksum() {
  uint8_t temp_buff[4]; //Checksum için gerekli değişkenler

  tx_buff[0] = '<';
  tx_buff[1] = '[';

  convert_2_int(int(haberlesme[0] * 1000), temp_buff);
  tx_buff[2] = temp_buff[0]; //basamak basamak ayrilan sayinin 1000'ler basamagi
  tx_buff[3] = temp_buff[1]; //basamak basamak ayrilan sayinin 100'ler basamagi
  tx_buff[4] = temp_buff[2]; //basamak basamak ayrilan sayinin 10'ler basamagi              //hucre 0
  tx_buff[5] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[1] * 1000), temp_buff);
  tx_buff[6] = temp_buff[0];
  tx_buff[7] = temp_buff[1];                                                      //hucre 1
  tx_buff[8] = temp_buff[2];
  tx_buff[9] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[2] * 1000), temp_buff);
  tx_buff[10] = temp_buff[0];
  tx_buff[11] = temp_buff[1];                                         //hucre 2
  tx_buff[12] = temp_buff[2];
  tx_buff[13] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[3] * 1000), temp_buff);
  tx_buff[14] = temp_buff[0];
  tx_buff[15] = temp_buff[1];                                         //hucre 3
  tx_buff[16] = temp_buff[2];
  tx_buff[17] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[4] * 1000), temp_buff);
  tx_buff[18] = temp_buff[0];
  tx_buff[19] = temp_buff[1];
  tx_buff[20] = temp_buff[2];                                             //hucre 4
  tx_buff[21] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[5] * 1000), temp_buff);
  tx_buff[22] = temp_buff[0];
  tx_buff[23] = temp_buff[1];                                         //hucre 5
  tx_buff[24] = temp_buff[2];
  tx_buff[25] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[6] * 1000), temp_buff);
  tx_buff[26] = temp_buff[0];
  tx_buff[27] = temp_buff[1];
  tx_buff[28] = temp_buff[2];                                               //hucre 6
  tx_buff[29] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[7] * 1000), temp_buff);
  tx_buff[30] = temp_buff[0];
  tx_buff[31] = temp_buff[1];                                     //hucre 7
  tx_buff[32] = temp_buff[2];
  tx_buff[33] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[8] * 1000), temp_buff);
  tx_buff[34] = temp_buff[0];
  tx_buff[35] = temp_buff[1];
  tx_buff[36] = temp_buff[2];                                   //hucre 8
  tx_buff[37] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[9] * 1000), temp_buff);
  tx_buff[38] = temp_buff[0];
  tx_buff[39] = temp_buff[1];                                     //hucre 9
  tx_buff[40] = temp_buff[2];
  tx_buff[41] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[10] * 1000), temp_buff);
  tx_buff[42] = temp_buff[0];
  tx_buff[43] = temp_buff[1];
  tx_buff[44] = temp_buff[2];                                     //hucre 10
  tx_buff[45] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[11] * 1000), temp_buff);
  tx_buff[46] = temp_buff[0];
  tx_buff[47] = temp_buff[1];
  tx_buff[48] = temp_buff[2];                                   //hucre 11
  tx_buff[49] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[12] * 1000), temp_buff);
  tx_buff[50] = temp_buff[0];
  tx_buff[51] = temp_buff[1];
  tx_buff[52] = temp_buff[2];                                       //hucre 12
  tx_buff[53] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[13] * 1000), temp_buff);
  tx_buff[54] = temp_buff[0];
  tx_buff[55] = temp_buff[1];
  tx_buff[56] = temp_buff[2];                                     //hucre 13
  tx_buff[57] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[14] * 1000), temp_buff);
  tx_buff[58] = temp_buff[0];
  tx_buff[59] = temp_buff[1];
  tx_buff[60] = temp_buff[2];                                     //hucre 14
  tx_buff[61] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[15] * 1000), temp_buff);
  tx_buff[62] = temp_buff[0];
  tx_buff[63] = temp_buff[1];
  tx_buff[64] = temp_buff[2];                                   //hucre 15
  tx_buff[65] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[16] * 1000), temp_buff);
  tx_buff[66] = temp_buff[0];
  tx_buff[67] = temp_buff[1];
  tx_buff[68] = temp_buff[2];                                     //hucre 16
  tx_buff[69] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[17] * 1000), temp_buff);
  tx_buff[70] = temp_buff[0];
  tx_buff[71] = temp_buff[1];
  tx_buff[72] = temp_buff[2];                                         //hucre 17
  tx_buff[73] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[18] * 1000), temp_buff);
  tx_buff[74] = temp_buff[0];
  tx_buff[75] = temp_buff[1];                                     //hucre 18
  tx_buff[76] = temp_buff[2];
  tx_buff[77] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[19] * 1000), temp_buff);
  tx_buff[78] = temp_buff[0];
  tx_buff[79] = temp_buff[1];                                   //hucre 19
  tx_buff[80] = temp_buff[2];
  tx_buff[81] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[20] * 1000), temp_buff);
  tx_buff[82] = temp_buff[0];
  tx_buff[83] = temp_buff[1];                                //hucre 20
  tx_buff[84] = temp_buff[2];
  tx_buff[85] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[21] * 1000), temp_buff);
  tx_buff[86] = temp_buff[0];
  tx_buff[87] = temp_buff[1];                                //hucre 21
  tx_buff[88] = temp_buff[2];
  tx_buff[89] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[22] * 1000), temp_buff);
  tx_buff[90] = temp_buff[0];
  tx_buff[91] = temp_buff[1];                                  //min Hücre
  tx_buff[92] = temp_buff[2];
  tx_buff[93] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[23] * 1000), temp_buff);
  tx_buff[94] = temp_buff[0];
  tx_buff[95] = temp_buff[1];                                  //max Hücre
  tx_buff[96] = temp_buff[2];
  tx_buff[97] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[24] * 100), temp_buff);
  tx_buff[98] = temp_buff[0];
  tx_buff[99] = temp_buff[1];
  tx_buff[100] = temp_buff[2];                                  //toplamHucre
  tx_buff[101] = temp_buff[3];
  tx_buff[102] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[25] * 100), temp_buff);
  tx_buff[103] = temp_buff[0];
  tx_buff[104] = temp_buff[1];                                  //sicaklik 1
  tx_buff[105] = temp_buff[2];
  tx_buff[106] = temp_buff[3];
  tx_buff[107] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[26] * 100), temp_buff);
  tx_buff[108] = temp_buff[0];
  tx_buff[109] = temp_buff[1];
  tx_buff[110] = temp_buff[2];                                      //sicaklik 2
  tx_buff[111] = temp_buff[3];
  tx_buff[112] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[27] * 100), temp_buff);
  tx_buff[113] = temp_buff[0];
  tx_buff[114] = temp_buff[1];
  tx_buff[115] = temp_buff[2];
  tx_buff[116] = temp_buff[3];                                        //sicaklik 3
  tx_buff[117] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[28] * 100), temp_buff);
  tx_buff[118] = temp_buff[0];
  tx_buff[119] = temp_buff[1];
  tx_buff[120] = temp_buff[2];                                          //sicaklik 4
  tx_buff[121] = temp_buff[3];
  tx_buff[122] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[29] * 100), temp_buff);
  tx_buff[123] = temp_buff[0];
  tx_buff[124] = temp_buff[1];                                        //max Sicaklik
  tx_buff[125] = temp_buff[2];
  tx_buff[126] = temp_buff[3];
  tx_buff[127] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[30] * 100), temp_buff);
  tx_buff[128] = temp_buff[0];
  tx_buff[129] = temp_buff[1];                                      //akim
  tx_buff[130] = temp_buff[2];
  tx_buff[131] = temp_buff[3];

  tx_buff[132] = '|'; //ayirma biti

  convert_2_int(int(haberlesme[31]), temp_buff);
  tx_buff[133] = temp_buff[0];                                  //harcanan enerji wh
  tx_buff[134] = temp_buff[1];
  tx_buff[135] = temp_buff[2];
  tx_buff[136] = temp_buff[3];

  tx_buff[137] = '|';

  convert_2_int(int(haberlesme[32]), temp_buff);
  tx_buff[138] = temp_buff[0];                                  //kalan enerji wh
  tx_buff[139] = temp_buff[1];
  tx_buff[140] = temp_buff[2];
  tx_buff[141] = temp_buff[3];

  tx_buff[142] = '|';

  convert_2_int(int(haberlesme[33]) , temp_buff);     //kalan enerji yüzdesi
  tx_buff[143] = temp_buff[1];
  tx_buff[144] = temp_buff[2];
  tx_buff[145] = temp_buff[3];

  tx_buff[146] = ']';

  uint8_t chcksm_array[4];
  checksum = calculate_checksum(tx_buff, 147);
  convert_2_int(int(checksum), chcksm_array);

  tx_buff[147] = chcksm_array[1];
  tx_buff[148] = chcksm_array[2];
  tx_buff[149] = chcksm_array[3];

  tx_buff[150] = '*'; //checksum bitiş biti
  tx_buff[151] = '>'; //bitis biti

  
  kontrol = 1;
  
  for (int i = 0; i < 152 ; i++)
  {
    if(tx_buff[i] == '0' || tx_buff[i] == '1' || tx_buff[i] == '2' || tx_buff[i] == '3' || tx_buff[i] == '4' || tx_buff[i] == '5' || tx_buff[i] == '6' || tx_buff[i] == '7' || tx_buff[i] == '8' || tx_buff[i] == '9' || tx_buff[i] == '<' ||tx_buff[i] == '>' || tx_buff[i] == '*' ||tx_buff[i] == '|' ||tx_buff[i] == '[' ||tx_buff[i] == ']'  )
    {
        kontrol = 1;    
    }
    else
    {
        kontrol = 0;
    }
      
  }

    for (int i = 0; i < 152 ; i++)
  {
    if(kontrol == 1)
    {
       Serial3.print(tx_buff[i]);
    }
      
  }
  Serial3.println();
}

/*************************************************************************************/

void init_timer_interrupt() {

  //set timer4 interrupt at 1Hz
  TCCR4A = 0;// set entire TCCR1A register to 0
  TCCR4B = 0;// same for TCCR1B
  TCNT4  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR4A = 15624 / 1; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  sei();//allow interrupts

}

/***************************************************************************************/

ISR(TIMER4_COMPA_vect) {
  //verileri_derle();
  send_data_checksum();

  timerCounter++;

  if(timerCounter > 30){
    EEPROM.put(eeAdress, remainingEnergyWh);
    timerCounter = 0;
  }
  
  //verileri_yazdir();
}


/***************************************************************************************/
