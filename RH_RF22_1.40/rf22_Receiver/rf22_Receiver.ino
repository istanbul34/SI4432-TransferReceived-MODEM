#define MODE_Receiver
//#define MODE_Transmitter

/*1.4_Write_by_ImaSoft_03.07.16
 * Воскресенье - v1.0 от 22.11.15, Начало_проекта_написания_фунции_передачи_данных_с_клиенской_части_на_сервер_для_модема_SI4432
 * Понедельник - v1.1 от 23.11.15, Обновил_модуль_библиотеки_на_RF22_SI4432-1.40_изменил_формулу_вывода_данных_уровня_сигнала_RSSI
 * Пятница     - v1.2 от 27.11.15, Ввел_переменную_максимального_значения_мощьности_передатчика
 * Субота      - v1.3 от 02.07.16, По просьбе пользователя с Youtube "da da", добавил в приемник и передатчик индификатор, устройства, так-же добавил в буфер передачи, передачу одного значения LONG
 * Воскресенье - v1.4 от 03.07.16, Перевод кода, в библиотеку!
 * -----------------------------------------------------------------------------------------------------------

 
/* Таблица_переменных_выходной_мощьности_передатчика
 * RF22_TXPOW_1DBM
 * RF22_TXPOW_2DBM
 * RF22_TXPOW_5DBM
 * RF22_TXPOW_8DBM
 * RF22_TXPOW_11DBM
 * RF22_TXPOW_14DBM
 * RF22_TXPOW_17DBM
 * RF22_TXPOW_20DBM
 * RF22_TXPOW_MAXDBM
*/

/* Таблица_видов_додуляции_и_скорости_передачи_данных
 * FSK_Rb2Fd5                                                     //< FSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
 * FSK_Rb2_4Fd36                                                  //< FSK, No Manchester, Rb = 2.4kbs,  Fd = 36kHz
 * FSK_Rb4_8Fd45                                                  //< FSK, No Manchester, Rb = 4.8kbs,  Fd = 45kHz
 * FSK_Rb9_6Fd45                                                  //< FSK, No Manchester, Rb = 9.6kbs,  Fd = 45kHz
 * FSK_Rb19_2Fd9_6                                                //< FSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
 * FSK_Rb38_4Fd19_6                                               //< FSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
 * FSK_Rb57_6Fd28_8                                               //< FSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
 * FSK_Rb125Fd125                                                 //< FSK, No Manchester, Rb = 125kbs,  Fd = 125kHz
 * FSK_Rb_512Fd2_5                                                //< FSK, No Manchester, Rb = 512bs,  Fd = 2.5kHz, for POCSAG compatibility
 * FSK_Rb_512Fd4_5                                                //< FSK, No Manchester, Rb = 512bs,  Fd = 4.5kHz, for POCSAG compatibility
 *
 * GFSK_Rb2Fd5                                                    //< GFSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
 * GFSK_Rb2_4Fd36                                                 //< GFSK, No Manchester, Rb = 2.4kbs,  Fd = 36kHz
 * GFSK_Rb4_8Fd45                                                 //< GFSK, No Manchester, Rb = 4.8kbs,  Fd = 45kHz
 * GFSK_Rb9_6Fd45                                                 //< GFSK, No Manchester, Rb = 9.6kbs,  Fd = 45kHz
 * GFSK_Rb19_2Fd9_6                                               //< GFSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
 * GFSK_Rb38_4Fd19_6                                              //< GFSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
 * GFSK_Rb57_6Fd28_8                                              //< GFSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
 * GFSK_Rb125Fd125                                                //< GFSK, No Manchester, Rb = 125kbs,  Fd = 125kHz
 *
 * OOK_Rb2_4Bw335                                                 //< OOK, No Manchester, Rb = 2.4kbs,  Rx Bandwidth = 335kHz
 * OOK_Rb4_8Bw335                                                 //< OOK, No Manchester, Rb = 4.8kbs,  Rx Bandwidth = 335kHz
 * OOK_Rb9_6Bw335                                                 //< OOK, No Manchester, Rb = 9.6kbs,  Rx Bandwidth = 335kHz
 * OOK_Rb19_2Bw335                                                //< OOK, No Manchester, Rb = 19.2kbs, Rx Bandwidth = 335kHz
 * OOK_Rb38_4Bw335                                                //< OOK, No Manchester, Rb = 38.4kbs, Rx Bandwidth = 335kHz
 * OOK_Rb40Bw335                                                  //< OOK, No Manchester, Rb = 40kbs,   Rx Bandwidth = 335kHz
*/


//-------------------------------------------------------------- //
//#include <SPI.h>                                               //
#include <RF22.h>                                                // скачанная библиотека, RadioHead https://yadi.sk/d/Impr0nuNWY2Zq
RF22 RF22_Si4432;                                                //
//-------------------------------------------------------------- //

//---Дерективы_отладки--После_отдадки_все_за_комментировать----- //
#define Debug_Serial                                             //Разрешить_использование_для_отладки_последавательный_порт
#define Debug_RF-DATA                                            //Вывод_в_сериал_данных_информации_зачении_потециометра
//-------------------------------------------------------------- //
#define RF22_TXPOW_MAXDBM     15                                 //Значение_максимальной_мощьности_прердатчика_для_SI4432
#define ID_SIGNATUR           0xAA                               //Сигнатура_устройства
#define RF_TIME_OUT           500                                //Время_ожидания_приема_данных_в_милисекундах
byte PACAGE_LOOP_TX         = 0x03;                              //Количество_повторений_пакета_TX_в_случае_неудачной_передачи_данных
float OperatingFreq         = 434.17;                            //Рабочая_частота_передатчика_в_мГц_на_LPD-45канал
boolean FLERRInitRHRF22     = true;                              //Флаг_ошибки_инициализации_драйвера_RH_RF22_модуля_SI4432
RF22::ModemConfigChoice Modulations = RF22_Si4432.OOK_Rb2_4Bw335;// 
//-------------------------------------------------------------- //

/*
 * Структура_буфера_приемника/передатчика
 * | BYTE(Размер_масива_CHAR)| BYTE(ID_SIGNATUR | LONG(RFData) | Масив(CHAR)|
 */
 

//***START_DataTransferRF22************************************* //
int  RF22LastRssi     = 0;                                       //Уровень_принемаемого_сигнала
//-------------------------------------------------------------- //
boolean DataTransferRF22(uint8_t* buf, uint8_t Sizebuf, float OperatiFreq, uint8_t PowerTransmitter, byte PACAGELOOPTX, word RFTIMEOUT) {
  //------------------------------------------------------------ //
  RF22_Si4432.setFrequency(OperatiFreq, 0.1);                    //
  RF22_Si4432.setTxPower(PowerTransmitter);                      //
  //------------------------------------------------------------ //
  for(byte i =0; i<PACAGELOOPTX; i++) {                          //
    //---------------------------------------------------------- //
    if (!RF22_Si4432.send(buf, Sizebuf)) return false;           //
    RF22_Si4432.waitPacketSent();                                //
    if (RF22_Si4432.waitAvailableTimeout(RFTIMEOUT)) {           // Now wait for a reply
      uint8_t Data[RF22_MAX_MESSAGE_LEN];                        //
      uint8_t len = sizeof(Data);                                //
      // Should be a reply message for us now                    //
      if (RF22_Si4432.recv(Data, &len))                          //
        {                                                        //
         for(byte ii =1; ii<Data[0]; ii++) {                     //
          if (buf[ii] != Data[ii]) {return false;}               //
          //Serial.print("Length_Data_Buf:");Serial.println(len,DEC );
          //Serial.print("buf[");Serial.print(ii);Serial.print("]:");Serial.println(buf[ii],DEC );
          //Serial.print("Data[");Serial.print(ii);Serial.print("]:");Serial.println(Data[ii],DEC );
          //Serial.println("");                                  //
         }                                                       //
         RF22LastRssi = (int8_t)(-120 + ((RF22_Si4432.lastRssi()/2)));
	       RF22_Si4432.setModeTx();                                //Уменьшаем_потребление(в_режиме_приема~20мА)_модуля_переводом_его_в_режим_передатчика_для_уменьшения_потребления_до~2мА
         return true;                                            //
        }                                                        //
      else return false;                                         //
    }                                                            //
    else {                                                       //
      //Serial.print("waitAvailableTimeout: ERROR, RFTIMEOUT:");Serial.println(RFTIMEOUT,DEC);
    }                                                            //
  }                                                              //
  return false;                                                  //
}                                                                //
//***END_DataTransferRF22*************************************** //

//***START_DataSendSI4432*************************************** //
byte DataSendSI4432(uint8_t* buf, uint8_t Sizebuf, float OperatiFreq, uint8_t PowerTransmitter, byte PACAGELOOPTX, word RFTIMEOUT) {
  if (!FLERRInitRHRF22) {                                        //
    if (!DataTransferRF22(buf, Sizebuf, OperatingFreq, PowerTransmitter, PACAGE_LOOP_TX, RFTIMEOUT)) {
      #ifdef Debug_RF-DATA                                       //
      Serial.println("No reply, is rf22_server: Data Send FAILED!");
      Serial.println("");                                        //
      #endif                                                     //
      return 2;                                                  //
    }                                                            //
   else {                                                        //
      #ifdef Debug_RF-DATA                                       //
      Serial.println("Sending to rf22_server");                  // Send a message to rf22_server
      char Str[buf[0]];for(byte i =0; i<=(buf[0]-1); i++) Str[i] = buf[i+1+1+4];
      byte x[4];for(byte i = 0; i < 4; i++) x[i] = buf[i+2];long *RFData = (long *)&x;
      Serial.print("Data_Send_to_Server: ");Serial.print(Str);Serial.print("; String Len:");Serial.print(sizeof(Str));Serial.print("; Data:");Serial.print((RFData[0]/1000), DEC);Serial.println("S");
      Serial.print("RSSIRead:");Serial.println(RF22LastRssi,DEC);//
      Serial.println("");                                        //
      #endif                                                     //
      return 1;                                                  //
   }                                                             //
  }                                                              //
  else {return 3;}                                               //
}                                                                //
//---END_DataSendSI4432----------------------------------------- //

//***START_DataReadSI4432*************************************** //
byte DataReadSI4432(uint8_t* buf, uint8_t len, byte ID, byte RFTIMEOUT) {
  if (!FLERRInitRHRF22) {                                        //
    if (RF22_Si4432.waitAvailableTimeout(RF_TIME_OUT)) {         //
      delay(10);                                                 //
      RF22_Si4432.recv(buf, &len);                               //
        if (buf[1] == ID) {                                      //
         //---Send_a_reply-------------------------------------- //
         RF22_Si4432.send(buf, (buf[0]+2));                      //
         RF22_Si4432.waitPacketSent();                           //
         //----------------------------------------------------- //
         //digitalWrite(LedPin,LOW);                             //
         return 1;                                               //
        }                                                        //
        else {                                                   //
          #ifdef Debug_RF-DATA                                   //
          Serial.println("ID-failed");                           //
          Serial.println("");                                    //
          #endif                                                 //
          return 2;                                              //
        }                                                        //
        //------------------------------------------------------ //
    }
   return 0;
  }
  else {return 3;}
}
//---END_DataReadSI4432-----------------------------------------




//***START_RF22Si4432init*************************************** //
void Si4432_init(float OperatingFreq, RF22::ModemConfigChoice TableOFmodulations, byte TXPOW) {
  if (RF22_Si4432.init()) {                                      //
    FLERRInitRHRF22 = false;                                     //Флаг_ошибки_инициализации_модуля_SI4432
    RF22_Si4432.setFrequency(OperatingFreq, 0.1);                //
    RF22_Si4432.setModemConfig(TableOFmodulations);              //
    RF22_Si4432.setTxPower(RF22_TXPOW);                          //
    #ifdef Debug_RF-DATA                                         //
    Serial.print("RH_RF22:");Serial.println(" Init-OK");         //
    Serial.println("");                                          //
    #endif                                                       //
  }                                                              //
  else {                                                         //
    FLERRInitRHRF22 = true;                                      //Флаг_ошибки_инициализации_модуля_SI4432
    #ifdef Debug_RF-DATA                                         //
    Serial.print("RH_RF22:");Serial.println(" Init-Failed");     //
    Serial.println("");                                          //
    #endif                                                       //
  }                                                              //
}                                                                //
//---END_DataReadSI4432----------------------------------------- //




//***START_Setup************************************************ //
void setup() {                                                   //
  //------------------------------------------------------------ // 
  #ifdef Debug_Serial                                            //
  Serial.begin(9600);                                            //
  #endif                                                         //
  //------------------------------------------------------------ //
}                                                                //
//***END_Setup************************************************** //








#ifdef MODE_Transmitter
//***START_loop************************************************* //
void loop() {                                                    //
  //------------------------------------------------------------ //
  char Str[] = "Hello_World";                                    //
  long RFData = millis();                                        //
  //------------------------------------------------------------ //
  byte SizeStr = sizeof(Str);                                    //
  uint8_t buf[SizeStr+1+1+4];buf[0] = SizeStr;buf[1] = ID_SIGNATUR;
  byte *x = (byte *)&RFData;for(byte i = 0; i < 4; i++) buf[i+2]=x[i];
  for(byte i =0; i<=(SizeStr-1); i++) buf[i+6] = Str[i];         //
  //Serial.print("Data_Send_to_Server:");Serial.print((char*)buf);Serial.print(" length:");Serial.print(sizeof(buf));Serial.print(" length_buf[0]:");Serial.println(buf[0],DEC);
  //------------------------------------------------------------ // 
  /*Коды_ошибок_модуля_передатчика(DataSendSI4432)               //
   * 1 - Успешная_передача_данных                                //
   * 2 - Нет_ответа_от_приемника                                 //
   * 3 - Ошибка_инициализации_модуля_SI4432                      //
   */                                                            //
  byte FLError = DataSendSI4432(buf, sizeof(buf), OperatingFreq, RF22_TXPOW_MAXDBM, PACAGE_LOOP_TX, RF_TIME_OUT);
  if (FLError == 3) {delay(3000);RF22Si4432_init(OperatingFreq, Modulations, RF22_TXPOW_MAXDBM);}
  //------------------------------------------------------------ //
  
  //------------------------------------------------------------ //
  delay(random(500, 3000));                                      //
  //------------------------------------------------------------ //
}                                                                //
//***END_loop*************************************************** //
#endif                                                           //








#ifdef MODE_Receiver
//***START_loop************************************************* //
uint8_t buf[RF22_MAX_MESSAGE_LEN];                               //
uint8_t len = sizeof(buf);                                       //
//-------------------------------------------------------------- //
void loop() {                                                    //
  //-------------------------------------------------------------//
  /*Коды_ошибок_модуля_приемник(DataReadSI4432)                  //
   * 1 - Успешный_прием_данных                                   //
   * 2 - Не_савподает_ID_сигнатура_устройства                    //
   * 3 - Ошибка_инициализации_модуля_SI4432                      //
  */                                                             //
  byte FLError = DataReadSI4432(buf, len, ID_SIGNATUR, RF_TIME_OUT);
  if (FLError == 1) {                                            //     
    char Str[buf[0]];for(byte i =0; i<=(buf[0]-1); i++) Str[i] = buf[i+1+1+4];
    byte x[4];for(byte i = 0; i < 4; i++) x[i] = buf[i+2];long *RFData = (long *)&x;
    #ifdef Debug_RF-DATA                                         //
    Serial.print("RSSI: ");Serial.println((int8_t)(-120 + (RF22_Si4432.lastRssi()/2)), DEC);
    Serial.print("got request: ");Serial.print(Str);Serial.print("; String Len:");Serial.print(sizeof(Str));Serial.print("; RFData: ");Serial.print((RFData[0]/1000), DEC);Serial.println("S");
    Serial.println("");                                          //
    #endif                                                       //
    delay(10);                                                   //
  } else {if (FLError == 3) {delay(3000);Si4432_init(OperatingFreq, Modulations, RF22_TXPOW_MAXDBM);}}
 //------------------------------------------------------------- //
 
 //------------------------------------------------------------- //
}                                                                //
//***END_loop*************************************************** //
#endif                                                           //

