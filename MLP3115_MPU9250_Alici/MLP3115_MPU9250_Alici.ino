//#include <EBYTE.h> kullanılmasa da olur
//#include <SoftwareSerial.h>
#include <LoRa_E32.h>
#include <Wire.h>
static const int RXPin = 19, TXPin = 18;
//SoftwareSerial portlora(RXPin,TXPin);
//SoftwareSerial mySerial(18, 19); //TX, RX
LoRa_E32 e32ttl(&Serial1);


#define Address 2 //0--65000 arası bir değer girebilirsiniz. Diğer Modüllerden FARKLI olmalı
#define Channel 23 //Frekans değeri (410 + 23) (E32 için 0-31 arası) 
#define M0 10
#define M1 11

struct Signal{
  float pressure;
  float altitude;
  float temperature;
  float AccelX;
  float AccelY;
  float AccelZ;
  float GyroX;
  float GyroY;
  float GyroZ;
  float MagX;
  float MagY;
  float MagZ;
} data;
void LoraE32Configuration() {
  ResponseStructContainer c;
  c = e32ttl.getConfiguration();
  Configuration configuration = *(Configuration*)c.data;
 
  //DEĞİŞEBİLEN AYARLAR
  // Üstte #define kısmında ayarlayınız
  configuration.ADDL = lowByte(Address);
  configuration.ADDH = highByte(Address);
  configuration.CHAN = Channel;
 
  //SEÇENEKLİ AYARLAR
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;  //Veri Gönderim Hızı 2,4 varsayılan
  //configuration.SPED.airDataRate = AIR_DATA_RATE_000_03;  //Veri Gönderim Hızı 0,3 En uzak Mesafe
  //configuration.SPED.airDataRate = AIR_DATA_RATE_101_192; //Veri Gönderim Hızı 19,2 En Hızlı
 
  configuration.OPTION.transmissionPower = POWER_20;  //Geönderim Gücü max Varsayılan
  //configuration.OPTION.transmissionPower = POWER_10;  //Geönderim Gücü min
  //configuration.OPTION.transmissionPower = POWER_30; // E32 30d modülleri için bunu aktif et
 
  //GELİŞMİŞ AYARLAR
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  configuration.OPTION.fec = FEC_0_OFF;
  //configuration.OPTION.fec = FEC_1_ON;
  configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
  //configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
 
  // Ayarları KAYDET ve SAKLA
  ResponseStatus rs = e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  c.close();
}
void setup() {
  Wire.begin();
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  Serial.begin(9600);
  e32ttl.begin();

  LoraE32Configuration();
 
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
 
  delay(500);
  Serial.println("Sensorlerden veri alimina baslaniyor..");

}

void loop() {
  while (e32ttl.available() > 1) {
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    struct Signal data = *(Signal*)rsc.data;
    rsc.close();
    delay(500);
    //Wire.requestFrom(4,6);
    /*while (Wire.available()){
      char c = Wire.read();
      Serial.print(c);
    }*/
    delay(500);
    Serial.println("-----------------");
    Serial.print("Gelen Basinc Verisi:");
    Serial.print(data.pressure);
    Serial.println(" m");
    Serial.print("Gelen Yukseklik Verisi:");
    Serial.print(data.altitude);
    Serial.println(" hPa");
    Serial.print("Gelen Sicaklik Verisi:");
    Serial.print(data.temperature);
    Serial.println(" C");
  Serial.println("-----------------");
  Serial.print("X Eksenindeki İvme = "); Serial.println(data.AccelX);
  Serial.print("Y Eksenindeki İvme = "); Serial.println(data.AccelY);
  Serial.print("Z Eksenindeki İvme = "); Serial.println(data.AccelZ);
  Serial.print("X Eksenindeki Aci = "); Serial.println(data.GyroX);
  Serial.print("Y Eksenindeki Aci = "); Serial.println(data.GyroY);
  Serial.print("Z Eksenindeki Aci = "); Serial.println(data.GyroZ);
  Serial.print("X Eksenindeki Manyetizma = "); Serial.println(data.MagX);
  Serial.print("Y Eksenindeki Manyetizma = "); Serial.println(data.MagY);
  Serial.print("Z Eksenindeki Manyetizma = "); Serial.println(data.MagZ);
  
  }
}
