#include <Adafruit_MPL3115A2.h>
//#include <EBYTE.h> kullanılamasa da olur
//#include <SoftwareSerial.h>
#include <LoRa_E32.h>
#include <MPU9250.h>
#include <Wire.h>
Adafruit_MPL3115A2 sensor; //class adı, kendi belirlediğimiz veri ismi
MPU9250 IMU(Wire,0x68); 
static const int RXPin = 19, TXPin = 18;
//SoftwareSerial mySerial(18, 19); //TX, RX
LoRa_E32 e32ttl(&Serial1); //class adı, kendi belirlediğimiz değişken + rx ve tx in bağlandığı serial port numarası
#define Address 1 //0--65000 arası bir değer girebilirsiniz. Diğer Modüllerden FARKLI olmalı
#define Channel 23 //Frekans değeri (410 + 23) (E32 için 0-31 arası) 
#define AddressToBeSend 2
#define M0 10
#define M1 11
//int status;
struct Signal {
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
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
 
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
  Wire.begin(); //ı2c haberleşmelerini başlat.
  //Wire.onRequest(requestEvent);
  Serial.begin(9600);
  while(!Serial);

  if (!sensor.begin()) {
    Serial.println("Sensor bulunamadi, baglantiyi kontrol edin.");
    while(1); }
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  e32ttl.begin();
  LoraE32Configuration();
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
 
  delay(500);
  Serial.println("Sensor verileri gonderiliyor...");
  sensor.setSeaPressure(1013.26);

  /*if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    
  }*/
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

}
/*void requestEvent(){
  Wire.write("hello");
}*/
void loop() {
  data.pressure = sensor.getPressure();
  data.altitude = sensor.getAltitude();
  data.temperature = sensor.getTemperature();
  data.AccelX = IMU.getAccelX_mss();
  data.AccelY = IMU.getAccelY_mss();
  data.AccelZ = IMU.getAccelZ_mss();
  data.GyroX = IMU.getGyroX_rads();
  data.GyroY = IMU.getGyroY_rads();
  data.GyroZ = IMU.getGyroZ_rads();
  data.MagX = IMU.getMagX_uT();
  data.MagY = IMU.getMagY_uT();
  data.MagZ = IMU.getMagZ_uT();
  ResponseStatus rs = e32ttl.sendFixedMessage(highByte(AddressToBeSend), lowByte(AddressToBeSend), Channel, &data, sizeof(Signal));
  delay(500);
  Serial.println("-----------------");
  Serial.print("Basinc = "); Serial.print(data.pressure); Serial.println(" hPa");
  Serial.print("Yukseklik = "); Serial.print(data.altitude); Serial.println(" m");
  Serial.print("Sicaklik = "); Serial.print(data.temperature); Serial.println(" C");
  IMU.readSensor();
  //portlora.listen();

  //ResponseStatus rs = e32ttl.sendFixedMessage(0, 3, 7, &data, sizeof(Signal));
  //Serial.println(rs.getResponseDescription());

  Serial.println("-----------------");
  Serial.print("X Eksenindeki İvme = "); Serial.println(data.AccelX);
  Serial.print("Y Eksenindeki İvme = "); Serial.println(data.AccelY);
  Serial.print("Z Eksenindeki İvme = "); Serial.println(data.AccelZ);
  Serial.print("X Eksenindeki Aci = "); Serial.println(data.GyroX);
  Serial.print("Y Eksenindeki Aci = "); Serial.println(data.GyroY);
  Serial.print("Z Eksenindeki Aci = "); Serial.println(data.GyroZ);
  Serial.print("X Eksenindeki Manyetizma = "); Serial.println(data.MagX);
  Serial.print("Y Eksenindeki Manyetizma = "); Serial.println(data.MagY);
  Serial.print("Z Eksenindeki Manyetizma = "); Serial.println(data.MagZ);}