//Código PI2 Kenner Marqueti Couto

#include <SoftwareSerial.h>
#include <DHT.h>
#include <BlynkSimpleSerialBLE.h> 
#define BLYNK_PRINT Serial
#define DHTPIN 22 // pino que estamos conectado o DHT
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

int aSensor,
    temp = 0x00,   
    umidade = 0x00,
    rt,
    buf,
    ru;
int sensor;
int GAS = 23;
int LED = 13; // the on-board LED
int Data; // the data received
int luz = 53;
int nluz = 52;
int inetervalo;
int probe = 49;
int tempo;
int tmenos;


SoftwareSerial Bluetooth(10, 9); // RX, TX

 
void setup() {
  Bluetooth.begin(9600);
  Serial.begin(9600);
  Serial.println("Waiting for command...");
  Bluetooth.println("Aquecendo Sensor de Gás, por favor aguarde.");
  pinMode(LED,OUTPUT);
  pinMode(luz,OUTPUT);
  pinMode(nluz, OUTPUT);
  pinMode(probe,OUTPUT);
  dht.begin();
  delay(2000);
  Bluetooth.println("Pronto!");
}
 
void loop() {

  //leitura do dht
  dht.read(DHTPIN);
  temp = dht.readTemperature();
  umidade = dht.readHumidity();

  //LEITURA SENSOR DE GÁS

  sensor = digitalRead(GAS);
  if (sensor == LOW){
    Bluetooth.println("Gases tóxicos ou Inflamáveis!!!");
    delay (2000);
    }
  // ignora erros de leitura
  if (temp != 0 || umidade !=0){
    rt = temp;
    ru = umidade;
}
  

//sensor de umidade
if (ru > 90){
   // Bluetooth.println("umidade alta");
    digitalWrite(LED,1);
    }
    else if (ru < 90){
      digitalWrite(LED,0);
      }

  //código bluetooth
  
  if (Bluetooth.available()){ //wait for data received
    Data=Bluetooth.read();
         
    if (Data=='1'){  
     digitalWrite(luz,1);
     buf = 1;
     digitalWrite(nluz,0);
     
     Serial.println("Luz acesa!");
     Bluetooth.println("Luz acesa!!");
   }
    else if(Data=='0'){
       digitalWrite(luz,0);
       buf =0;
       digitalWrite(nluz,1);
       Serial.println("Luz apagada!");
       Bluetooth.println("Luz apagada! ");
    
   }
   
     Bluetooth.print("Umidade: ");
     Bluetooth.print(ru);
     Bluetooth.println("%");
     Bluetooth.print("Temperatura: ");
     Bluetooth.print(rt);
     Bluetooth.println(" C");
    
     if (buf == '0') {
          Bluetooth.println("Luz apagada."); 
      }
      else if (buf == '1')
      {
        Bluetooth.println("Luz acesa.");
      }
    
    }

     
   
    delay(100);  
  }
