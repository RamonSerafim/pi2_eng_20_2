Curso Superior em Engenharia Eletrônica 

Unidade Curricular: Projeto Integrador  II

Professores: Luiz Alberto de Azevedo e Fernando Henriques de Miranda

Aluno: Ramon Busiquia Serafim



# Domótica



#### INTRODUÇÃO 

A tecnologia utilizada para a automação de residências para o controle automático de dispositivos eletrônicos é chamada de Domótica, não precisar se mover para usar dispositivos, conseguir utilizá -los remotamente por aplicativos e comandos de voz. Este conceito veio para reduzir o esforço sobre algumas atividades e também melhorar a segurança sobre outras. 



Casas programadas com rotinas semanais, abrir e fechar cortinas/janelas, ligar e desligar aparelhos eletrônicos fazendo com que facilitem a vivência nesses locais. Apesar de ser pouco divulgada, ela visa a comodidade, parecendo ser coisas simples que podem alterar o modo de vida tanto de um ambiente residencial como comercial. Parte dessa tecnologia vem sendo aplicada para o público idoso, visando uma melhor segurança para eles sem se preocupar com possíveis riscos triviais. Sensores de gás, água, fumaça podem diminuir drasticamente o risco de ocorrer acidentes. Câmeras que enviam imagens para o celular dos seus responsáveis em qualquer lugar. Hora sendo vista como utopia, hoje vem sendo aplicada em grande escala melhorando o ambiente de vivencia.



#### DESIGN

Com as ideias prontas, foi feita uma representação em software para visualizar melhor a montagem na fase de implementação. O primeiro croqui feito é demonstrado na foto abaixo.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/Croqui%20Ramon%20Serafim.png?raw=true">

Após alguns testes e visualizar problemas que serão descritos no decorrer do projeto o Croqui final e o projeto final ficou no modelo da foto abaixo.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/Croqui%20final%20Ramon%20Serafim.png?raw=true">



#### Implementação e componentes utilizados.



**Arduino Mega 2560**

Uma placa microcontrolada baseada no ATmega2560. Possui 54 pinos de entrada/saída digital. Contém tudo que é necessário para dar suporte ao microcontrolador; funciona ao se conectar a um computador por USB ou um adaptador AC-DC ou uma bateria. Compatível com a maioria de blindagens projetadas para o Uno.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/arduino%20mega%202560.jpg?raw=true">

**Sensor de Gás MQ-2**

Módulo confiável e simples de usar em seus projetos de automação residencial, capaz de detectar concentrações de gases combustíveis e fumaça no ar.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/sensor%20gas.png?raw=true">

**Módulo Buzzer 5V Passivo**

Por ser passivo permite que você tenha mais controle sobre a melodia, utilizando diferentes frequências. Resultando em um som mais limpo.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/modulo%20buzzer.jpg?raw=true">

**Sensor de Presença e Movimento PIR HC-SR501**

Consegue detectar o movimento de objetos que estejam em uma área de até 7 metros! Caso algo ou alguém se movimentar nesta área o pino I/O recebe valor alto.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/sensor%20pir.png?raw=true">

 **Sensor de Água**

Utilizado para medir nível ou profundidade de água em um recipiente. Dependendo de sua programação pode medir diferentes volumes de água.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/sensor%20agua.png?raw=true">

**Sensor de Umidade e Temperatura DHT11**

Um sensor de temperatura e umidade que permite fazer leituras de temperaturas entre 0 a 50 Celsius e umidade entre 20 a 90%

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/sensor%20Dht11.jpg?raw=true">

**Sensor de distância Ultrassônico HC-SR04**

Capaz de medir distâncias de 2cm a 4m com ótima precisão e baixo preço. Este módulo possui um circuito pronto com emissor e receptor acoplados.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/sensor%20ultrassonico.jpg?raw=true">

**Display LCD 16X2**

Display que possui 16 colunas e 2 linhas para escrita, utilizado onde precise de uma visualização entre o projeto e o operador.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/lcd%2016x2.jpeg?raw=true">

**Sensor LDR - Light Dependent Resistor**

Um resistor dependente de luz ou fotorresistência. Utilizador como sensor de luz devido ao seu baixo custo e facilidade de utilização.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/sensor%20ldr.jpg?raw=true">

**Componentes secundários**

Foram utilizados 3 Leds de alto brilho nas cores verde, vermelho e lilás. Implementado 4 resistores, sendo 3 no valor de 1kΩ para os leds e um de 101kΩ para o LDR. Um potenciômetro de 100K para controlar o contraste no LCD 16X2.



#### Código Programação

```
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>      // usando a biblioteca LiquidCrystal
#include "dht.h" //INCLUSÃO DE BIBLIOTECA
#include "Ultrasonic.h"            //INCLUSÃO DAS BIBLIOTECAS NECESSÁRIA PARA FUNCIONAMENTO DO CÓDIGO

//SoftwareSerial SerialBLE(10, 11); // RX, TX
//#define BLYNK_PRINT SerialBLE
//#define BLYNK_USE_DIRECT_CONNECT

const int pinoSensorAgua = A1;     //PINO ANALÓGICO UTILIZADO PELO SENSOR
const int pinoDHT11 = A2;          //PINO ANALÓGICO UTILIZADO PELO DHT11
const int pinoLDR = A3;            //PINO ANALÓGICO UTILIZADO PELO LDR
const int pinoBuzzer = 3;          //PINO DIGITAL UTILIZADO PELO BUZZER
const int pinoSensorGas = 4;       //PINO DIGITAL UTILIZADO PELO SENSOR
const int trigPin = 5;             //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)
const int echoPin = 6;             //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)
const int pinoSensorPIR = 7;       //PINO DIGITAL UTILIZADO PELO SENSOR DE PRESENÇA
const int pinoLED0 = 8;            //PINO DIGITAL UTILIZADO PELO LED AGUA
const int pinoLED1 = 9;            //PINO DIGITAL UTILIZADO PELO LED PIR
const int pinoLED2 = 12;           //PINO DIGITAL UTILIZADO PELO LED LDR
const int rs = 28, rw = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22; // definicao dos pinos do Display

LiquidCrystal lcd(rs, rw, en, d4, d5, d6, d7);                // configurando os pinos
Ultrasonic ultrasonic(trigPin, echoPin); //INICIALIZANDO OS PINOS DO ARDUINO

int distancia; //VARIÁVEL DO TIPO INTEIRO
String result; //VARIÁVEL DO TIPO STRING
dht DHT;       //VARIÁVEL DO TIPO DHT

//char auth[] = "---";  // TOKEN PARA O APP BLYNK.

//Array simbolo grau
byte grau[8] ={ B00001100,
                B00010010,
                B00010010,
                B00001100,
                B00000000,
                B00000000,
                B00000000,
                B00000000,};

void setup()
{
  pinMode(pinoSensorAgua, INPUT);     //DEFINE O PINO COMO ENTRADA
  pinMode(pinoBuzzer, OUTPUT);        //DEFINE O PINO COMO SAÍDA
  pinMode(pinoSensorGas, INPUT);      //DEFINE O PINO COMO ENTRADA
  pinMode(echoPin, INPUT);            //DEFINE O PINO COMO ENTRADA (RECEBE)
  pinMode(trigPin, OUTPUT);           //DEFINE O PINO COMO SAIDA (ENVIA)
  pinMode(pinoSensorPIR, INPUT);      //DEFINE O PINO COMO ENTRADA
  pinMode(pinoLDR, INPUT);            //DEFINE O PINO COMO ENTRADA
  pinMode(pinoLED0, OUTPUT);          //DEFINE O PINO COMO SAÍDA
  pinMode(pinoLED1, OUTPUT);          //DEFINE O PINO COMO SAÍDA
  pinMode(pinoLED2, OUTPUT);          //DEFINE O PINO COMO SAÍDA

  Serial.begin(9600);                 //INICIALIZA A PORTA SERIAL
  //SerialBLE.begin(38400);
  //Blynk.begin(SerialBLE, auth);

  lcd.begin(16, 2); //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY. EM SUMA: UMA MATRIZ DE 16 COLUNAS E 2 LINHAS
  lcd.createChar(0, grau);
}

void loop()
{
  Blynk.run();

  // ROTINA SENSOR DE AGUA
  if (analogRead(pinoSensorAgua)> 100) //SE A LEITURA DO PINO FOR MAIOR QUE 690 BITS (PODE SER AJUSTADO), FAZ
  { 
      digitalWrite(pinoLED0, HIGH); //ACENDE O LED
  }else{ //SENÃO, FAZ
    digitalWrite(pinoLED0, LOW); //APAGA O LED
  }

  // ROTINA SENSOR DE GAS
  if (digitalRead(pinoSensorGas) == LOW)
  {                                 //SE A LEITURA DO PINO FOR IGUAL A LOW, FAZ
    pinMode(pinoBuzzer, LOW); //LIGA O BUZZER
  }
  else
  {                            //SENÃO, FAZ
    pinMode(pinoBuzzer, HIGH); //DESLIGA O BUZZER
  }

  // ROTINA SENSOR ULTRASSONICO
  hcsr04();                   // FAZ A CHAMADA DO MÉTODO "hcsr04()"
  Serial.print("Distancia "); //IMPRIME O TEXTO NO MONITOR SERIAL
  Serial.print(result);       ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
  Serial.println("cm");       //IMPRIME O TEXTO NO MONITOR SERIAL

  // ROTINA SENSOR PIR
  if (digitalRead(pinoSensorPIR) == HIGH)
 { 
      digitalWrite(pinoLED1, HIGH); //ACENDE O LED
  }else{ //SENÃO, FAZ
    digitalWrite(pinoLED1, LOW);    //APAGA O LED
  }

  //ROTINA SENSOR LDR
  if(analogRead(pinoLDR) > 600){ //SE O VALOR LIDO FOR MAIOR QUE 600, FAZ
    digitalWrite(pinoLED2, HIGH); //ACENDE O LED
  }  
  else{ //SENÃO, FAZ
    digitalWrite(pinoLED2, LOW); //APAGA O LED
  }

  //ROTINA SENSOR UMIDADE
  DHT.read11(pinoDHT11); //LÊ AS INFORMAÇÕES DO SENSOR
  Serial.print("Umidade: "); //IMPRIME O TEXTO NA SERIAL
  Serial.print(DHT.humidity); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO
  Serial.print("%"); //ESCREVE O TEXTO EM SEGUIDA
  Serial.print(" / Temperatura: "); //IMPRIME O TEXTO NA SERIAL
  Serial.print(DHT.temperature, 0); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO E REMOVE A PARTE DECIMAL
  Serial.println("*C"); //IMPRIME O TEXTO NA SERIAL
  delay(2000); //INTERVALO DE 2 SEGUNDOS * NÃO DIMINUIR ESSE VALOR

  //ROTINA LCD
  float h = DHT.humidity;     //Le o valor da umidade
  float t = DHT.temperature;  //Le o valor da temperatura

  //FUNÇAO TEMPERATURA NO LCD
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.setCursor(5,0);
  lcd.print(t,1);
  lcd.setCursor(9,0);
  lcd.write((byte)0); //Mostra o simbolo do grau formado pelo array

  //FUNÇAO UMIDADE NO LCD
  lcd.setCursor(0,1);
  lcd.print("Umid: ");
  lcd.setCursor(5,1);
  lcd.print(h,1);
  lcd.setCursor(9,1);
  lcd.print("%");

  //FUNÇAO DISTANCIA NO LCD
  lcd.setCursor(11,0);
  lcd.print("Dist:");
  lcd.setCursor(11,1);
  lcd.print(distancia,1);
  lcd.setCursor(14,1);
  lcd.print("cm");

  //Intervalo recomendado para leitura do sensor
  delay(2000);

}

// FUNÇÃO DO SENSOR ULTRASSONICO
void hcsr04()
{
  digitalWrite(trigPin, LOW);  //SETA O PINO 6 COM UM PULSO BAIXO "LOW"
  delayMicroseconds(2);        //INTERVALO DE 2 MICROSSEGUNDOS
  digitalWrite(trigPin, HIGH); //SETA O PINO 6 COM PULSO ALTO "HIGH"
  delayMicroseconds(10);       //INTERVALO DE 10 MICROSSEGUNDOS
  digitalWrite(trigPin, LOW);  //SETA O PINO 6 COM PULSO BAIXO "LOW" NOVAMENTE
  //FUNÇÃO RANGING, FAZ A CONVERSÃO DO TEMPO DE
  //RESPOSTA DO ECHO EM CENTIMETROS, E ARMAZENA
  //NA VARIAVEL "distancia"
  distancia = (ultrasonic.Ranging(CM)); //VARIÁVEL GLOBAL RECEBE O VALOR DA DISTÂNCIA MEDIDA
  result = String(distancia);           //VARIÁVEL GLOBAL DO TIPO STRING RECEBE A DISTÂNCIA(CONVERTIDO DE INTEIRO PARA STRING)
  delay(2000);                           //INTERVALO DE 500 MILISSEGUNDOS
}


```

#### Operação



Após conectar e ajeitar todos os componentes na protobord e na mesa, o projeto ficou com esse formato.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/circuito%20na%20mesa.jpg?raw=true">

O visor LCD representando um painel central, os leds de sinalização abaixo dele e os sensores de presença, ultrassônico e de gás mais separados para dar a ideia de que eles podem estar mais distante do ponto central do projeto, representando outros cômodos em uma residência.



Afim de não deixar todo o circuito a mostra e para colocar um certo grau de proteção no mesmo foi decidido colocar em uma caixa, porém o circuito possui muitas conexões.

<img src="https://github.com/RamonSerafim/pi2_eng_20_2/blob/main/RAMON%20BUSIQUIA%20SERAFIM/Projeto%20Final/PI2%20CAIXA%20FALHA.jpg?raw=true">

Com uma parte fixada no local foi feitos novos testes e a partir desse momento foi detectado muitos erros por mau contato nos componentes. Vendo que não conseguiria implementar essa parte foi deixada de lado e o projeto voltou a ser o da foto anterior. Sem os maus contatos decidimos executar o projeto para testes. Após alguns minutos conseguimos verificar que todas as implementações estavam funcionando perfeitamente e ao mesmo tempo, sem problema de código ou algo do gênero.



#### CONSIDERAÇÕES FINAIS 

Após finalizar o projeto e revisar o que iria ser implementado no começo do mesmo conseguimos ver uma grande diferença entres os mesmos. Devido algumas importunidades de projeto como o Módulo Buzzer funcionar somente para outro módulo, HC-05 não tem pareamento bluetooth com dispositivos IOS, sensor LDR possui uma programação fixada em si e para alterar precisamos de outros componentes. Como não foi possível implementar o módulo bluetooth, decidi implementar o LCD 16X2 para visualizar a umidade e temperatura juntamente com a distância do sensor ultrassônico. Algumas futuras melhorias para o projeto seria a implementação de um módulo wi-fi no lugar do bluetooth, pois facilita a conexão e não precisa ficar pareando a todo momento, possuindo conexão com IOS e android. Mais módulos Buzzer para todas as implementações necessárias e o Relé conectado para ligar um controlador de umidade presente no cômodo instalado.
