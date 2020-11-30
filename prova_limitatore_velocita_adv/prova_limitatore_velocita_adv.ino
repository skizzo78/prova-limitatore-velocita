


//                         COLLEGAMENTI ARDUINO  ENCODER_5V  DISPLAY_OLED

// collegare un canale encoder su pin digitale D2
//(ogni impulso attiva un interrupt per contatore)
// collegare display_OLED_I2C 128x32  pin SCL = A5 , pin SDA = A4 , alimentare a 5 volt



#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     4

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


volatile unsigned int counter = 0;         // contatore impulsi encoder
unsigned long current_millis = 0;          // variabile tempo aggiornamento impulsi encoder

int impulsi = 145;       //impulsi x 2metri , varia in base a diametro puleggia e risoluzione encoder

void setup() {


  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.display();
  display.clearDisplay();

  pinMode(2, INPUT_PULLUP);             // imposta pin per segnale encoder (2) come input + pullup
  attachInterrupt(0, ai0, RISING);      // imposta interrupt

}


void loop() {

  if ( (millis() - current_millis) > 1000) {
    // tempo di aggiornamento di 1 sec

    int velocita = map (counter , 0 , impulsi , 0 , 2000 );
    // funzione map per calcolare velocita (impulsi x 2 metri)

    float velocita_m_s = ((float)velocita / 1000 );
    // divido per convertire da millimetri/sec in metri/sec

    display.clearDisplay();
    display.setTextSize(4); display.setTextColor(WHITE); display.setCursor(0, 2);
    display.println(velocita_m_s);
    display.setTextSize(1); display.setTextColor(WHITE); display.setCursor(105, 22);
    display.print ("M/s");
    display.display();

    current_millis = millis();            // aggiorna millis
    counter = 0;                          // azzera contatore
  }
}


void ai0() {            // aumenta contatore ad ogni interrupt
  counter++;
}
