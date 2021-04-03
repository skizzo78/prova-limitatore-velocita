


//    MOTORE PER PROVA LIMITATORI DI VELOCITA REGOLATO IN PWM CON RETROAZIONE TRAMITE ENCODER

     //encoder = collegare canale A su pin digitale 2
     //driver l298n  ingresso pwm = pin 9 arduino
     //display oled ssd1306   scl=pin A5   sda=pinA4
     //potenziometro pin anolog 0 arduino
     //calcolo 145 impulsi encoder per 2 metri (motore ADV) 
     //pin 8 per pulsante programmazione



#include <EEPROMex.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 32 , &Wire, 4);

#define disp2 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(2);
#define disp3 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(3);
#define disp4 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(4);
#define pwmPin 9
#define potenziometroPin A0
#define debounce_delay 2000

#define t_PID 100                       //tempo aggiornamento pid in msec
#define n_impulsi_PID 15                //circa 1/10 di n_impulsi per scala max 2m/sec (con aggiornamento a 0,1 sec)
int n_impulsi = EEPROM.readInt(0);      //circa 145 impulsi per fare 2 metri (motore adv)

bool first_eeprom_read = true;          //flag per una sola lettora eeprom
bool first_print = true;                //flag per una sola trasmissione al display

volatile unsigned int enc = 0;          //contatore impulsi per gestione PID (durata 0,1 sec)
volatile unsigned int enc2 = 0;         //contatore impulsi per rilevamento velocita (durata 1 sec)
unsigned long vel_millis = 0;
unsigned long PID_millis = 0;
unsigned long t = 0;
int stato = 0;
int pwm = 0 ;
int vel_richiesta = 0 ;
int velocita = 0;
float velocita_m_s = 0;


void setup() {

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.display();
  display.clearDisplay();

  Serial.begin (9600);
  Serial.println("seriale ok");
  pinMode(8, INPUT_PULLUP);             //pin programmazione
  pinMode(2, INPUT_PULLUP);             //pin encoder
  pinMode(pwmPin, OUTPUT);              //pin pwm
  attachInterrupt(0, ai0, RISING);
 
  if (digitalRead (8) == 0) {
    stato = 1;
    disp2 display.setCursor(0, 2); display.print("rilasciareil pulsante"); display.display();
    delay (4000);
  }

  disp4; display.setCursor(0, 2); display.print(EEPROM.readInt(0)); display.display();
  delay(2000);

}

void loop() {

  switch (stato) {
    case 0: mainloop(); break;
    case 1: menu_prog_exit(); break;
    case 2: menu_prog_up(); break;
    case 21: menu_prog_up_sub(); break;
    case 3: menu_prog_down(); break;
    case 31: menu_prog_down_sub(); break;
    case 4: menu_prog_save(); break;
  }
}

void mainloop() {

  if (first_eeprom_read == true) {
    n_impulsi = EEPROM.readInt(0);
    first_eeprom_read = false;
  }

  int valorePotenziometro = analogRead(potenziometroPin);
  vel_richiesta = map(valorePotenziometro, 0, 1023, 0, n_impulsi_PID); /////  n_impulsi_PID

  if ( (millis() - PID_millis) > t_PID) {

    if (vel_richiesta > enc) {
      pwm = pwm + (vel_richiesta - enc);
    }
    if (vel_richiesta <= enc ) {
      pwm = pwm - (enc - vel_richiesta);
    }
    if (pwm > 255) {
      pwm = 255;
    }
    if (pwm < 0) {
      pwm = 0;
    }
    if (vel_richiesta == 0) {
      pwm = 0;
    }
    analogWrite(pwmPin, pwm);

    // Serial.println (enc);


    if (digitalRead(8) == 0) {
      display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
      display.setCursor(0, 0); display.print("pwm %");
      display.setCursor(60, 0); display.println(map(pwm , 0, 255, 0, 100));
      display.setCursor(0, 7); display.print("v_ric");
      display.setCursor(60, 7); display.println(vel_richiesta);
      display.setCursor(0, 14); display.print("v_enc");
      display.setCursor(60, 14); display.println(enc);
      display.setCursor(0, 21); display.print("m/s");
      display.setCursor(60, 21); display.println(velocita_m_s );
      display.display();
    }
    else {
      disp4  display.setCursor(0, 2);
      display.println(velocita_m_s);
      display.setTextSize(1); display.setCursor(105, 22); display.print ("M/s");
      display.display();
    }

    PID_millis = millis();
    enc = 0;
    
  }


  
  if ( (millis() - vel_millis) > 1000) {

    velocita = map (enc2 , 0 , n_impulsi , 0 , 2000 );    // n_impulsi (145) motore ADV
    velocita_m_s = ((float)velocita / 1000 );
    vel_millis = millis();
    enc2 = 0;
  
  }

}

///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_exit() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("uscire");
    display.display();
    first_print == false;

  }


  if ( digitalRead(8) == 0) {
    setStato(0);
  }



  if (enc > 1) {
    setStato(2);
  }

}

///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_up() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("aumenta");
    display.display();
    first_print == false;

  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print == false;
        setStato(21);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }



  if (enc > 1) {
    setStato(3);
  }


}

///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_up_sub() {



  if (first_print == true) {
    disp4; display.setCursor(0, 2); display.print(n_impulsi);
    display.display();
    first_print == false;
  }

  if (enc > 1) {
    n_impulsi = ++n_impulsi;
    enc = 0 ;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print == false;
        setStato(2);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }


}

///////////////////\\\\\\\\\\\\\\\\\\


void menu_prog_down() {


  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print == false;
        setStato(31);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }



  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("diminuisci");
    display.display();
    first_print == false;
  }



  if (enc > 1) {
    setStato(4);
  }


}

///////////////////\\\\\\\\\\\\\\\\\\


void menu_prog_down_sub() {



  if (first_print == true) {
    disp4; display.setCursor(0, 2); display.print(n_impulsi);
    display.display();
    first_print == false;
  }

  if (enc > 1) {
    n_impulsi = --n_impulsi;
    enc = 0 ;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print == false;
        setStato(3);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }



}

///////////////////\\\\\\\\\\\\\\\\\\


void menu_prog_save() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("premere   5 sec per salvare");
    display.display();
    first_print == false;

  }


  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        EEPROM.updateInt(0, n_impulsi);//145adv 3600 fly   dato provvisorio
        delay(100);
        disp2;
        display.setCursor(0, 2); display.print("salva");
        display.setCursor(70, 2); display.print(EEPROM.readInt(0));
        display.display();
        delay(2000);
        setStato(0);
        break;
      }
    }
  }



  if (enc > 1) {
    setStato(1);
  }


}

///////////////////\\\\\\\\\\\\\\\\\\

void setStato(int s) {
  stato = s;
  enc = 0;
  first_print = true;
  display.clearDisplay();
  // delay(2000);
}

///////////////////\\\\\\\\\\\\\\\\\\

void ai0() {
  enc++;
  enc2++;    // impulsi x 2 metri
  first_print = true;
}





////////////////////END OF FILE\\\\\\\\\\\\\\\\\\\\
