// aggiornato il 05/2023


//    MOTORE PER PROVA LIMITATORI DI VELOCITA REGOLATO IN PWM CON RETROAZIONE TRAMITE ENCODER

//encoder = collegare canale A su pin digitale 2
//driver l298n  ingresso pwm = pin 9 arduino
//display oled ssd1306   scl=pin A5   sda=pinA4
//calcolo 145 impulsi encoder per 2 metri (motore ADV)
//pin 7 & 8 per programmazione/velocita



#include <EEPROMex.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 32 , &Wire, 4);

#define disp1 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);
#define disp2 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(2);
#define disp3 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(3);
#define disp4 display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(4);
#define enc_a 2
#define up 7
#define down 8
#define pwmPin 9
#define debounce_delay 2000
#define t_step 500                      //tempo debaunce pulsanti
#define t_PID 100                       //tempo aggiornamento pid in msec
#define n_impulsi_PID 15                //circa 1/10 di n_impulsi per scala max 2m/sec (con aggiornamento a 0,1 sec)
int n_impulsi = EEPROM.readInt(0);      //circa 145 impulsi per fare 2 metri (motore adv) (ridurre della meta per aggiornamento a 500ms)
int Kp = EEPROM.readInt(3);              //proporzionale PID

bool first_eeprom_read = true;          //flag per una sola lettora eeprom
bool first_print = true;                //flag per una sola trasmissione al display
bool first_step = true;                 //flag per debaunce pulsanti
volatile unsigned int enc = 0;          //contatore impulsi per gestione PID (durata 0,1 sec)
volatile unsigned int enc2 = 0;         //contatore impulsi per rilevamento velocita (durata 1 sec)
unsigned long vel_millis = 0;
unsigned long PID_millis = 0;
unsigned long t = 0;                    //millis per debounce
int stato = 0;
int pwm = 0 ;
int vel_richiesta = 0 ;
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
  pinMode(up, INPUT_PULLUP);             //pin programmazione
  pinMode(down, INPUT_PULLUP);             //pin programmazione
  pinMode(2, INPUT_PULLUP);             //pin encoder
  pinMode(pwmPin, OUTPUT);              //pin pwm
  attachInterrupt(0, ai0, RISING);

  if (digitalRead (8) == 0) {
    stato = 1;
    disp2 display.setCursor(0, 2); display.print("rilasciarepulsante"); display.display();
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
    case 4: menu_prog_up_pid(); break;
    case 41: menu_prog_up_pid_sub(); break;
    case 5: menu_prog_down_pid(); break;
    case 51: menu_prog_down_pid_sub(); break;
    case 6: menu_prog_save(); break;
  }
}

void mainloop() {

  if (first_eeprom_read == true) {
    n_impulsi = EEPROM.readInt(0);
    Kp = EEPROM.readInt(3);
    first_eeprom_read = false;
  }


  if ( digitalRead(up) == 0 && first_step == true) {
    vel_richiesta = ++ vel_richiesta;

    t = millis();
    first_step = false;
  }

  if ( (millis() - t) > t_step) {
    first_step = true;
  }


  if ( digitalRead(down) == 0 && first_step == true) {
    vel_richiesta = -- vel_richiesta;

    t = millis();
    first_step = false;
  }

  if ( (millis() - t) > t_step) {
    first_step = true;
  }



  vel_richiesta = constrain(vel_richiesta , 0, 16);



  if ( (millis() - PID_millis) > t_PID) {

    if (vel_richiesta > enc) {
      pwm = pwm + (( vel_richiesta - enc) * Kp);
    }
    if (vel_richiesta <= enc ) {
      pwm = pwm - ((enc - vel_richiesta) * Kp);
    }

    pwm = constrain(pwm , 0, 255);

    if (vel_richiesta == 0) {
      pwm = 0;
    }
    analogWrite(pwmPin, pwm);




    PID_millis = millis();
    enc = 0;

  }



  if ( (millis() - vel_millis) > 500) {

    int velocita_mm_s = map (enc2 , 0 , n_impulsi , 0 , 2000 );    // n_impulsi (145) motore ADV
    velocita_m_s = ((float)velocita_mm_s / 1000 );


    disp4  display.setCursor(0, 2);
    display.println(velocita_m_s);
    display.setTextSize(1); display.setCursor(100, 22); display.print ("M/s");
    display.setCursor(105, 10); display.println(vel_richiesta);
    display.setCursor(122, 0); display.print("%");
    display.setCursor(105, 0); display.println(map(pwm , 0, 255, 0, 100));
    display.display();


    vel_millis = millis();
    enc2 = 0;

  }

}

///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_exit() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("esci senza salvare");
    display.display();
    first_print = false;

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
    disp2; display.setCursor(0, 2); display.print("diminuisciK_velocita");
    display.display();
    first_print = false;

  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
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
    disp2; display.setCursor(0, 5); display.print(n_impulsi);
    display.setCursor(35, 5); display.print("imp/gir");
    display.display();
    first_print = false;
  }

  if (enc > 1) {
    n_impulsi = ++n_impulsi;
    enc = 0 ;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
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




  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("aumenta   K_velocita");
    //  disp2; display.setCursor(10, 2); display.print("K_encoder");

    display.display();
    first_print = false;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
        setStato(31);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }




  if (enc > 1) {
    setStato(4);
  }


}

///////////////////\\\\\\\\\\\\\\\\\\


void menu_prog_down_sub() {



  if (first_print == true) {
    disp2; display.setCursor(0, 5); display.print(n_impulsi);
    display.setCursor(35, 5); display.print("imp/gir");
    display.display();
    first_print = false;
  }

  if (enc > 1) {
    n_impulsi = --n_impulsi;
    enc = 0 ;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
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
///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_up_pid() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("aumenta   K_pid");
    display.display();
    first_print = false;

  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
        setStato(41);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }



  if (enc > 1) {
    setStato(5);
  }


}

///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_up_pid_sub() {



  if (first_print == true) {
    disp4; display.setCursor(0, 2); display.print(Kp);
    display.display();
    first_print = false;
  }

  if (enc > 1) {
    Kp = ++ Kp;
    enc = 0 ;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
        setStato(4);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }


}
///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_down_pid() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("diminuisciK_pid");
    display.display();
    first_print = false;

  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
        setStato(51);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }



  if (enc > 1) {
    setStato(6);
  }


}

///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_down_pid_sub() {



  if (first_print == true) {
    disp4; display.setCursor(0, 2); display.print(Kp);
    display.display();
    first_print = false;
  }

  if (enc > 1) {
    Kp = -- Kp;
    enc = 0 ;
  }



  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        first_print = false;
        setStato(5);
        disp4; display.setCursor(0, 2); display.print("wait");
        display.display();
        delay(2000);
        break;
      }
    }
  }


}

///////////////////\\\\\\\\\\\\\\\\\\
///////////////////\\\\\\\\\\\\\\\\\\

void menu_prog_save() {

  if (first_print == true) {
    disp2; display.setCursor(0, 2); display.print("prem 5sec per salva");
    display.display();
    first_print = false;

  }


  if ( digitalRead(8) == 0) {
    t = millis();
    while (( digitalRead(8) == 0)) {
      if ( (millis() - t) > debounce_delay) {
        EEPROM.updateInt(0, n_impulsi);//145adv 3600 fly   dato provvisorio
        EEPROM.updateInt(3, Kp);//proporzionale pid
        delay(100);
        disp2;
        display.setCursor(0, 2); display.print("salva");
        display.setCursor(70, 2); display.print(EEPROM.readInt(0));
        display.setCursor(100, 2); display.print(EEPROM.readInt(3));
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
