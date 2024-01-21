//Encoder setting
#define  ENCODER_OPTIMIZE_INTERRUPTS //countermeasure of encoder noise
#include <Encoder.h>
#include <Bounce2.h>


//Oled setting
#include<Wire.h>
#include<Math.h>
#include<Adafruit_GFX.h>
#include<Adafruit_SSD1306.h>

#define OLED_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
int b=0;
unsigned long counter=0;
int oldTrigTime = 0;
bool oldTrigExt = 0;
// Pins
int pin_clk_4 = A1;
int pin_clk_6 = A2;
int pin_clk_8 = A3;
int pin_clk_12 = A6;
int pin_clk_16 = A7;
int eucPin[3] = {6, 7, 8 };

// Euclydian rhythm
const static byte euc16[17][16] PROGMEM = {//euclidian rythm
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
 {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
 {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
 {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
 {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
 {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0},
 {1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0},
 {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0},
 {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
 {1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0},
 {1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1},
 {1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1},
 {1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1},
 {1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1},
 {1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

//each channel param
byte hits[3] = { 4, 4, 5 };//each channel hits
byte offset[3] = { 0, 2, 0 };//each channele step offset
bool mute[3] = {0, 0, 0 }; //mute 0 = off , 1 = on
int limit[3] = {16, 16, 16 };//eache channel max step
byte playing_step[3] = {0, 0, 0 };

//rotery encoder
Encoder myEnc(2, 3);
int optChange = 0;
int oldEnc  = -999;
int newEnc = -999;

// General variables
bool trigIn = 0;
bool oldTrigIn = 0;
int clock_pulse = 0;
unsigned long gate_timer = 0;
unsigned long previousMillis = 0;
const int numBeats = 5;
const int beatPins[numBeats] = {pin_clk_4, pin_clk_6, pin_clk_8, pin_clk_12, pin_clk_16};
const int beatValues[numBeats] = {24, 18, 12, 8, 6};
unsigned long select_time = 0;

// Display
byte select_menu = 0;
int select_ch = 0;
int bpm = 0;
bool disp_refresh = 1;
// Drawing the circles
const int r_circle = 18;
const double step_size = 2 * M_PI / 16;
const byte graph_x[3] = {22, 64, 106 }; // Center point of circles
const byte graph_y[3] = {24, 24, 24 }; // Center point of circles
int x16[16];  
int y16[16]; 
// Sequence variables
byte j = 0;
byte k = 0;
byte m = 0;
byte buf_count = 0;
// Drawing the lines
bool offset_buf[3][16];
byte line_xbuf[17];//Buffer for drawing lines
byte line_ybuf[17];//Buffer for drawing lines

// User BPM
const float min_bpm = 60;
const float max_bpm = 150;
float steps_bpm = (max_bpm - min_bpm) / 1024;

int mode = 0;
Bounce debouncer = Bounce();  // Create a Bounce object

void get_coordinates(int radius) { // Function to get coordinates mathematically
  for (int i = 0; i < 16; ++i) {
    double angle_section = i * step_size;
    x16[i] = radius * cos(angle_section);
    y16[i] = radius * sin(angle_section);
  }
}

void setup() {
  // OLED setting
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  Serial.begin(9600);

  get_coordinates(r_circle);
  OLED_display(0,0, mode, 120);

  pinMode(pin_clk_4, OUTPUT);
  pinMode(pin_clk_6, OUTPUT);
  pinMode(pin_clk_8, OUTPUT);
  pinMode(pin_clk_12, OUTPUT);
  pinMode(pin_clk_16, OUTPUT);
  pinMode(2, INPUT_PULLUP); 
  pinMode(3, INPUT_PULLUP); 
  pinMode(4, INPUT_PULLUP); //BUTTON
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(12, INPUT_PULLUP); 
  // pinMode(A0, INPUT_PULLUP); 
  debouncer.attach(4);
  debouncer.interval(5); 
}

void loop() {
  // int clockValue = digitalRead(12);
  // if (clockValue == LOW) {
  //   Serial.println("Cable plugged in");
  // }
  // else {
  //   Serial.println("Cable plugged out");   
  // }

  debouncer.update(); 
  if (debouncer.rose()){
    mode = 1 - mode;
    if (mode == 1) {
      select_time = millis();
      disp_refresh = 1;
    }
    switch (select_menu) {
      case 4: //mute
        mute[select_ch] = !mute[select_ch];
        mode = 0;
        break;

      case 5: //reset
        for (k = 0; k <= 3; k++) {
          playing_step[k] = 0;
        }
        mode = 0;
        break;
    }
  }

  newEnc = myEnc.read() / 4;
  optChange = up_down(oldEnc, newEnc);
  oldEnc = newEnc;
  
  if (optChange != 0) {
    select_time = millis(); 
    disp_refresh = 1;

    if (mode == 0) {
      select_menu = get_menu(optChange, select_menu);
    }
    else { 
      switch (select_menu) {
        case 0: //select chanel
          select_ch += optChange;
          if (select_ch > 2) {
            select_ch = 0;
          }
          else if (select_ch < 0) {
            select_ch = 2;
          }
          break;
        
        case 1: //hits
          hits[select_ch] += optChange;
          if (hits[select_ch] >= 17) {
            hits[select_ch] = 1;
          }
          else if (hits[select_ch] <= 0) {
            hits[select_ch] = 16;
          }
          break;

        case 2: //offset
          offset[select_ch] += optChange;
          if (offset[select_ch] >= 16) {
            offset[select_ch] = 0;
          }
          else if (offset[select_ch] < 0) {
            offset[select_ch] = 16;
          }
          break;

        case 3: //limit
          limit[select_ch] += optChange ;
          if (limit[select_ch] >= 17) {
            limit[select_ch] = 0;
          }
          else if (limit[select_ch] < 0) {
            limit[select_ch] = 17;
          }
          break;
      }
    }
  }
  if ((millis() - select_time) > 5000) {
    mode = 0;
  } 

  //-----------------offset setting----------------------
  for (k = 0; k <= 2; k++) { 
    for (int i = offset[k]; i <= 15; i++) {
      offset_buf[k][i - offset[k]] = (pgm_read_byte(&(euc16[hits[k]][i]))) ;
    }

    for (int i = 0; i < offset[k]; i++) {
      offset_buf[k][16 - offset[k] + i] = (pgm_read_byte(&(euc16[hits[k]][i])));
    }
  }

  //-----------------trigger detect & output----------------------
  // clock_pulse = userBPM();
  // trigIn = (clock_pulse % 24 == 0 ? 1 : 0);
  // int bpm = BPM(120);
  int trigExt = digitalRead(12); //external trigger in
  if (trigExt == 1) {
    counter++;
  } else {
    counter = 0;
  }
  
  if (counter > 10) {
    // For testing fix the clockrate
    int clock_rate = 900; // analogRead(A0);
    bpm = round(min_bpm + (clock_rate * steps_bpm));
    trigIn = BPM(bpm); // (clock_pulse % 24 == 0 ? 1 : 0);
  } else {
    trigIn = trigExt;
    bpm = 0;
  }

  
  if (oldTrigIn == 0 && trigIn == 1) {
    gate_timer = millis();
    for (int i = 0; i <= 2; i++) {
      playing_step[i]++;      //When the trigger in, increment the step by 1.
      if (playing_step[i] >= limit[i]) {
        playing_step[i] = 0;  //When the step limit is reached, the step is set back to 0.
      }
    }
    for (k = 0; k <= 2; k++) {//output gate signal
      if (offset_buf[k][playing_step[k]] == 1 && mute[k] == 0) {
        digitalWrite(eucPin[k], HIGH);
      }
    }
    disp_refresh = 1;
  }
  if (gate_timer + 10 <= millis()) { //off all gate , gate time is 10msec
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }

  oldTrigIn = trigIn;
  
  // disp_refresh = 1;
  if (disp_refresh == 1) { // For debugging
    OLED_display(select_ch, select_menu, mode, bpm);
    disp_refresh = 0;
  }
  else {
    delay(3); // Don't know why this is needed but otherwise it hangs (after implementing the input clk switch thing)
  }

}

int get_menu(int change, int select_menu) {
  select_menu = select_menu + change;
  if (select_menu < 0) {
    select_menu = 5;
  }
  else if (select_menu > 5 ) {
    select_menu = 0;
  }
  return select_menu;
}

int up_down(int oldOptEnc, int newOptEnc) {
  if ( (oldOptEnc - newOptEnc) > 0  ) {//turn left
    optChange = -1;
  }
  else if ( (newOptEnc - oldOptEnc) > 0 ) {//turn right
    optChange = 1;
  }
  else {
    optChange = 0;
  }
  return optChange;
}

int BPM(int bpm) {
  float beatInterval = 60000 / (bpm);
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= beatInterval) {
    previousMillis = currentMillis;
    return 1;
  } else {
    return 0;
  }
}

int userBPM() {
  // int clock_rate = analogRead(pin_bpm_input);
  // int bpm = round(min_bpm + (clock_rate * steps_bpm));
  int bpm = 120;
  float beatInterval = 60000 / (bpm * 24);
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= beatInterval) {
    clock_pulse ++;
    previousMillis = currentMillis; 
    writePulse(clock_pulse);
  }
  return clock_pulse;
}

void writePulse(int clock_pulse) {
  for (int i = 0; i < numBeats; ++i) {
    digitalWrite(beatPins[i], clock_pulse % beatValues[i] == 0 ? HIGH : LOW); // clock divider
  }
  if (clock_pulse == 72) {
    clock_pulse = 0;
  }
}

void OLED_display(int select_ch, int select_menu, int mode, int bpm) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(2, 54);
  display.print(select_ch + 1);
  display.setCursor(12, 54);
  display.print("HITS");
  display.setCursor(40, 54);
  display.print("OFF");
  display.setCursor(65, 54);
  display.print("LIM");
  display.setCursor(89, 54);
  display.print("M");
  display.setCursor(100, 54);
  display.print("R");
  display.setCursor(110, 54);
  display.fillRect(109, 51, 19, 13, WHITE);
  display.setTextColor(BLACK);
  if (bpm != 0) {
    display.print(bpm);
  } else {
    display.print("EXT");
  }
  
  

  //draw select box
  if ( select_menu == 0) {
    display.drawRect(0, 51, 9, 13, WHITE);
  }
  else if ( select_menu == 1) {
    display.drawRect(9, 51, 28, 13, WHITE);
  }
  else if ( select_menu == 2) {
    display.drawRect(37, 51, 24, 13, WHITE);
  }
  else if ( select_menu == 3) {
    display.drawRect(61, 51, 25, 13, WHITE);
  }
  else if ( select_menu == 4) {
    display.drawRect(86, 51, 11, 13, WHITE);
  }
  else if ( select_menu == 5) {
    display.drawRect(97, 51, 11, 13, WHITE);
  }
  // draw select circle
  if (mode == 1) {
    display.drawCircle(graph_x[select_ch], graph_y[select_ch], 21, WHITE);
  }

  //draw step dot
  for (k = 0; k <= 2; k++) { 
    for (j = 0; j <= limit[k] - 1; j++) { 
      display.drawPixel(graph_x[k] + x16[j], graph_y[k] + y16[j], WHITE);
    }
    buf_count = 0;
    for (m = 0; m < 16; m++) {
      if (offset_buf[k][m] == 1) {
        line_xbuf[buf_count] = x16[m] + graph_x[k];//store active step
        line_ybuf[buf_count] = y16[m] + graph_y[k];
        buf_count++;
      }
    }
    for (j = 0; j < buf_count - 1; j++) {
      display.drawLine(line_xbuf[j], line_ybuf[j], line_xbuf[j + 1], line_ybuf[j + 1], WHITE);
    }
    display.drawLine(line_xbuf[0], line_ybuf[0], line_xbuf[j], line_ybuf[j], WHITE);
  }
  for (j = 0; j < 16; j++) {//line_buf reset
    line_xbuf[j] = 0;
    line_ybuf[j] = 0;
  }
  for (k = 0; k <= 2; k++) { //ch count
    buf_count = 0;
    if (hits[k] == 1) {
      display.drawLine(graph_x[k], graph_y[k], x16[offset[k]] + graph_x[k], y16[offset[k]] + graph_y[k], WHITE);
    }
  }
  //draw play step circle
  for (k = 0; k <= 2; k++) { //ch count
    if (mute[k] == 0) { //mute on = no display circle
      if (offset_buf[k][playing_step[k]] == 0) {
        display.drawCircle(x16[playing_step[k]] + graph_x[k], y16[playing_step[k]] + graph_y[k], 2, WHITE);
      }
      if (offset_buf[k][playing_step[k]] == 1) {
        display.fillCircle(x16[playing_step[k]] + graph_x[k], y16[playing_step[k]] + graph_y[k], 3, WHITE);
      }
    }
  }
  display.display();
}
