//Encoder setting
#define  ENCODER_OPTIMIZE_INTERRUPTS //countermeasure of encoder noise
#include <Encoder.h>
#include <Bounce2.h>

//Oled setting
#include<Wire.h>
#include<Adafruit_GFX.h>
#include<Adafruit_SSD1306.h>

#define OLED_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pins
const byte eucPin[3] = {5, 6, 7 };
const byte rotPin[2] = {14, 15};
const byte clkPin = 2;
const byte divPin = 8;

// Clock divider
uint16_t pulseTimes[5]; 
unsigned long currentPulse = 0; 
unsigned long oldPulse = 0;
unsigned long lightPulse = 0;  
uint16_t pulseTime = 0;

int bpmDisp = 0;
int bpm = 120;
byte divPulses[3] = { 1, 1, 1 };

// Euclydian rhythma
const static bool euc16[17][16] PROGMEM = {//euclidian rythm
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
byte limit[3] = {16, 16, 16 };//eache channel max step
byte multiplier[3] = {1, 1, 1 };
byte playing_step[3] = {0, 0, 0 };
byte select_menu = 0;
byte clk_division = 1;
byte select_ch = 0;

//rotery encoder
Encoder myEnc(rotPin[0], rotPin[1]);
int8_t optChange = 0;
int oldEnc  = -999;
int newEnc = -999;

// General variables
bool trigIn = 0;
bool oldTrigIn = 0;
unsigned long previousMillis = 0;
unsigned long select_time = 0;

// Display
bool disp_refresh = 1;
// Drawing the circles
const byte graph_x[3] = {22, 64, 106 }; // Center point of circles
const byte graph_y[3] = {24, 24, 24 }; // Center point of circles
int8_t x16[16] = { 18, 17, 13, 7, 0, -7, -13, -17, -18, -17, -13, -7, 0, 7, 13, 17};
int8_t y16[16] = { 0, 7, 13, 17, 18, 17, 13, 7, 0, -7, -13, -17, -18, -17, -13, -7};

// Drawing the lines
bool offset_buf[3][16];
byte line_xbuf[17];//Buffer for drawing lines
byte line_ybuf[17];//Buffer for drawing lines

// Trigger
unsigned long counter=0;
uint8_t pulseCount = 0;
uint8_t bufferIndex = 0;
byte mode = 0;
bool clk_source = 0; // 0 is internal, 1 is external

// Buttons
bool new_reset_state = 0;
bool old_reset_state = 1;
Bounce channel_select = Bounce();
Bounce debouncer = Bounce();  // Create a Bounce object

void setup() {
  // OLED setting
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  Serial.begin(115200);
  
  // get_coordinates(r_circle, step_size);
  OLED_display(0,0, mode, 120, 1);

  pinMode(rotPin[0], INPUT_PULLUP); // Rotary encoder 1
  pinMode(rotPin[1], INPUT_PULLUP); // Rotary encoder 2 
  pinMode(9, INPUT); // Rotary encoder switch

  pinMode(eucPin[2], OUTPUT); // Euclidean CH3
  pinMode(eucPin[1], OUTPUT); // Euclidean CH2
  pinMode(eucPin[0], OUTPUT); // Euclidean CH1
  pinMode(divPin, OUTPUT); // Clock divider
  pinMode(A7, INPUT); // Reset pattern
  pinMode(10, INPUT); // channel select pattern
  pinMode(clkPin, INPUT_PULLUP); // Clock in

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);

  debouncer.attach(9);
  debouncer.interval(5); 
  channel_select.attach(10);
  channel_select.interval(5); 
}

void loop() {
  debouncer.update(); 
  channel_select.update();
  new_reset_state = analogRead(A7) * 2 / 1024;

  if (debouncer.rose()){
    disp_refresh = 1;
    if (mode == 2) {
      mode = 0;
    }
    else if (mode == 1 and (millis()-select_time < 250)) {
      mode = 2;  
    }
    else {
      mode = 1 - mode;
      if (mode == 1) {
        select_time = millis();
      }
      if (select_menu == 3) { //mute
          mute[select_ch] = !mute[select_ch];
          mode = 0;
      }
    }
  }
  if (channel_select.fell()){
    select_ch += 1;
    if (select_ch > 2) {
      select_ch = 0;
    }
    disp_refresh = 1;
  }
  
  // Reset button
  if (new_reset_state == 0 & old_reset_state == 1 ) {
    disp_refresh = 1;
    for (byte k = 0; k <= 3; k++) {
      playing_step[k] = 0;
    }
    
  }
  old_reset_state = new_reset_state;

  // Rotary encoder
  optChange = up_down(oldEnc, newEnc);
  oldEnc = newEnc;

  if (optChange != 0) {
    select_time = millis(); 
    disp_refresh = 1;

    if (mode == 0) {
      select_menu = get_menu(optChange, select_menu);
    }
    else if (mode == 2) {
      clk_division = get_clk_division(optChange, clk_division);
    }
    else { 
      switch (select_menu) {
        case 0: //hits
          hits[select_ch] += optChange;
          if (hits[select_ch] >= 17) {
            hits[select_ch] = 1;
          }
          else if (hits[select_ch] <= 0) {
            hits[select_ch] = 16;
          }
          break;

        case 1: //offset
          offset[select_ch] += optChange;
          if (offset[select_ch] >= 16) {
            offset[select_ch] = 0;
          }
          else if (offset[select_ch] < 0) {
            offset[select_ch] = 16;
          }
          break;

        case 2: //limit
          limit[select_ch] += optChange ;
          if (limit[select_ch] >= 17) {
            limit[select_ch] = 0;
          }
          else if (limit[select_ch] < 0) {
            limit[select_ch] = 17;
          }
          break;

        case 4: //multiplier
          multiplier[select_ch] += optChange ;
          if (multiplier[select_ch] > 4) {
            multiplier[select_ch] = 1;
          }
          else if (multiplier[select_ch] < 1) {
            multiplier[select_ch] = 4;
          }
          break;

        case 5: //bpm
          bpm += optChange;
          break;
      }
    }
  }
  if ((millis() - select_time) > 5000) {
    mode = 0;
  } 

  //-----------------offset setting----------------------
  for (byte k = 0; k <= 2; k++) { 
    for (int i = offset[k]; i <= 15; i++) {
      offset_buf[k][i - offset[k]] = (pgm_read_byte(&(euc16[hits[k]][i]))) ;
    }

    for (int i = 0; i < offset[k]; i++) {
      offset_buf[k][16 - offset[k] + i] = (pgm_read_byte(&(euc16[hits[k]][i])));
    }
  }

  //-----------------trigger detect & output----------------------
  bool trigExt = 1 - digitalRead(clkPin); //external trigger in

  if (trigExt == 1) {
    counter = millis();
  } 
  
  if (millis() - counter > 1000) {
    trigIn = BPM(bpm); 
    bpmDisp = bpm;
  } else {
    trigIn = trigExt;
    bpmDisp = 0;
  }
  
  if (oldTrigIn == 0 && trigIn == 1) {
    currentPulse = millis();

    // CLOCK DIVIDER
    if (pulseCount < 5) {
      pulseCount++;
    }
    pulseTime = calculateAvg(oldPulse, currentPulse, pulseCount); 
    oldPulse = currentPulse;
    
    for (int i = 0; i <= 2; i++) {
      advance_step(i);
    }
    digitalWrite(divPin, HIGH);
    lightPulse = currentPulse;

    divPulses[0] = 1;
    divPulses[1] = 1;
    divPulses[2] = 1;
    disp_refresh = 1;
  }

  for (int i = 0; i <= 2; i++) { 
    int divider = i + 2;
    int clkInt = pulseTime/divider;
    if (clock_divider(clkInt, divider)) {
      if (divider == clk_division) {
        digitalWrite(divPin, HIGH);
        lightPulse = millis();
      }

      for (int channel = 0; channel <= 2; channel++) {
        if (multiplier[channel] == divider){
          advance_step(channel);
          disp_refresh = 1;
          lightPulse = millis();
        }
      }
    }
  }

  if (lightPulse + 10 <= millis()) { //off all gate , gate time is 10msec
    digitalWrite(divPin, LOW);
    digitalWrite(eucPin[0], LOW);
    digitalWrite(eucPin[1], LOW);
    digitalWrite(eucPin[2], LOW);
  }

  oldTrigIn = trigIn;

  if (disp_refresh == 1) {
    OLED_display(select_ch, select_menu, mode, bpmDisp, clk_division);
    disp_refresh = 0;
  }
}

ISR (PCINT1_vect) {
  newEnc = myEnc.read() / 4;
}


void advance_step(int i) {
  playing_step[i]++; //When the trigger in, increment the step by 1.
  if (playing_step[i] >= limit[i]) {
    playing_step[i] = 0; //When the step limit is reached, the step is set back to 0.
  }
  if (offset_buf[i][playing_step[i]] == 1 && mute[i] == 0) {
    digitalWrite(eucPin[i], HIGH);
  }
}

bool clock_divider(int clkInt, int divider) {
  if ((millis() - currentPulse >= clkInt*divPulses[divider-2] && divPulses[divider-2] < divider)) {
    divPulses[divider-2]++;
    return true;
  } else {
    return false;
  }
}

int calculateAvg(unsigned long oldPulse, unsigned long currentPulse, int pulseCount) {
  pulseTimes[bufferIndex] = (currentPulse - oldPulse);
  bufferIndex = (bufferIndex + 1) % 5;  // Circular buffer index update

  int totalPulse = 0;
  for (int i = 0; i < pulseCount; i++) {
    totalPulse += pulseTimes[i]; // Needs logic to tackle average before 10 pulses are reached
  }
  return totalPulse/pulseCount;
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

int get_clk_division(int change, int clk_division) {
  clk_division = clk_division + change;
  if (clk_division < 1) {
    clk_division = 4;
  }
  else if (clk_division > 4 ) {
    clk_division = 1;
  }
  return clk_division;
}

int up_down(int oldOptEnc, int newOptEnc) {
  if ( (oldOptEnc - newOptEnc) > 0  ) { //turn left
    optChange = 1;
  }
  else if ( (newOptEnc - oldOptEnc) > 0 ) { //turn right
    optChange = -1;
  }
  else {
    optChange = 0;
  }
  return optChange;
}

int BPM(int bpm) {
  int beatInterval = 60000 / (bpm);
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= beatInterval) {
    previousMillis = currentMillis; // previousMillis needs to be global
    return 1;
  } else {
    return 0;
  }
}

void OLED_display(int select_ch, int select_menu, int mode, int bpm, int clk_division) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  if (mode == 2){
    display.setCursor(23, 15);
    display.print("Clock division:");
    display.setCursor(66, 30);
    display.print(clk_division);
    display.setCursor(58, 30);
    display.print("/");
  } else {
    display.setCursor(2, 54);
    display.print("HITS");
    display.setCursor(30, 54);
    display.print("OFF");
    display.setCursor(52, 54);
    display.print("LI");
    display.setCursor(67, 54);
    display.print("MU");
    display.setCursor(82, 54);
    display.print("MULT");
    display.setCursor(110, 54);
    display.fillRect(109, 51, 19, 13, WHITE);
    display.setTextColor(BLACK);
    if (bpm != 0) {
      display.print(bpm);
    } else {
      display.print("EXT");
    }

    //draw select box
    switch (select_menu) {
      case 0: // HITS
        display.drawRect(0, 51, 28, 13, WHITE);
        break;
      case 1: // OFFSET
        display.drawRect(27, 51, 23, 13, WHITE);
        break;
      case 2: // LI
        display.drawRect(50, 51, 15, 13, WHITE);
        break;
      case 3: // M 
        display.drawRect(65, 51, 15, 13, WHITE);
        break;
      case 4: // X
        display.drawRect(80, 51, 28, 13, WHITE);
        break;
    }
    // draw active channel
    if (mode == 0) {
      display.fillCircle(graph_x[select_ch], graph_y[select_ch], 2, WHITE);
    }
    
    // draw select circle
    if (mode == 1 and select_menu != 5) {
      display.drawCircle(graph_x[select_ch], graph_y[select_ch], 21, WHITE);
      display.setTextColor(WHITE);
      display.setCursor(graph_x[select_ch]-5, graph_y[select_ch]-3);
      display.print(multiplier[select_ch]);
      display.setCursor(graph_x[select_ch]+2, graph_y[select_ch]-3);
      display.print("X");
    }

    for (byte k = 0; k <= 2; k++) { 
      //draw step dot
      for (byte j = 0; j <= limit[k] - 1; j++) { 
        display.drawPixel(graph_x[k] + x16[j], graph_y[k] + y16[j], WHITE);
      }

      byte buf_count = 0;

      // Draw single hit line
      if (hits[k] == 1) {
        display.drawLine(graph_x[k], graph_y[k], x16[offset[k]] + graph_x[k], y16[offset[k]] + graph_y[k], WHITE);
      }

      for (byte m = 0; m < 16; m++) {
        if (offset_buf[k][m] == 1) {
          line_xbuf[buf_count] = x16[m] + graph_x[k];//store active step
          line_ybuf[buf_count] = y16[m] + graph_y[k];
          buf_count++;
        }
      }
      for (byte j = 0; j < buf_count - 1; j++) {
        display.drawLine(line_xbuf[j], line_ybuf[j], line_xbuf[j + 1], line_ybuf[j + 1], WHITE);
      }
      display.drawLine(line_xbuf[0], line_ybuf[0], line_xbuf[buf_count - 1], line_ybuf[buf_count - 1], WHITE); // Could be wrong

      // draw rotating dot
      if (mute[k] == 0) { //mute on = no display circle
        if (offset_buf[k][playing_step[k]] == 0) {
          display.drawCircle(x16[playing_step[k]] + graph_x[k], y16[playing_step[k]] + graph_y[k], 2, WHITE);
        }
        if (offset_buf[k][playing_step[k]] == 1) {
          display.fillCircle(x16[playing_step[k]] + graph_x[k], y16[playing_step[k]] + graph_y[k], 3, WHITE);
        }
      }
    }
  }

  for (byte j = 0; j < 16; j++) {//line_buf reset
    line_xbuf[j] = 0;
    line_ybuf[j] = 0;
  }
  display.display();
}
