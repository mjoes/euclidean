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

// Pins
const byte eucPin[3] = {8, 5, 4 };
// const int beatPins[3] = {A2, A3, A6};

// Clock divider
unsigned long pulseTimes[5]; 
unsigned long pulseDiffs[4]; 
unsigned long currentPulse = 0; 
unsigned long oldPulse = 0;
unsigned long lightPulse = 0;  
unsigned int pulseTime = 0;
int bpmDisp = 0;
int bpm = 120;
byte divPulses[3] = { 1, 1, 1 };

// Euclydian rhythma
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
byte limit[3] = {16, 16, 16 };//eache channel max step
byte multiplier[3] = {1, 1, 1 };
byte playing_step[3] = {0, 0, 0 };
byte select_menu = 0;
byte select_ch = 0;

//rotery encoder
Encoder myEnc(2, 3);
int optChange = 0;
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
int x16[16];  
int y16[16]; 

// Drawing the lines
bool offset_buf[3][16];
byte line_xbuf[17];//Buffer for drawing lines
byte line_ybuf[17];//Buffer for drawing lines

// Trigger
unsigned long counter=0;
int pulseCount = 0;
int bufferIndex = 0;
byte mode = 0;
Bounce debouncer = Bounce();  // Create a Bounce object

void get_coordinates(int radius, float step_size) { // Function to get coordinates mathematically
  for (int i = 0; i < 16; ++i) {
    double angle_section = i * step_size;
    x16[i] = radius * cos(angle_section);
    y16[i] = radius * sin(angle_section);
  }
}

void setup() {
  const byte r_circle = 18;
  const float step_size = 2 * M_PI / 16;
  // OLED setting
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  Serial.begin(9600);
  
  get_coordinates(r_circle, step_size);
  OLED_display(0,0, mode, 120);

  // pinMode(A1, OUTPUT); NO SPACE LEFT FOR THESE ON BOARD
  // pinMode(A2, OUTPUT); NO SPACE LEFT FOR THESE ON BOARD
  // pinMode(A3, OUTPUT); NO SPACE LEFT FOR THESE ON BOARD
  // pinMode(A6, OUTPUT); NO SPACE LEFT FOR THESE ON BOARD

  pinMode(2, INPUT_PULLUP); // Rotary encoder 1
  pinMode(3, INPUT_PULLUP); // Rotary encoder 2 
  pinMode(9, INPUT_PULLUP); // Rotary encoder switch

  pinMode(4, OUTPUT); // Euclidean CH3
  pinMode(5, OUTPUT); // Euclidean CH2
  pinMode(8, OUTPUT); // Euclidean CH1
  pinMode(7, OUTPUT); // Clock divider
  pinMode(11, INPUT_PULLUP); // Clock in
  debouncer.attach(9);
  debouncer.interval(5); 
}

void loop() {
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
        for (byte k = 0; k <= 3; k++) {
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

        case 6: //multiplier
          multiplier[select_ch] += optChange ;
          if (multiplier[select_ch] > 4) {
            multiplier[select_ch] = 1;
          }
          else if (multiplier[select_ch] < 1) {
            multiplier[select_ch] = 4;
          }
          break;

        case 7: //bpm
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
  int trigExt = digitalRead(11); //external trigger in

  if (trigExt == 1) {
    counter++;
  } else {
    counter = 0;
  }
  
  if (counter > 10) {
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
    pulseTime = calculateAvg(oldPulse, currentPulse, pulseCount); // currently outputs an int, need to have a float??
    oldPulse = currentPulse;
    
    for (int i = 0; i <= 2; i++) {
      advance_step(i);
    }
    // digitalWrite(A1, HIGH);
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
      // digitalWrite(beatPins[i+1], HIGH);
      // lightPulse = millis();

      for (int channel = 0; channel <= 2; channel++) {
        if (multiplier[channel] == divider){
          advance_step(channel);
          disp_refresh = 1;
        }
      }
    }
  }

  if (lightPulse + 10 <= millis()) { //off all gate , gate time is 10msec
    // digitalWrite(A1, LOW);
    // digitalWrite(A2, LOW);
    // digitalWrite(A3, LOW);
    // digitalWrite(A6, LOW);
    digitalWrite(eucPin[0], LOW);
    digitalWrite(eucPin[1], LOW);
    digitalWrite(eucPin[2], LOW);
  }

  oldTrigIn = trigIn;

  if (disp_refresh == 1) {
    OLED_display(select_ch, select_menu, mode, bpmDisp);
    disp_refresh = 0;
  }
  // else {
  //   delay(3); // Don't know why this is needed but otherwise it hangs (after implementing the input clk switch thing)
  // }
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

float calculateAvg(unsigned long oldPulse, unsigned long currentPulse, int pulseCount) {
  pulseTimes[bufferIndex] = (currentPulse - oldPulse);
  bufferIndex = (bufferIndex + 1) % 5;  // Circular buffer index update

  float totalPulse = 0;
  for (int i = 0; i < pulseCount; i++) {
    totalPulse += pulseTimes[i]; // Needs logic to tackle average before 10 pulses are reached
  }
  return totalPulse/pulseCount;
}

int get_menu(int change, int select_menu) {
  select_menu = select_menu + change;
  if (select_menu < 0) {
    select_menu = 7;
  }
  else if (select_menu > 7 ) {
    select_menu = 0;
  }
  return select_menu;
}

int up_down(int oldOptEnc, int newOptEnc) {
  if ( (oldOptEnc - newOptEnc) > 0  ) { //turn left
    optChange = -1;
  }
  else if ( (newOptEnc - oldOptEnc) > 0 ) { //turn right
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
    previousMillis = currentMillis; // previousMillis needs to be global
    return 1;
  } else {
    return 0;
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
  display.setCursor(62, 54);
  display.print("LI");
  display.setCursor(79, 54);
  display.print("M");
  display.setCursor(90, 54);
  display.print("R");
  display.setCursor(100, 54);
  display.print("X");
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
    case 0: 
      display.drawRect(0, 51, 9, 13, WHITE);
      break;
    case 1: 
      display.drawRect(9, 51, 28, 13, WHITE);
      break;
    case 2: 
      display.drawRect(37, 51, 23, 13, WHITE);
      break;
    case 3: // LI
      display.drawRect(59, 51, 17, 13, WHITE);
      break;
    case 4: // M 
      display.drawRect(75, 51, 12, 13, WHITE);
      break;
    case 5: // R
      display.drawRect(87, 51, 11, 13, WHITE);
      break;
    case 6: // X
      display.drawRect(97, 51, 11, 13, WHITE);
      break;
  }
  // draw select circle
  if (mode == 1 and select_menu != 7) {
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

  for (byte j = 0; j < 16; j++) {//line_buf reset
    line_xbuf[j] = 0;
    line_ybuf[j] = 0;
  }
  display.display();
}
