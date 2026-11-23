/*
  SafeDoor BLE - DEMO / 16 NeoPixels
  OrangeBoard BLE (Arduino Uno), SoftwareSerial D4 RX / D5 TX
  SONAR: TRIG/ECHO: D2/D3, A0/A1, D7/D8, D10/D11
  PIR D9, WS2812B D6 (16 pixels), ACTIVE BUZZER D12

  DEMO ONLY: sonar timeouts (999) count as no detected obstruction.
  A disconnected/failed sonar can therefore incorrectly clear an alarm.
  Not suitable for a real fire-egress safety installation.

  BLE 9600 baud, newline-delimited:
   T,state,person,object,d1,d2,d3,d4,seconds,threshold,enabled,muted
   E,event,seconds
   GET, MUTE,1 / MUTE,0, ENABLE,1 / ENABLE,0, DIST,5..150
*/
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <stdlib.h>
#include <string.h>

SoftwareSerial ble(4, 5);  // RX, TX
const byte TRIG[4] = {2, A0, 7, 10};
const byte ECHO[4] = {3, A1, 8, 11};
const byte PIR_PIN = 9, LED_PIN = 6, BUZZER_PIN = 12;
const byte NUM_LEDS = 16;
const byte BRIGHTNESS = 60;
const byte DEBOUNCE = 3;

const unsigned long PIR_WARMUP_MS = 20000UL;
const unsigned long PIR_HOLD_MS = 2000UL;
const unsigned long YELLOW_MS = 1000UL;
const unsigned long RED_MS = 3000UL;
const unsigned long ADMIN_MS = 6000UL;
const unsigned long RECHECK_MS = 2000UL;
const unsigned long ADMIN_REPEAT_MS = 10000UL;
const unsigned long SEND_MS = 1000UL;

const byte CLEAR=0, PERSON=1, COUNTING=2, YELLOW=3,
           RED=4, ADMIN=5, RECHECK=6, OFF=7;
const char* NAMES[] = {"CLEAR","PERSON","COUNTING","YELLOW",
                       "RED","ADMIN","RECHECK","OFF"};
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

byte state = CLEAR, resumeState = CLEAR;
int dist[4] = {999, 999, 999, 999};
int detectCm = 50;
bool present = false, seenPir = false, enabled = true, muted = false;
byte hits = 0, misses = 0;
unsigned long objectStart = 0, recheckStart = 0;
unsigned long lastPir = 0, lastAdmin = 0, lastSend = 0;
char cmd[32];
byte cmdLen = 0;

void sendEvent(const char* e, unsigned long now) {
  ble.print(F("E,")); ble.print(e); ble.print(',');
  ble.println(objectStart ? (now-objectStart)/1000UL : 0UL);
  Serial.print(F("[EVENT] ")); Serial.println(e);
}

void changeState(byte next, unsigned long now) {
  if (next == state) return;
  Serial.print(NAMES[state]); Serial.print(F(" -> "));
  Serial.println(NAMES[next]);
  state = next;
  sendEvent(NAMES[next], now);
  if (next == ADMIN) lastAdmin = now;
}

int readCm(byte i) {
  digitalWrite(TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[i], LOW);
  unsigned long us = pulseIn(ECHO[i], HIGH, 10000UL);
  if (us == 0) return 999;
  return (int)(us / 58UL);
}

void updateObject() {
  bool raw = false;
  for (byte i = 0; i < 4; i++) {
    dist[i] = readCm(i);
    if (dist[i] > 2 && dist[i] <= detectCm) raw = true;
    delay(25);
  }
  // DEMO: all 999 -> raw false -> count toward clearing.
  if (raw) {
    misses = 0;
    if (hits < DEBOUNCE) ++hits;
    if (hits >= DEBOUNCE) present = true;
  } else {
    hits = 0;
    if (misses < DEBOUNCE) ++misses;
    if (misses >= DEBOUNCE) present = false;
  }
}

byte severity(unsigned long elapsed) {
  if (elapsed >= ADMIN_MS) return ADMIN;
  if (elapsed >= RED_MS) return RED;
  if (elapsed >= YELLOW_MS) return YELLOW;
  return COUNTING;
}

void paint(byte r, byte g, byte b) {
  uint32_t color = strip.Color(r, g, b);
  for (byte i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, color);
  strip.show();
}

void outputs(unsigned long now) {
  const bool slow = (now / 500UL) % 2;
  const bool fast = (now / 250UL) % 2;
  // During removal confirmation, keep prior warning until actually cleared.
  byte visual = (state == RECHECK) ? resumeState : state;
  switch (visual) {
    case CLEAR: case COUNTING: paint(0,255,0); break;
    case PERSON: if (slow) paint(0,255,0); else paint(0,0,0); break;
    case YELLOW: paint(255,130,0); break;
    case RED: if (slow) paint(255,0,0); else paint(0,0,0); break;
    case ADMIN: if (fast) paint(255,0,0); else paint(0,0,0); break;
    case OFF: paint(0,0,0); break;
  }
  const bool alarm = (visual == RED || visual == ADMIN);
  digitalWrite(BUZZER_PIN, alarm && !muted && (visual == ADMIN ? fast : slow) ? HIGH : LOW);
}

void telemetry(unsigned long now, bool person) {
  ble.print(F("T,")); ble.print(NAMES[state]); ble.print(',');
  ble.print(person ? 1 : 0); ble.print(',');
  ble.print(present ? 1 : 0);
  for (byte i = 0; i < 4; ++i) {
    ble.print(','); ble.print(dist[i]);
  }
  ble.print(',');
  ble.print(objectStart && state != CLEAR && state != OFF && state != PERSON
              ? (now-objectStart)/1000UL : 0UL);
  ble.print(','); ble.print(detectCm);
  ble.print(','); ble.print(enabled ? 1 : 0);
  ble.print(','); ble.println(muted ? 1 : 0);
}

void execute(char* line, unsigned long now) {
  bool person = seenPir && (now-lastPir < PIR_HOLD_MS);
  if (!strcmp(line, "GET")) { telemetry(now, person); return; }
  if (!strcmp(line, "MUTE,1")) { muted = true; sendEvent("MUTED", now); return; }
  if (!strcmp(line, "MUTE,0")) { muted = false; sendEvent("UNMUTED", now); return; }
  if (!strcmp(line, "ENABLE,0")) {
    enabled = false;
    present = false; hits = misses = 0; objectStart = 0;
    changeState(OFF, now); return;
  }
  if (!strcmp(line, "ENABLE,1")) {
    enabled = true;
    present = false; hits = misses = 0; objectStart = 0;
    changeState(CLEAR, now); return;
  }
  if (!strncmp(line, "DIST,", 5)) {
    char* end;
    long val = strtol(line+5, &end, 10);
    if (*end == '\0' && val >= 5 && val <= 150) {
      detectCm = (int)val;
      sendEvent("DIST_CHANGED", now);
    }
  }
}

void commands(unsigned long now) {
  while (ble.available()) {
    char c = (char)ble.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen) {
        cmd[cmdLen] = '\0';
        execute(cmd, now);
        cmdLen = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (cmdLen < sizeof(cmd)-1) cmd[cmdLen++] = c;
      else cmdLen = 0;
    }
  }
}

void setup() {
  Serial.begin(9600);
  ble.begin(9600);
  for (byte i = 0; i < 4; ++i) {
    pinMode(TRIG[i], OUTPUT);
    digitalWrite(TRIG[i], LOW);
    pinMode(ECHO[i], INPUT);
  }
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  strip.begin(); strip.setBrightness(BRIGHTNESS); strip.show();
  Serial.println(F("SafeDoor DEMO 16LED: PIR warmup 20 sec"));
  unsigned long start = millis();
  while (millis()-start < PIR_WARMUP_MS) {
    if ((millis()/500UL)%2) paint(40,40,40);
    else paint(0,0,0);
    commands(millis());
    delay(50);
  }
  paint(0,255,0);
  Serial.println(F("SafeDoor ready - DEMO timeout clears enabled"));
}

void loop() {
  unsigned long now = millis();
  commands(now);
  if (!enabled) {
    outputs(now);
    if (now-lastSend >= SEND_MS) {
      telemetry(now, false);
      lastSend = now;
    }
    return;
  }

  if (digitalRead(PIR_PIN) == HIGH) { lastPir = now; seenPir = true; }
  updateObject();
  now = millis();
  if (digitalRead(PIR_PIN) == HIGH) { lastPir = now; seenPir = true; }
  bool person = seenPir && (now-lastPir < PIR_HOLD_MS);
  unsigned long elapsed = now-objectStart;

  switch (state) {
    case CLEAR:
      if (present) {
        if (person) changeState(PERSON, now);
        else { objectStart = now; changeState(COUNTING, now); }
      } else if (person) changeState(PERSON, now);
      break;
    case PERSON:
      if (!person && present) {
        objectStart = now;
        changeState(COUNTING, now);
      } else if (!person && !present) {
        changeState(CLEAR, now);
      }
      break;
    case COUNTING: case YELLOW: case RED: case ADMIN:
      // PIR must not cancel an alarm that already started.
      if (!present) {
        resumeState = state;
        recheckStart = now;
        changeState(RECHECK, now);
      } else {
        changeState(severity(elapsed), now);
        if (state == ADMIN && now-lastAdmin >= ADMIN_REPEAT_MS) {
          sendEvent("ADMIN_REPEAT", now);
          lastAdmin = now;
        }
      }
      break;
    case RECHECK:
      if (present) {
        changeState(severity(elapsed), now);
      } else if (now-recheckStart >= RECHECK_MS) {
        // Report removal before clearing timer, then reset app time to zero.
        sendEvent("REMOVED", now);
        objectStart = 0;
        resumeState = CLEAR;
        changeState(person ? PERSON : CLEAR, now);
      }
      break;
    case OFF:
      changeState(CLEAR, now);
      break;
  }

  outputs(now);
  if (now-lastSend >= SEND_MS) {
    telemetry(now, person);
    lastSend = now;
  }
  commands(millis());
}
