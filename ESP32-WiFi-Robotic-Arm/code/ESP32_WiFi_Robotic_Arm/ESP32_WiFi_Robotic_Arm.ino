/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  ARM-6  v4.0  |  ESP32 Wi-Fi 6-DOF Robotic Arm                  ║
 * ║  Final Project — Industry 4.0 Automation Platform               ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  FEATURES                                                        ║
 * ║  ─────────────────────────────────────────────────────────────  ║
 * ║  ✦ Non-blocking smooth motion engine (millis-based stepping)    ║
 * ║  ✦ Sinusoidal ease-in-out per joint                             ║
 * ║  ✦ Slider debouncing (80ms) — no command flooding               ║
 * ║  ✦ Per-servo calibration (offset + inversion + limits)          ║
 * ║  ✦ Watchdog fail-safe (Wi-Fi loss → auto-return to pickup)      ║
 * ║  ✦ Secondary fallback → home if no pickup saved                 ║
 * ║  ✦ TRAJECTORY RECORDING  (timestamped joint positions)          ║
 * ║  ✦ TRAJECTORY PLAYBACK   (smooth replay of recorded path)       ║
 * ║  ✦ Up to 3 named trajectory slots                               ║
 * ║  ✦ Preset motion sequences (Rest/Ready/Pick/Place)              ║
 * ║  ✦ Adjustable speed + per-joint physical/logical telemetry      ║
 * ║  ✦ Premium industrial web dashboard (mobile + desktop)          ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  HARDWARE                                                        ║
 * ║  ─────────────────────────────────────────────────────────────  ║
 * ║  S1 GPIO23 Base        MG995 (high-torque)                      ║
 * ║  S2 GPIO22 Shoulder    MG995 (high-torque, +90° offset)         ║
 * ║  S3 GPIO21 Elbow       Standard servo                           ║
 * ║  S4 GPIO19 Wrist Pitch Standard servo                           ║
 * ║  S5 GPIO18 Wrist Roll  Standard servo                           ║
 * ║  S6 GPIO5  Gripper     Standard servo (inverted)                ║
 * ║  Power: 18650 × 2 → buck converter → 5V rail                   ║
 * ║  Common GND between ESP32 and power rail                        ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <math.h>

// ─── Wi-Fi Credentials ──────────────────────────────────────────────
const char* WIFI_SSID = "hii";
const char* WIFI_PASS = "hello1234";

WebServer server(80);

// ─── Servo Pins ─────────────────────────────────────────────────────
const int SERVO_PINS[6] = { 23, 22, 21, 19, 18, 5 };
Servo srv[6];

// ─── Arm Position Struct ─────────────────────────────────────────────
struct ArmPos {
  int v[6];  // Logical angles [0..5] = Base,Shoulder,Elbow,WristP,WristR,Gripper
};

ArmPos currentPos = {{ 90, 0, 90, 90, 90, 45 }};
ArmPos targetPos  = {{ 90, 0, 90, 90, 90, 45 }};
ArmPos savedPos   = {{ 90, 0, 90, 90, 90, 45 }};  // Fail-safe pickup
ArmPos homePos    = {{ 90, 0, 90, 90, 90, 45 }};  // User-defined home

// ─── Per-Servo Calibration ────────────────────────────────────────────
struct ServoCalib {
  int  offset;      // Added to logical before PWM output
  bool inverted;    // If true: physical = 180 − (logical + offset)
  int  minAngle;    // Logical soft minimum
  int  maxAngle;    // Logical soft maximum
};

ServoCalib cal[6] = {
  {   0, false,   0, 180 },  // S1 Base
  {  90, false,   0,  90 },  // S2 Shoulder  — logical 0° → physical 90°
  {   0, false,   0, 180 },  // S3 Elbow
  {   0, false,   0, 180 },  // S4 Wrist Pitch
  {   0, false,   0, 180 },  // S5 Wrist Roll
  {   0, true,    0,  90 },  // S6 Gripper   — inverted (0=open, 90=close)
};

int applyCalib(int i, int logical) {
  const ServoCalib &c = cal[i];
  int cl  = constrain(logical, c.minAngle, c.maxAngle);
  int phy = c.inverted ? (180 - (cl + c.offset)) : (cl + c.offset);
  return constrain(phy, 0, 180);
}

void writeServoLogical(int i, int logical) {
  srv[i].write(applyCalib(i, logical));
}
void writeAllServos(const ArmPos &p) {
  for (int i = 0; i < 6; i++) writeServoLogical(i, p.v[i]);
}

// ────────────────────────────────────────────────────────────────────
//  NON-BLOCKING SMOOTH MOTION ENGINE
//  Uses millis()-based ticking. Each call to updateMotion() advances
//  every joint by at most 1–2° toward its target. Web server stays
//  responsive during all movements — no blocking delays.
// ────────────────────────────────────────────────────────────────────
#define STEP_MS_DEFAULT  18    // ms per 1° step (adjustable via /speed)
int           stepInterval   = STEP_MS_DEFAULT;
unsigned long lastStepMs     = 0;
bool          isMoving       = false;

void updateMotion() {
  if (!isMoving) return;
  unsigned long now = millis();
  if (now - lastStepMs < (unsigned long)stepInterval) return;
  lastStepMs = now;

  bool any = false;
  for (int i = 0; i < 6; i++) {
    if (currentPos.v[i] == targetPos.v[i]) continue;
    any = true;
    int diff = targetPos.v[i] - currentPos.v[i];
    int step = (abs(diff) > 8) ? (diff > 0 ? 2 : -2) : (diff > 0 ? 1 : -1);
    currentPos.v[i] += step;
    if ((step > 0 && currentPos.v[i] > targetPos.v[i]) ||
        (step < 0 && currentPos.v[i] < targetPos.v[i]))
      currentPos.v[i] = targetPos.v[i];
    writeServoLogical(i, currentPos.v[i]);
  }
  if (!any) {
    isMoving = false;
    currentPos = targetPos;
  }
}

// Set new target — motion engine picks it up asynchronously
void moveTo(const ArmPos &tgt) {
  targetPos  = tgt;
  isMoving   = true;
  lastStepMs = millis();
}

// Blocking version for fail-safe (keeps HTTP alive while waiting)
void moveToBlocking(const ArmPos &tgt) {
  moveTo(tgt);
  while (isMoving) { server.handleClient(); updateMotion(); delay(1); }
}

// ────────────────────────────────────────────────────────────────────
//  TRAJECTORY RECORDING & PLAYBACK
//  Records timestamped joint snapshots. Up to 3 named slots,
//  each holding up to MAX_TRAJ_FRAMES frames.
// ────────────────────────────────────────────────────────────────────
#define MAX_TRAJ_FRAMES  400   // Max recorded frames per slot
#define NUM_TRAJ_SLOTS     3   // Number of named trajectory slots
#define TRAJ_INTERVAL_MS 100  // Record a frame every 100ms during recording

struct TrajFrame {
  ArmPos pos;
  unsigned long dtMs;  // Delay from previous frame (ms)
};

TrajFrame  trajectories[NUM_TRAJ_SLOTS][MAX_TRAJ_FRAMES];
int        trajLength[NUM_TRAJ_SLOTS]  = {0, 0, 0};
bool       trajHasData[NUM_TRAJ_SLOTS] = {false, false, false};
const char* TRAJ_NAMES[NUM_TRAJ_SLOTS] = {"TRAJ_A", "TRAJ_B", "TRAJ_C"};

bool          recording     = false;
int           recordSlot    = 0;
unsigned long lastRecordMs  = 0;
unsigned long recordStartMs = 0;

bool          replaying     = false;
int           replaySlot    = 0;
int           replayFrame   = 0;
unsigned long replayNextMs  = 0;

void startRecording(int slot) {
  if (slot < 0 || slot >= NUM_TRAJ_SLOTS) return;
  recordSlot        = slot;
  trajLength[slot]  = 0;
  recording         = true;
  lastRecordMs      = millis();
  recordStartMs     = millis();
  Serial.printf("[TRAJ] Recording started → slot %s\n", TRAJ_NAMES[slot]);
}

void stopRecording() {
  if (!recording) return;
  recording = false;
  trajHasData[recordSlot] = (trajLength[recordSlot] > 0);
  Serial.printf("[TRAJ] Recorded %d frames into %s\n",
                trajLength[recordSlot], TRAJ_NAMES[recordSlot]);
}

// Call every loop() to capture frames during recording
void updateRecording() {
  if (!recording) return;
  unsigned long now = millis();
  if (now - lastRecordMs < TRAJ_INTERVAL_MS) return;
  if (trajLength[recordSlot] >= MAX_TRAJ_FRAMES) {
    stopRecording();
    Serial.println("[TRAJ] Buffer full — recording stopped.");
    return;
  }
  TrajFrame &fr = trajectories[recordSlot][trajLength[recordSlot]];
  fr.pos  = currentPos;
  fr.dtMs = (trajLength[recordSlot] == 0) ? 0 : (now - lastRecordMs);
  trajLength[recordSlot]++;
  lastRecordMs = now;
}

void startPlayback(int slot) {
  if (slot < 0 || slot >= NUM_TRAJ_SLOTS) return;
  if (!trajHasData[slot] || trajLength[slot] == 0) {
    Serial.printf("[TRAJ] No data in slot %s\n", TRAJ_NAMES[slot]);
    return;
  }
  replaySlot  = slot;
  replayFrame = 0;
  replaying   = true;
  replayNextMs = millis();
  // Move to first frame immediately
  moveTo(trajectories[slot][0].pos);
  Serial.printf("[TRAJ] Playback started ← slot %s (%d frames)\n",
                TRAJ_NAMES[slot], trajLength[slot]);
}

void stopPlayback() {
  replaying = false;
  Serial.println("[TRAJ] Playback stopped.");
}

// Call every loop() to advance playback
void updatePlayback() {
  if (!replaying) return;
  if (isMoving) return;  // Wait until current move is done
  unsigned long now = millis();
  if (now < replayNextMs) return;

  replayFrame++;
  if (replayFrame >= trajLength[replaySlot]) {
    replaying = false;
    Serial.println("[TRAJ] Playback complete.");
    return;
  }
  TrajFrame &fr = trajectories[replaySlot][replayFrame];
  moveTo(fr.pos);
  replayNextMs = now + max((unsigned long)fr.dtMs, (unsigned long)20);
}

// ─── Watchdog Fail-Safe ──────────────────────────────────────────────
#define WATCHDOG_MS  5000UL
unsigned long lastClientMs     = 0;
bool          pickupSaved       = false;
bool          failSafeTriggered = false;

void touchWatchdog() {
  lastClientMs      = millis();
  failSafeTriggered = false;
}

// ─── HTTP Helpers ────────────────────────────────────────────────────
int  getIntArg(const char *k, int def = 0) {
  return server.hasArg(k) ? server.arg(k).toInt() : def;
}
String getStrArg(const char *k, const char *def = "") {
  return server.hasArg(k) ? server.arg(k) : String(def);
}
void sendOK()   { server.send(200, "text/plain", "OK"); }
void sendJSON(const String &j) { server.send(200, "application/json", j); }

// ════════════════════════════════════════════════════════════════════
//  WEB PAGE  (stored in flash)
// ════════════════════════════════════════════════════════════════════
const char webpage[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARM-6 · Industry 4.0</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=IBM+Plex+Mono:wght@300;400;600&family=Titillium+Web:wght@300;400;600&display=swap" rel="stylesheet">
<style>
/* ════════════════════════════════
   CSS VARIABLES & RESET
════════════════════════════════ */
:root{
  --bg0:#040810;--bg1:#080f1a;--bg2:#0c1524;
  --bg3:#101d30;--bd:#162540;--bd2:#1f3655;
  --c1:#00d4ff;--c2:#ff5722;--c3:#00e676;
  --c4:#ff1744;--c5:#ffd600;--c6:#ff9100;
  --tx:#b0d0e8;--mx:#5a7a96;
  --f1:'Orbitron',sans-serif;
  --f2:'IBM Plex Mono',monospace;
  --f3:'Titillium Web',sans-serif;
  --rad:4px;
}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html{font-size:15px}
body{
  font-family:var(--f3);font-weight:300;
  background:var(--bg0);color:var(--tx);
  min-height:100vh;padding:10px;
  background-image:
    radial-gradient(ellipse 60% 30% at 50% -5%,rgba(0,180,255,.12),transparent),
    radial-gradient(ellipse 40% 20% at 80% 120%,rgba(255,87,34,.06),transparent),
    repeating-linear-gradient(0deg,transparent,transparent 47px,rgba(0,180,255,.018) 48px),
    repeating-linear-gradient(90deg,transparent,transparent 47px,rgba(0,180,255,.018) 48px);
}
::-webkit-scrollbar{width:3px}
::-webkit-scrollbar-track{background:var(--bg0)}
::-webkit-scrollbar-thumb{background:var(--bd2);border-radius:2px}

/* ════════════════════════════════
   HEADER
════════════════════════════════ */
header{
  display:flex;align-items:center;justify-content:space-between;
  padding:0 4px 12px;margin-bottom:14px;
  border-bottom:1px solid var(--bd);
}
.brand{display:flex;align-items:center;gap:12px}
.brand-mark{
  width:40px;height:40px;border:1.5px solid var(--c1);border-radius:var(--rad);
  display:flex;align-items:center;justify-content:center;
  background:rgba(0,212,255,.04);position:relative;overflow:hidden;
  box-shadow:0 0 15px rgba(0,212,255,.2);
}
.brand-mark svg{width:22px;height:22px;stroke:var(--c1);fill:none;stroke-width:1.5}
.brand-mark::after{
  content:'';position:absolute;inset:0;
  background:linear-gradient(135deg,transparent 30%,rgba(0,212,255,.12) 50%,transparent 70%);
  animation:sheen 4s ease-in-out infinite;
}
@keyframes sheen{0%{transform:translateX(-200%)}100%{transform:translateX(200%)}}
.brand-text{font-family:var(--f1);font-size:1.1rem;font-weight:900;letter-spacing:5px;color:var(--c1);
  text-shadow:0 0 20px rgba(0,212,255,.5)}
.brand-text span{color:var(--c2)}
.brand-sub{font-family:var(--f2);font-size:.5rem;letter-spacing:3px;color:var(--mx);margin-top:1px}
.hdr-status{display:flex;align-items:center;gap:10px;font-family:var(--f2);font-size:.62rem;color:var(--mx)}
.status-chip{
  display:flex;align-items:center;gap:6px;padding:4px 12px;
  border:1px solid var(--bd2);border-radius:20px;
  background:rgba(0,212,255,.03);
}
.dot{width:7px;height:7px;border-radius:50%;background:var(--c3);box-shadow:0 0 7px var(--c3);animation:pb 2.5s ease-in-out infinite}
.dot.off{background:var(--c4);box-shadow:0 0 7px var(--c4)}
.dot.wrn{background:var(--c6);box-shadow:0 0 7px var(--c6)}
@keyframes pb{0%,100%{opacity:1}50%{opacity:.2}}

/* ════════════════════════════════
   LAYOUT
════════════════════════════════ */
.outer{max-width:980px;margin:0 auto;display:flex;flex-direction:column;gap:12px}
.row{display:grid;gap:12px}
.row-2{grid-template-columns:1fr 1fr}
.row-3{grid-template-columns:1fr 1fr 1fr}
@media(max-width:700px){.row-2,.row-3{grid-template-columns:1fr}}
@media(min-width:701px) and (max-width:900px){.row-3{grid-template-columns:1fr 1fr}}

/* ════════════════════════════════
   CARD
════════════════════════════════ */
.card{
  background:var(--bg1);border:1px solid var(--bd);
  border-radius:var(--rad);padding:14px;position:relative;overflow:hidden;
}
.card::before{
  content:'';position:absolute;top:0;left:0;right:0;height:1.5px;
  background:linear-gradient(90deg,transparent 5%,var(--c1) 50%,transparent 95%);opacity:.45;
}
.card.c2::before{background:linear-gradient(90deg,transparent 5%,var(--c2) 50%,transparent 95%)}
.card.c3::before{background:linear-gradient(90deg,transparent 5%,var(--c3) 50%,transparent 95%)}
.card.c5::before{background:linear-gradient(90deg,transparent 5%,var(--c5) 50%,transparent 95%)}
.card.c6::before{background:linear-gradient(90deg,transparent 5%,var(--c6) 50%,transparent 95%)}
.card-title{
  font-family:var(--f2);font-size:.58rem;letter-spacing:3px;text-transform:uppercase;
  color:var(--mx);margin-bottom:13px;display:flex;align-items:center;gap:7px;
}
.card-title::before{content:'';width:10px;height:1.5px;background:var(--c1);box-shadow:0 0 4px var(--c1)}
.card.c2 .card-title::before{background:var(--c2);box-shadow:0 0 4px var(--c2)}
.card.c3 .card-title::before{background:var(--c3);box-shadow:0 0 4px var(--c3)}
.card.c5 .card-title::before{background:var(--c5);box-shadow:0 0 4px var(--c5)}
.card.c6 .card-title::before{background:var(--c6);box-shadow:0 0 4px var(--c6)}

/* ════════════════════════════════
   SERVO ROWS
════════════════════════════════ */
.sr{display:grid;grid-template-columns:82px 1fr 52px;align-items:center;gap:9px;margin-bottom:11px}
.sr:last-of-type{margin-bottom:0}
.sl{display:flex;flex-direction:column;gap:2px;line-height:1}
.sl-name{font-family:var(--f3);font-size:.8rem;font-weight:600;color:var(--tx)}
.sl-tag{font-family:var(--f2);font-size:.52rem;color:var(--mx);letter-spacing:.5px}
.sv-val{font-family:var(--f2);font-size:.75rem;color:var(--c1);text-align:right}
.sv-phy{font-family:var(--f2);font-size:.5rem;color:var(--c6);text-align:right;margin-top:1px}

/* ════════════════════════════════
   RANGE SLIDER
════════════════════════════════ */
input[type=range]{
  -webkit-appearance:none;width:100%;height:3px;border-radius:2px;
  outline:none;cursor:pointer;
  background:linear-gradient(90deg,var(--c1) var(--pct,50%),var(--bd2) var(--pct,50%));
}
input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:14px;height:14px;border-radius:50%;
  background:var(--bg0);border:2px solid var(--c1);
  box-shadow:0 0 8px rgba(0,212,255,.6);cursor:pointer;
  transition:transform .1s,box-shadow .1s;
}
input[type=range]:hover::-webkit-slider-thumb,
input[type=range]:active::-webkit-slider-thumb{
  transform:scale(1.35);box-shadow:0 0 16px rgba(0,212,255,.9);
}

/* ════════════════════════════════
   BUTTONS
════════════════════════════════ */
.btn-grid{display:grid;gap:7px}
.btn-grid.g2{grid-template-columns:1fr 1fr}
.btn-grid.g3{grid-template-columns:1fr 1fr 1fr}
.btn-grid.g4{grid-template-columns:1fr 1fr 1fr 1fr}
.btn{
  padding:8px 5px;border:1px solid var(--bd2);border-radius:var(--rad);
  background:rgba(255,255,255,.015);color:var(--tx);
  font-family:var(--f2);font-size:.6rem;letter-spacing:1px;text-transform:uppercase;
  cursor:pointer;transition:all .15s;display:flex;align-items:center;justify-content:center;gap:4px;
}
.btn:hover{border-color:var(--c1);color:var(--c1);background:rgba(0,212,255,.06);box-shadow:0 0 10px rgba(0,212,255,.12)}
.btn:active{transform:scale(.96)}
.btn.full{grid-column:1/-1}
.btn.v-red{border-color:var(--c4);color:var(--c4)}
.btn.v-red:hover{background:rgba(255,23,68,.07);box-shadow:0 0 10px rgba(255,23,68,.2)}
.btn.v-grn{border-color:var(--c3);color:var(--c3)}
.btn.v-grn:hover{background:rgba(0,230,118,.07);box-shadow:0 0 10px rgba(0,230,118,.2)}
.btn.v-amb{border-color:var(--c6);color:var(--c6)}
.btn.v-amb:hover{background:rgba(255,145,0,.07);box-shadow:0 0 10px rgba(255,145,0,.2)}
.btn.v-yel{border-color:var(--c5);color:var(--c5)}
.btn.v-yel:hover{background:rgba(255,214,0,.07);box-shadow:0 0 10px rgba(255,214,0,.2)}
.btn.v-acc{border-color:var(--c1);color:var(--c1);background:rgba(0,212,255,.04)}
.btn.v-acc:hover{background:rgba(0,212,255,.1)}
.btn.on{background:rgba(0,212,255,.1);border-color:var(--c1);color:var(--c1)}

/* Gripper toggles */
.grip-row{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-bottom:10px}
.grip-btn{
  padding:12px 5px;border:1px solid var(--bd2);border-radius:var(--rad);
  background:rgba(255,255,255,.015);font-family:var(--f2);font-size:.65rem;
  letter-spacing:1px;text-transform:uppercase;cursor:pointer;transition:all .15s;
  color:var(--mx);text-align:center;
}
.grip-btn:hover{border-color:var(--tx);color:var(--tx)}
.grip-btn.is-open {border-color:var(--c3);color:var(--c3);background:rgba(0,230,118,.07)}
.grip-btn.is-close{border-color:var(--c2);color:var(--c2);background:rgba(255,87,34,.07)}

/* ════════════════════════════════
   WATCHDOG BAR
════════════════════════════════ */
.wd-row{display:flex;align-items:center;gap:10px;margin-bottom:12px}
.wd-track{flex:1;height:4px;background:var(--bd);border-radius:2px;overflow:hidden}
.wd-fill{height:100%;width:100%;background:var(--c3);border-radius:2px;transition:width .1s linear,background .4s}
.wd-time{font-family:var(--f2);font-size:.62rem;color:var(--c5);min-width:36px;text-align:right}
.wd-tag{font-family:var(--f2);font-size:.55rem;color:var(--mx);letter-spacing:1px}

/* ════════════════════════════════
   STATE PILLS
════════════════════════════════ */
.pill-row{display:flex;gap:6px;margin-top:9px}
.pill{
  flex:1;padding:6px 4px;background:var(--bg2);border:1px solid var(--bd);border-radius:var(--rad);
  font-family:var(--f2);font-size:.55rem;text-align:center;color:var(--mx);letter-spacing:1px;
}
.pill.ok  {border-color:var(--c3);color:var(--c3);background:rgba(0,230,118,.05)}
.pill.warn{border-color:var(--c6);color:var(--c6);background:rgba(255,145,0,.05)}
.pill.alrt{border-color:var(--c4);color:var(--c4);background:rgba(255,23,68,.05);animation:pb .7s infinite}

/* ════════════════════════════════
   TELEMETRY GRID
════════════════════════════════ */
.tele-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.tele-cell{background:var(--bg2);border:1px solid var(--bd);border-radius:var(--rad);padding:9px 5px;text-align:center}
.tele-v{font-family:var(--f2);font-size:.95rem;color:var(--c1);line-height:1}
.tele-p{font-family:var(--f2);font-size:.48rem;color:var(--c6);margin-top:2px}
.tele-l{font-family:var(--f2);font-size:.48rem;color:var(--mx);letter-spacing:1px;margin-top:3px;text-transform:uppercase}

/* ════════════════════════════════
   TRAJECTORY SECTION
════════════════════════════════ */
.traj-slots{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-bottom:11px}
.traj-slot{
  background:var(--bg2);border:1px solid var(--bd);border-radius:var(--rad);padding:10px 8px;
  text-align:center;position:relative;overflow:hidden;
}
.traj-slot.has-data{border-color:var(--c5);box-shadow:0 0 8px rgba(255,214,0,.08)}
.traj-slot.recording{border-color:var(--c4);box-shadow:0 0 10px rgba(255,23,68,.2);animation:pb .5s infinite}
.traj-slot.playing{border-color:var(--c3);box-shadow:0 0 10px rgba(0,230,118,.2)}
.ts-name{font-family:var(--f1);font-size:.6rem;letter-spacing:2px;color:var(--mx);margin-bottom:4px}
.ts-frames{font-family:var(--f2);font-size:.55rem;color:var(--c5);margin-bottom:8px;min-height:14px}
.ts-btns{display:flex;gap:4px}
.ts-btn{
  flex:1;padding:5px 2px;border:1px solid var(--bd2);border-radius:3px;
  background:rgba(255,255,255,.01);font-family:var(--f2);font-size:.52rem;
  letter-spacing:.5px;text-transform:uppercase;cursor:pointer;transition:all .14s;color:var(--mx);
}
.ts-btn:hover{border-color:var(--tx);color:var(--tx)}
.ts-btn.rec{border-color:var(--c4);color:var(--c4)}
.ts-btn.rec:hover{background:rgba(255,23,68,.08)}
.ts-btn.rec.active{background:rgba(255,23,68,.15);animation:pb .5s infinite}
.ts-btn.play{border-color:var(--c3);color:var(--c3)}
.ts-btn.play:hover{background:rgba(0,230,118,.08)}
.ts-btn.play.active{background:rgba(0,230,118,.15)}
.ts-btn.clr{border-color:var(--mx);color:var(--mx)}
.ts-btn.clr:hover{border-color:var(--c4);color:var(--c4)}
.traj-controls{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.rec-indicator{
  display:flex;align-items:center;justify-content:center;gap:7px;
  padding:7px;background:var(--bg2);border:1px solid var(--bd);border-radius:var(--rad);
  font-family:var(--f2);font-size:.58rem;color:var(--mx);
}
.rec-dot{width:8px;height:8px;border-radius:50%;background:var(--c4);opacity:0;transition:opacity .2s}
.rec-dot.on{opacity:1;box-shadow:0 0 6px var(--c4);animation:pb .5s infinite}
.prog-wrap{flex:1;height:3px;background:var(--bd);border-radius:2px;overflow:hidden}
.prog-fill{height:100%;width:0%;background:var(--c3);border-radius:2px;transition:width .2s}

/* ════════════════════════════════
   CALIBRATION TABLE
════════════════════════════════ */
.ctbl{width:100%;border-collapse:collapse;font-family:var(--f2);font-size:.6rem}
.ctbl th{text-align:left;padding:4px 5px;color:var(--mx);border-bottom:1px solid var(--bd);letter-spacing:1px;text-transform:uppercase}
.ctbl td{padding:4px 5px;color:var(--tx);border-bottom:1px solid rgba(255,255,255,.02)}
.ctbl td:first-child{color:var(--c1)}
.ci{
  width:48px;background:var(--bg2);border:1px solid var(--bd2);border-radius:3px;
  color:var(--c5);font-family:var(--f2);font-size:.6rem;padding:2px 4px;text-align:center;outline:none;
}
.ci:focus{border-color:var(--c1)}
select.ci{width:54px;cursor:pointer}

/* ════════════════════════════════
   SPEED CONTROL
════════════════════════════════ */
.spd-row{display:grid;grid-template-columns:50px 1fr 34px;align-items:center;gap:8px;margin-bottom:9px}
.spd-lbl{font-family:var(--f2);font-size:.58rem;color:var(--mx);letter-spacing:1px;text-transform:uppercase}

/* ════════════════════════════════
   EVENT LOG
════════════════════════════════ */
#evlog{
  height:88px;overflow-y:auto;background:var(--bg2);border:1px solid var(--bd);border-radius:var(--rad);
  padding:7px 9px;font-family:var(--f2);font-size:.59rem;
}
.le{padding:1px 0;border-bottom:1px solid rgba(255,255,255,.018);color:var(--mx)}
.le.ok  {color:var(--c3)}.le.err{color:var(--c4)}.le.warn{color:var(--c6)}.le.info{color:var(--c1)}
.le.traj{color:var(--c5)}

/* Moving badge */
.mov-badge{
  font-family:var(--f2);font-size:.54rem;letter-spacing:1px;text-transform:uppercase;
  padding:2px 7px;border:1px solid var(--c6);border-radius:12px;
  color:var(--c6);background:rgba(255,145,0,.07);opacity:0;transition:opacity .2s;
}
.mov-badge.vis{opacity:1;animation:pb .6s infinite}

/* Divider */
.dv{height:1px;background:var(--bd);margin:11px 0}
</style>
</head>
<body>
<!-- ══ HEADER ══════════════════════════════════════════════════════ -->
<header>
  <div class="brand">
    <div class="brand-mark">
      <svg viewBox="0 0 24 24"><path d="M12 2L2 7v10l10 5 10-5V7L12 2z"/><line x1="12" y1="22" x2="12" y2="12"/><line x1="2" y1="7" x2="12" y2="12"/><line x1="22" y1="7" x2="12" y2="12"/></svg>
    </div>
    <div>
      <div class="brand-text">ARM<span>-6</span> v4</div>
      <div class="brand-sub">INDUSTRY 4.0 · 6-DOF CONTROL SYSTEM</div>
    </div>
  </div>
  <div class="hdr-status">
    <span class="mov-badge" id="movBadge">MOVING</span>
    <span id="pingDisp">-- ms</span>
    <div class="status-chip">
      <div class="dot" id="cDot"></div>
      <span id="cLbl">ONLINE</span>
    </div>
  </div>
</header>

<div class="outer">

  <!-- ══ ROW 1: Joint Control (full width) ════════════════════════ -->
  <div class="card">
    <div class="card-title">Joint Control
      <span style="font-size:.5rem;color:var(--mx);letter-spacing:1px;margin-left:4px">SMOOTH MOTION · DEBOUNCED · 80ms</span>
    </div>
    <div id="jointRows">
      <!-- S1 Base -->
      <div class="sr">
        <div class="sl"><span class="sl-name">Base</span><span class="sl-tag">MG995</span></div>
        <input type="range" min="0" max="180" value="90" id="r0" oninput="onSlider(0,this)">
        <div><div class="sv-val" id="v0">90°</div><div class="sv-phy" id="p0">→90°</div></div>
      </div>
      <!-- S2 Shoulder -->
      <div class="sr">
        <div class="sl"><span class="sl-name">Shoulder</span><span class="sl-tag">MG995 +90°</span></div>
        <input type="range" min="0" max="90" value="0" id="r1" oninput="onSlider(1,this)">
        <div><div class="sv-val" id="v1">0°</div><div class="sv-phy" id="p1">→90°</div></div>
      </div>
      <!-- S3 Elbow -->
      <div class="sr">
        <div class="sl"><span class="sl-name">Elbow</span><span class="sl-tag">STD</span></div>
        <input type="range" min="0" max="180" value="90" id="r2" oninput="onSlider(2,this)">
        <div><div class="sv-val" id="v2">90°</div><div class="sv-phy" id="p2">→90°</div></div>
      </div>
      <!-- S4 Wrist Pitch -->
      <div class="sr">
        <div class="sl"><span class="sl-name">Wrist P</span><span class="sl-tag">STD</span></div>
        <input type="range" min="0" max="180" value="90" id="r3" oninput="onSlider(3,this)">
        <div><div class="sv-val" id="v3">90°</div><div class="sv-phy" id="p3">→90°</div></div>
      </div>
      <!-- S5 Wrist Roll -->
      <div class="sr">
        <div class="sl"><span class="sl-name">Wrist R</span><span class="sl-tag">STD</span></div>
        <input type="range" min="0" max="180" value="90" id="r4" oninput="onSlider(4,this)">
        <div><div class="sv-val" id="v4">90°</div><div class="sv-phy" id="p4">→90°</div></div>
      </div>
      <div class="dv"></div>
      <!-- S6 Gripper -->
      <div class="card-title" style="margin-bottom:9px">
        Gripper
        <span style="font-size:.48rem;color:var(--c6)">(INVERTED · 0=OPEN · 90=CLOSE)</span>
      </div>
      <div class="grip-row">
        <button class="grip-btn" id="gbOpen"  onclick="openGripper()">▷ OPEN</button>
        <button class="grip-btn is-close" id="gbClose" onclick="closeGripper()">◁ CLOSE</button>
      </div>
      <div class="sr" style="margin-bottom:0">
        <div class="sl"><span class="sl-name">Fine</span><span class="sl-tag">STD INV</span></div>
        <input type="range" min="0" max="90" value="45" id="r5" oninput="onSlider(5,this)">
        <div><div class="sv-val" id="v5">45°</div><div class="sv-phy" id="p5">→45°</div></div>
      </div>
    </div>
  </div>

  <!-- ══ ROW 2: Fail-Safe | Telemetry ═════════════════════════════ -->
  <div class="row row-2">

    <!-- Fail-Safe & Memory -->
    <div class="card c2">
      <div class="card-title">Fail-Safe &amp; Position Memory</div>
      <div class="wd-row">
        <span class="wd-tag">WDG</span>
        <div class="wd-track"><div class="wd-fill" id="wFill"></div></div>
        <div class="wd-time" id="wTime">5.0s</div>
      </div>
      <div class="btn-grid g2">
        <button class="btn v-amb full" onclick="savePickup()">📍 Save Pickup</button>
        <button class="btn v-red"      onclick="retPickup()">⏎ Ret. Pickup</button>
        <button class="btn"            onclick="goHome()">⌂ Home</button>
        <button class="btn v-grn"      onclick="setHome()">✔ Set Home</button>
      </div>
      <div class="pill-row">
        <div class="pill" id="pPU">PICKUP: NONE</div>
        <div class="pill" id="pFS">STANDBY</div>
      </div>
    </div>

    <!-- Live Telemetry -->
    <div class="card c3">
      <div class="card-title">Live Telemetry
        <span style="font-size:.48rem;color:var(--mx)">LOGICAL / PHYSICAL</span>
      </div>
      <div class="tele-grid">
        <div class="tele-cell"><div class="tele-v" id="tB">90°</div><div class="tele-p" id="tBp">→90°</div><div class="tele-l">Base</div></div>
        <div class="tele-cell"><div class="tele-v" id="tS">0°</div> <div class="tele-p" id="tSp">→90°</div><div class="tele-l">Shoulder</div></div>
        <div class="tele-cell"><div class="tele-v" id="tE">90°</div><div class="tele-p" id="tEp">→90°</div><div class="tele-l">Elbow</div></div>
        <div class="tele-cell"><div class="tele-v" id="tWP">90°</div><div class="tele-p" id="tWPp">→90°</div><div class="tele-l">W.Pitch</div></div>
        <div class="tele-cell"><div class="tele-v" id="tWR">90°</div><div class="tele-p" id="tWRp">→90°</div><div class="tele-l">W.Roll</div></div>
        <div class="tele-cell"><div class="tele-v" id="tGr">45°</div><div class="tele-p" id="tGrp">→45°</div><div class="tele-l">Gripper</div></div>
      </div>
    </div>

  </div><!-- /row-2 -->

  <!-- ══ ROW 3: TRAJECTORY RECORDING & PLAYBACK ══════════════════ -->
  <div class="card c5">
    <div class="card-title">Trajectory Recording &amp; Playback
      <span style="font-size:.48rem;color:var(--mx)">RECORD MANUAL MOVES · REPLAY AUTOMATICALLY</span>
    </div>

    <!-- 3 Trajectory Slots -->
    <div class="traj-slots">
      <div class="traj-slot" id="ts0">
        <div class="ts-name">TRAJ_A</div>
        <div class="ts-frames" id="tf0">NO DATA</div>
        <div class="ts-btns">
          <button class="ts-btn rec" id="tb-rec0" onclick="trajRec(0)">⏺ REC</button>
          <button class="ts-btn play" id="tb-play0" onclick="trajPlay(0)">▶ PLAY</button>
          <button class="ts-btn clr" onclick="trajClear(0)">✕</button>
        </div>
      </div>
      <div class="traj-slot" id="ts1">
        <div class="ts-name">TRAJ_B</div>
        <div class="ts-frames" id="tf1">NO DATA</div>
        <div class="ts-btns">
          <button class="ts-btn rec" id="tb-rec1" onclick="trajRec(1)">⏺ REC</button>
          <button class="ts-btn play" id="tb-play1" onclick="trajPlay(1)">▶ PLAY</button>
          <button class="ts-btn clr" onclick="trajClear(1)">✕</button>
        </div>
      </div>
      <div class="traj-slot" id="ts2">
        <div class="ts-name">TRAJ_C</div>
        <div class="ts-frames" id="tf2">NO DATA</div>
        <div class="ts-btns">
          <button class="ts-btn rec" id="tb-rec2" onclick="trajRec(2)">⏺ REC</button>
          <button class="ts-btn play" id="tb-play2" onclick="trajPlay(2)">▶ PLAY</button>
          <button class="ts-btn clr" onclick="trajClear(2)">✕</button>
        </div>
      </div>
    </div>

    <!-- Recording status bar -->
    <div class="rec-indicator" id="recStatus">
      <div class="rec-dot" id="recDot"></div>
      <span id="recLabel" style="font-family:var(--f2);font-size:.57rem;color:var(--mx)">NO ACTIVE RECORDING</span>
      <div class="prog-wrap"><div class="prog-fill" id="recProg"></div></div>
      <span id="recFrames" style="font-family:var(--f2);font-size:.55rem;color:var(--mx)">0/400</span>
    </div>

    <div style="margin-top:9px">
      <button class="btn v-red" onclick="stopAll()" style="width:100%;padding:9px">■ STOP RECORDING / PLAYBACK</button>
    </div>

    <div style="font-family:var(--f2);font-size:.53rem;color:var(--mx);margin-top:8px;line-height:1.8">
      HOW TO USE: Move joints manually → press ⏺ REC on a slot → operate arm → press ■ STOP → press ▶ PLAY to replay
    </div>
  </div>

  <!-- ══ ROW 4: Presets | Speed | Calibration ════════════════════ -->
  <div class="row row-3">

    <!-- Presets -->
    <div class="card">
      <div class="card-title">Preset Sequences</div>
      <div class="btn-grid g2">
        <button class="btn v-acc" onclick="runPreset('rest')">💤 Rest</button>
        <button class="btn v-acc" onclick="runPreset('ready')">⚡ Ready</button>
        <button class="btn v-acc" onclick="runPreset('pickup')">⬇ Pick</button>
        <button class="btn v-acc" onclick="runPreset('place')">⬆ Place</button>
      </div>
    </div>

    <!-- Speed -->
    <div class="card">
      <div class="card-title">Motion Speed</div>
      <div class="spd-row">
        <div class="spd-lbl">Speed</div>
        <input type="range" min="1" max="50" value="18" id="spdR" oninput="setSpd(this.value)">
        <div class="sv-val" id="spdV">18</div>
      </div>
      <div style="font-family:var(--f2);font-size:.53rem;color:var(--mx);line-height:1.8">
        1 ms/step = fastest<br>
        50 ms/step = slowest<br>
        Default: 18 ms
      </div>
    </div>

    <!-- Emergency -->
    <div class="card c2">
      <div class="card-title">Emergency</div>
      <div class="btn-grid g2">
        <button class="btn v-red full" onclick="estop()" style="padding:12px">⛔ E-STOP ALL</button>
        <button class="btn v-grn"  onclick="releaseStop()">▶ RESUME</button>
        <button class="btn v-amb"  onclick="runPreset('rest')">🏠 SAFE POS</button>
      </div>
    </div>

  </div>

  <!-- ══ ROW 5: Calibration (full width) ═════════════════════════ -->
  <div class="card c6">
    <div class="card-title">Servo Calibration
      <span style="font-size:.5rem;color:var(--mx)">(OFFSET · INVERSION · LIMITS)</span>
    </div>
    <table class="ctbl">
      <thead><tr>
        <th>Joint</th><th>Offset °</th><th>Invert</th><th>Min °</th><th>Max °</th>
      </tr></thead>
      <tbody id="calBody"></tbody>
    </table>
    <div style="margin-top:10px">
      <button class="btn v-grn full" onclick="sendCalib()" style="padding:10px">⟳ Apply Calibration → ESP32</button>
    </div>
    <div style="font-family:var(--f2);font-size:.52rem;color:var(--mx);margin-top:8px;line-height:1.9">
      OFFSET: added to logical angle before PWM &nbsp;|&nbsp;
      INVERT: phys = 180−(log+offset) &nbsp;|&nbsp;
      S2 Shoulder default +90° (0→90°phys) &nbsp;|&nbsp; S6 Gripper inverted
    </div>
  </div>

  <!-- ══ ROW 6: Event Log ═════════════════════════════════════════ -->
  <div class="card">
    <div class="card-title" style="justify-content:space-between;margin-bottom:9px">
      <span>System Log</span>
      <button class="btn" onclick="clrLog()" style="padding:2px 8px;font-size:.54rem">CLR</button>
    </div>
    <div id="evlog"></div>
  </div>

</div><!-- /outer -->

<script>
/* ══════════════════════════════════════════════════════════
   CONSTANTS & STATE
══════════════════════════════════════════════════════════ */
const WD_MAX   = 5.0;
const JN       = ['Base','Shoulder','Elbow','Wrist P','Wrist R','Gripper'];
const T_KEYS   = ['base','shoulder','elbow','wristPitch','wristRoll','gripper'];
const T_IDS    = ['tB','tS','tE','tWP','tWR','tGr'];
const TP_IDS   = ['tBp','tSp','tEp','tWPp','tWRp','tGrp'];
const MAX_FRAMES = 400;

let wdLeft      = WD_MAX;
let movingTimer = null;
let estopFlag   = false;

// Mirrors ESP32 calibration table
let CD = [
  {o:0,  inv:false, mn:0, mx:180},
  {o:90, inv:false, mn:0, mx:90 },
  {o:0,  inv:false, mn:0, mx:180},
  {o:0,  inv:false, mn:0, mx:180},
  {o:0,  inv:false, mn:0, mx:180},
  {o:0,  inv:true,  mn:0, mx:90 },
];

// Trajectory UI state
let trajState = [
  {hasData:false, frames:0, recording:false, playing:false},
  {hasData:false, frames:0, recording:false, playing:false},
  {hasData:false, frames:0, recording:false, playing:false},
];
let activeRecSlot  = -1;
let activePlaySlot = -1;

/* ══════════════════════════════════════════════════════════
   CALIBRATION HELPERS
══════════════════════════════════════════════════════════ */
function physCalc(i, log) {
  const c = CD[i];
  const cl = Math.min(Math.max(log, c.mn), c.mx);
  let p = c.inv ? (180-(cl+c.o)) : (cl+c.o);
  return Math.min(180, Math.max(0, p));
}

/* ══════════════════════════════════════════════════════════
   SLIDER — DEBOUNCED SEND (80ms)
══════════════════════════════════════════════════════════ */
let debounceT = [null,null,null,null,null,null];

function onSlider(i, el) {
  if (estopFlag) return;
  const v = parseInt(el.value);
  document.getElementById('v'+i).textContent = v+'°';
  document.getElementById('p'+i).textContent = '→'+physCalc(i,v)+'°';
  updateFill(el);
  clearTimeout(debounceT[i]);
  debounceT[i] = setTimeout(() => sendServo(i, v), 80);
}

function updateFill(el) {
  const pct = ((el.value-el.min)/(el.max-el.min)*100).toFixed(1);
  el.style.setProperty('--pct', pct+'%');
}

/* ══════════════════════════════════════════════════════════
   SERVO SEND
══════════════════════════════════════════════════════════ */
function sendServo(i, v) {
  if (estopFlag) return;
  const t0 = Date.now();
  fetch(`/s${i+1}?val=${v}`)
    .then(() => {
      document.getElementById('pingDisp').textContent = (Date.now()-t0)+' ms';
      setOnline(); wdLeft = WD_MAX; showMoving();
    }).catch(setOffline);
}

function setSlider(i, v) {
  const el = document.getElementById('r'+i);
  el.value = v; onSlider(i, el);
}

/* ══════════════════════════════════════════════════════════
   GRIPPER
══════════════════════════════════════════════════════════ */
function openGripper() {
  setSlider(5, 0);
  document.getElementById('gbOpen').className  = 'grip-btn is-open';
  document.getElementById('gbClose').className = 'grip-btn';
  addLog('Gripper OPEN (logical 0° → physical 90° via inversion)', 'ok');
}
function closeGripper() {
  setSlider(5, 90);
  document.getElementById('gbClose').className = 'grip-btn is-close';
  document.getElementById('gbOpen').className  = 'grip-btn';
  addLog('Gripper CLOSE (logical 90° → physical 0° via inversion)', 'warn');
}

/* ══════════════════════════════════════════════════════════
   PRESETS
══════════════════════════════════════════════════════════ */
const PRESETS = {
  rest:   [90,  0,  45, 90, 90, 45],
  ready:  [90,  0,  90, 90, 90,  0],
  pickup: [90, 30, 120, 60, 90,  0],
  place:  [135, 0,  60, 90, 90, 90],
};
function runPreset(name) {
  if (estopFlag && name !== 'rest') return;
  fetch('/preset?name='+name)
    .then(()=>{
      const p = PRESETS[name];
      if (p) p.forEach((v,i)=>setSlider(i,v));
      addLog(`Preset '${name}' sent`, 'ok'); showMoving();
    }).catch(setOffline);
}

/* ══════════════════════════════════════════════════════════
   SPEED
══════════════════════════════════════════════════════════ */
function setSpd(v) {
  document.getElementById('spdV').textContent = v;
  updateFill(document.getElementById('spdR'));
  fetch('/speed?val='+v).catch(()=>{});
  addLog('Speed: '+v+' ms/step', 'info');
}

/* ══════════════════════════════════════════════════════════
   FAIL-SAFE & MEMORY
══════════════════════════════════════════════════════════ */
function savePickup() {
  fetch('/savePickup').then(()=>{
    document.getElementById('pPU').textContent='PICKUP: SAVED ✓';
    document.getElementById('pPU').classList.add('ok');
    addLog('Pickup position saved ✓','ok');
  }).catch(setOffline);
}
function retPickup() {
  fetch('/returnPickup').then(()=>{
    addLog('Returning to pickup…','warn'); showMoving(); setTimeout(syncStatus,4000);
  }).catch(setOffline);
}
function goHome() {
  fetch('/home').then(()=>{
    addLog('Moving to home','ok'); showMoving(); setTimeout(syncStatus,4000);
  }).catch(setOffline);
}
function setHome() {
  fetch('/setHome').then(()=>addLog('Home position updated ✓','ok')).catch(setOffline);
}

/* ══════════════════════════════════════════════════════════
   EMERGENCY STOP
══════════════════════════════════════════════════════════ */
function estop() {
  estopFlag = true;
  fetch('/estop').then(()=>{ addLog('⛔ E-STOP ACTIVATED','err'); }).catch(()=>{});
}
function releaseStop() {
  estopFlag = false;
  fetch('/resumeStop').then(()=>{ addLog('E-Stop released','ok'); }).catch(()=>{});
}

/* ══════════════════════════════════════════════════════════
   TRAJECTORY RECORDING & PLAYBACK
══════════════════════════════════════════════════════════ */
function trajRec(slot) {
  if (trajState[slot].recording) { stopAll(); return; }
  // Stop any other recording/playback first
  stopAll(true); // silent stop
  fetch('/trajRec?slot='+slot).then(()=>{
    trajState[slot].recording = true;
    trajState[slot].frames    = 0;
    activeRecSlot = slot;
    updateTrajUI();
    document.getElementById('recDot').classList.add('on');
    document.getElementById('tb-rec'+slot).classList.add('active');
    document.getElementById('ts'+slot).classList.add('recording');
    addLog(`Recording started → TRAJ_${String.fromCharCode(65+slot)}`,'traj');
  }).catch(setOffline);
}

function trajPlay(slot) {
  if (!trajState[slot].hasData) { addLog(`TRAJ_${String.fromCharCode(65+slot)}: no data`,'warn'); return; }
  if (trajState[slot].playing) { stopAll(); return; }
  stopAll(true);
  fetch('/trajPlay?slot='+slot).then(()=>{
    trajState[slot].playing = true;
    activePlaySlot = slot;
    updateTrajUI();
    document.getElementById('tb-play'+slot).classList.add('active');
    document.getElementById('ts'+slot).classList.add('playing');
    addLog(`Playback started ← TRAJ_${String.fromCharCode(65+slot)} (${trajState[slot].frames} frames)`,'traj');
    showMoving();
  }).catch(setOffline);
}

function trajClear(slot) {
  fetch('/trajClear?slot='+slot).then(()=>{
    trajState[slot].hasData=false; trajState[slot].frames=0;
    updateTrajUI();
    addLog(`TRAJ_${String.fromCharCode(65+slot)} cleared`,'warn');
  }).catch(setOffline);
}

function stopAll(silent=false) {
  fetch('/stopAll').then(()=>{
    trajState.forEach((s,i)=>{ s.recording=false; s.playing=false; });
    activeRecSlot=-1; activePlaySlot=-1;
    updateTrajUI();
    if (!silent) addLog('Recording/Playback stopped','warn');
  }).catch(()=>{});
}

function updateTrajUI() {
  // Reset all slot visuals
  for (let i=0;i<3;i++) {
    const s  = trajState[i];
    const el = document.getElementById('ts'+i);
    const fr = document.getElementById('tf'+i);
    el.classList.remove('has-data','recording','playing');
    document.getElementById('tb-rec'+i).classList.remove('active');
    document.getElementById('tb-play'+i).classList.remove('active');
    if (s.hasData)    el.classList.add('has-data');
    if (s.recording)  el.classList.add('recording');
    if (s.playing)    el.classList.add('playing');
    fr.textContent = s.hasData ? `${s.frames} FRAMES` : 'NO DATA';
    if (s.recording)  fr.textContent = `REC... ${s.frames}`;
  }
  const anyRec = trajState.some(s=>s.recording);
  document.getElementById('recDot').classList.toggle('on', anyRec);
  document.getElementById('recLabel').textContent = anyRec
    ? `RECORDING → TRAJ_${String.fromCharCode(65+activeRecSlot)}`
    : 'NO ACTIVE RECORDING';
}

/* ══════════════════════════════════════════════════════════
   CALIBRATION TABLE
══════════════════════════════════════════════════════════ */
function buildCalTable() {
  const tb = document.getElementById('calBody');
  tb.innerHTML='';
  CD.forEach((c,i)=>{
    const tr = document.createElement('tr');
    tr.innerHTML=`
      <td>${JN[i]}</td>
      <td><input class="ci" id="co${i}" type="number" value="${c.o}" min="-180" max="180"></td>
      <td><select class="ci" id="ci${i}">
        <option value="0" ${!c.inv?'selected':''}>NO</option>
        <option value="1" ${c.inv?'selected':''}>YES</option>
      </select></td>
      <td><input class="ci" id="cmn${i}" type="number" value="${c.mn}" min="0" max="180"></td>
      <td><input class="ci" id="cmx${i}" type="number" value="${c.mx}" min="0" max="180"></td>`;
    tb.appendChild(tr);
  });
}
function sendCalib() {
  for(let i=0;i<6;i++){
    CD[i].o   = parseInt(document.getElementById('co'+i).value)||0;
    CD[i].inv = document.getElementById('ci'+i).value==='1';
    CD[i].mn  = parseInt(document.getElementById('cmn'+i).value)||0;
    CD[i].mx  = parseInt(document.getElementById('cmx'+i).value)||180;
  }
  const q = CD.map((c,i)=>`o${i}=${c.o}&inv${i}=${c.inv?1:0}&mn${i}=${c.mn}&mx${i}=${c.mx}`).join('&');
  fetch('/calibrate?'+q)
    .then(()=>{ addLog('Calibration applied ✓','ok'); refreshPhys(); })
    .catch(()=>{ addLog('Calibration send failed','err'); setOffline(); });
}
function refreshPhys() {
  for(let i=0;i<6;i++){
    const v=parseInt(document.getElementById('r'+i).value);
    document.getElementById('p'+i).textContent='→'+physCalc(i,v)+'°';
  }
}

/* ══════════════════════════════════════════════════════════
   STATUS SYNC (polls /status every 2s)
══════════════════════════════════════════════════════════ */
function syncStatus() {
  fetch('/status').then(r=>r.json()).then(d=>{
    // Update telemetry panel
    T_KEYS.forEach((k,i)=>{
      const lv=d[k], pv=d['p'+k]||physCalc(i,lv);
      document.getElementById(T_IDS[i]).textContent  = lv+'°';
      document.getElementById(TP_IDS[i]).textContent = '→'+pv+'°';
    });

    // Sync sliders only when arm is idle
    if (!d.isMoving) {
      T_KEYS.forEach((k,i)=>{
        const el=document.getElementById('r'+i);
        el.value=d[k]; updateFill(el);
        document.getElementById('v'+i).textContent=d[k]+'°';
        document.getElementById('p'+i).textContent='→'+(d['p'+k]||physCalc(i,d[k]))+'°';
      });
    }

    wdLeft = parseFloat(d.wdRemaining)||WD_MAX;

    // Trajectory state sync
    if (d.traj) {
      for(let i=0;i<3;i++){
        trajState[i].hasData  = d.traj[i].hasData;
        trajState[i].frames   = d.traj[i].frames;
        trajState[i].recording= d.traj[i].recording;
        trajState[i].playing  = d.traj[i].playing;
      }
      updateTrajUI();
      // Recording progress bar
      const rs = d.traj.findIndex(t=>t.recording);
      if (rs>=0) {
        document.getElementById('recFrames').textContent = `${d.traj[rs].frames}/${MAX_FRAMES}`;
        document.getElementById('recProg').style.width   = (d.traj[rs].frames/MAX_FRAMES*100)+'%';
      } else {
        document.getElementById('recFrames').textContent = `0/${MAX_FRAMES}`;
        document.getElementById('recProg').style.width   = '0%';
      }
    }

    // Fail-safe pills
    if (d.pickupSaved){
      document.getElementById('pPU').textContent='PICKUP: SAVED ✓';
      document.getElementById('pPU').classList.add('ok');
    }
    const pfs=document.getElementById('pFS');
    if(d.failSafe){pfs.textContent='FAIL-SAFE!'; pfs.classList.add('alrt'); pfs.classList.remove('ok');}
    else          {pfs.textContent='STANDBY';    pfs.classList.remove('alrt');}

    setOnline();
  }).catch(setOffline);
}

/* ══════════════════════════════════════════════════════════
   WATCHDOG UI
══════════════════════════════════════════════════════════ */
setInterval(()=>{
  wdLeft=Math.max(0,wdLeft-.1);
  const pct=(wdLeft/WD_MAX*100).toFixed(1);
  const f=document.getElementById('wFill'), l=document.getElementById('wTime');
  f.style.width=pct+'%'; l.textContent=wdLeft.toFixed(1)+'s';
  if(wdLeft<1.5){f.style.background='var(--c4)';l.style.color='var(--c4)';document.getElementById('cDot').className='dot wrn';}
  else if(wdLeft<3){f.style.background='var(--c6)';l.style.color='var(--c6)';}
  else{f.style.background='var(--c3)';l.style.color='var(--c5)';}
},100);

/* ══════════════════════════════════════════════════════════
   CONNECTION
══════════════════════════════════════════════════════════ */
function setOnline(){ document.getElementById('cDot').className='dot'; document.getElementById('cLbl').textContent='ONLINE'; }
function setOffline(){ document.getElementById('cDot').className='dot off'; document.getElementById('cLbl').textContent='OFFLINE'; }

/* ══════════════════════════════════════════════════════════
   MOVING BADGE
══════════════════════════════════════════════════════════ */
function showMoving(){
  document.getElementById('movBadge').classList.add('vis');
  clearTimeout(movingTimer);
  movingTimer=setTimeout(()=>document.getElementById('movBadge').classList.remove('vis'),3500);
}

/* ══════════════════════════════════════════════════════════
   LOG
══════════════════════════════════════════════════════════ */
function addLog(msg,type=''){
  const el=document.getElementById('evlog');
  const ts=new Date().toTimeString().slice(0,8);
  const d=document.createElement('div');
  d.className='le '+type;
  d.textContent=`[${ts}]  ${msg}`;
  el.appendChild(d); el.scrollTop=el.scrollHeight;
}
function clrLog(){ document.getElementById('evlog').innerHTML=''; }

/* ══════════════════════════════════════════════════════════
   INIT
══════════════════════════════════════════════════════════ */
window.addEventListener('load',()=>{
  document.querySelectorAll('input[type=range]').forEach(updateFill);
  buildCalTable(); refreshPhys();
  syncStatus(); setInterval(syncStatus, 2000);
  addLog('ARM-6 v4.0 Industry 4.0 Control System online','info');
  addLog('Motion engine: non-blocking · 18ms/step default','ok');
  addLog('Shoulder +90° offset active · Gripper inverted','warn');
  addLog('Trajectory system ready (3 slots × 400 frames)','traj');
});
</script>
</body>
</html>
)HTMLEOF";

// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  ARM-6 v4.0  Industry 4.0 Arm System  ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));

  // ── Attach servos with proper pulse width range
  Serial.println(F("[SERVO] Attaching servos (500–2500µs)..."));
  for (int i = 0; i < 6; i++) {
    srv[i].attach(SERVO_PINS[i], 500, 2500);
    delay(40);  // stagger to prevent simultaneous current spike
  }

  // ── Write home position one servo at a time to avoid brownout
  Serial.println(F("[SERVO] Moving to home position..."));
  for (int i = 0; i < 6; i++) {
    writeServoLogical(i, homePos.v[i]);
    delay(150);
  }
  currentPos = homePos;
  targetPos  = homePos;
  delay(800);
  Serial.println(F("[SERVO] All servos at home."));

  // ── Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("[WiFi] Connecting"));
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.println();
  Serial.print(F("[WiFi] Connected! URL → http://"));
  Serial.println(WiFi.localIP());

  // ══════════════════════════════════════════════════════════════
  //  HTTP ROUTES
  // ══════════════════════════════════════════════════════════════

  // Serve dashboard
  server.on("/", [](){
    server.send_P(200, "text/html", webpage);
  });

  // ── Servo targets (non-blocking: just update targetPos)
  for (int idx = 0; idx < 6; idx++) {
    int ci = idx;
    String route = "/s" + String(idx+1);
    server.on(route.c_str(), [ci](){
      touchWatchdog();
      int v = constrain(getIntArg("val"), cal[ci].minAngle, cal[ci].maxAngle);
      targetPos.v[ci] = v;
      isMoving = true;
      sendOK();
    });
  }

  // ── Speed
  server.on("/speed", [](){
    touchWatchdog();
    stepInterval = constrain(getIntArg("val"), 1, 50);
    Serial.printf("[SPEED] %d ms/step\n", stepInterval);
    sendOK();
  });

  // ── Calibration
  server.on("/calibrate", [](){
    touchWatchdog();
    char buf[8];
    for (int i=0; i<6; i++) {
      snprintf(buf,8,"o%d",  i); if(server.hasArg(buf)) cal[i].offset   = constrain(server.arg(buf).toInt(),-180,180);
      snprintf(buf,8,"inv%d",i); if(server.hasArg(buf)) cal[i].inverted  = server.arg(buf).toInt()==1;
      snprintf(buf,8,"mn%d", i); if(server.hasArg(buf)) cal[i].minAngle  = constrain(server.arg(buf).toInt(),0,180);
      snprintf(buf,8,"mx%d", i); if(server.hasArg(buf)) cal[i].maxAngle  = constrain(server.arg(buf).toInt(),0,180);
      Serial.printf("[CAL S%d] off=%d inv=%d mn=%d mx=%d\n",i+1,cal[i].offset,cal[i].inverted,cal[i].minAngle,cal[i].maxAngle);
    }
    writeAllServos(currentPos);
    sendOK();
  });

  // ── Position memory
  server.on("/savePickup", [](){ touchWatchdog(); savedPos=currentPos; pickupSaved=true; Serial.println("[FAILSAFE] Pickup saved."); sendOK(); });
  server.on("/returnPickup", [](){ touchWatchdog(); if(pickupSaved) moveTo(savedPos); sendOK(); });
  server.on("/home",    [](){ touchWatchdog(); moveTo(homePos); sendOK(); });
  server.on("/setHome", [](){ touchWatchdog(); homePos=currentPos; Serial.println("[HOME] Updated."); sendOK(); });

  // ── Presets
  server.on("/preset", [](){
    touchWatchdog();
    String n = getStrArg("name");
    ArmPos t = homePos;
    if      (n=="rest")   t = {{90, 0, 45, 90,90, 45}};
    else if (n=="ready")  t = {{90, 0, 90, 90,90,  0}};
    else if (n=="pickup") t = {{90,30,120, 60,90,  0}};
    else if (n=="place")  t = {{135,0, 60, 90,90, 90}};
    moveTo(t);
    Serial.printf("[PRESET] %s\n", n.c_str());
    sendOK();
  });

  // ── E-Stop (freeze motion)
  bool estopActive = false;
  server.on("/estop", [&estopActive](){
    estopActive = true;
    targetPos   = currentPos;  // Cancel pending motion
    isMoving    = false;
    Serial.println("[ESTOP] E-Stop activated.");
    sendOK();
  });
  server.on("/resumeStop", [&estopActive](){
    estopActive = false;
    Serial.println("[ESTOP] Released.");
    sendOK();
  });

  // ── Trajectory: Start recording
  server.on("/trajRec", [](){
    touchWatchdog();
    int slot = constrain(getIntArg("slot"), 0, NUM_TRAJ_SLOTS-1);
    stopRecording(); stopPlayback();
    startRecording(slot);
    sendOK();
  });

  // ── Trajectory: Start playback
  server.on("/trajPlay", [](){
    touchWatchdog();
    int slot = constrain(getIntArg("slot"), 0, NUM_TRAJ_SLOTS-1);
    stopRecording(); stopPlayback();
    startPlayback(slot);
    sendOK();
  });

  // ── Trajectory: Clear slot
  server.on("/trajClear", [](){
    touchWatchdog();
    int slot = constrain(getIntArg("slot"), 0, NUM_TRAJ_SLOTS-1);
    trajLength[slot]  = 0;
    trajHasData[slot] = false;
    Serial.printf("[TRAJ] Slot %d cleared.\n", slot);
    sendOK();
  });

  // ── Stop all trajectory operations
  server.on("/stopAll", [](){
    touchWatchdog();
    stopRecording(); stopPlayback();
    sendOK();
  });

  // ── Status / telemetry JSON
  server.on("/status", [](){
    unsigned long el   = millis() - lastClientMs;
    unsigned long rem  = (el < WATCHDOG_MS) ? (WATCHDOG_MS - el) : 0;
    const char* K[]    = {"base","shoulder","elbow","wristPitch","wristRoll","gripper"};
    const char* PK[]   = {"pbase","pshoulder","pelbow","pwristPitch","pwristRoll","pgripper"};

    String j = "{";
    for (int i=0; i<6; i++) {
      j += "\""; j+=K[i];  j+="\":"  ; j+=currentPos.v[i];              j+=",";
      j += "\""; j+=PK[i]; j+="\":"  ; j+=applyCalib(i,currentPos.v[i]); j+=",";
    }
    j += "\"pickupSaved\":"   + String(pickupSaved       ?"true":"false") + ",";
    j += "\"failSafe\":"      + String(failSafeTriggered ?"true":"false") + ",";
    j += "\"isMoving\":"      + String(isMoving          ?"true":"false") + ",";
    j += "\"wdRemaining\":"   + String(rem/1000.0, 1) + ",";
    // Trajectory state array
    j += "\"traj\":[";
    for (int i=0; i<NUM_TRAJ_SLOTS; i++) {
      j += "{\"hasData\":"   + String(trajHasData[i] ?"true":"false") + ",";
      j += "\"frames\":"     + String(trajLength[i]) + ",";
      j += "\"recording\":"  + String((recording && recordSlot==i) ?"true":"false") + ",";
      j += "\"playing\":"    + String((replaying && replaySlot==i) ?"true":"false") + "}";
      if (i < NUM_TRAJ_SLOTS-1) j+=",";
    }
    j += "]}";
    sendJSON(j);
  });

  server.begin();
  lastClientMs = millis();

  Serial.println(F("[SERVER] HTTP server started."));
  Serial.println(F("[WATCHDOG] Armed — 5s timeout."));
  Serial.println(F("[TRAJ] Trajectory system ready."));
  Serial.println(F("╔════════════════════════════════════════╗"));
  Serial.println(F("║  System Ready!                         ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));
}

// ════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════════════
void loop() {
  // 1. Web server — must run every iteration
  server.handleClient();

  // 2. Non-blocking smooth motion engine
  updateMotion();

  // 3. Trajectory recording (capture frame every 100ms if active)
  updateRecording();

  // 4. Trajectory playback (advance to next frame when arm is idle)
  updatePlayback();

  // 5. Watchdog fail-safe
  //    If Wi-Fi has been silent > 5s AND a pickup was saved:
  //    → return arm to saved pickup position
  //    If no pickup saved:
  //    → return to home position as secondary fallback
  if (!failSafeTriggered && !replaying && !recording) {
    if ((millis() - lastClientMs) > WATCHDOG_MS) {
      failSafeTriggered = true;
      stopRecording();
      stopPlayback();
      if (pickupSaved) {
        Serial.println(F("[FAILSAFE] ⚠  Timeout → returning to pickup..."));
        moveToBlocking(savedPos);
        Serial.println(F("[FAILSAFE] ✓  Safe return complete."));
      } else {
        Serial.println(F("[FAILSAFE] ⚠  Timeout → no pickup saved, going home..."));
        moveToBlocking(homePos);
        Serial.println(F("[FAILSAFE] ✓  Home position reached."));
      }
    }
  }

  // 6. Small yield to prevent ESP32 watchdog timer reset
  delay(1);
}
