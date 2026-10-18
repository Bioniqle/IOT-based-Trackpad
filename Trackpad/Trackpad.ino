#include <Servo.h>

const float W = 32, H = 18, R = 2.5;
const float PX[2] = {0, 16}, PY[2] = {9, 0}, OFF[2] = {-90, 0};
const float K[2] = {0.0343, 0.0343}, ZD[2] = {0.5, 0.5};
const float EM[2] = {0.5, 0.5}; 

const byte TRG[2] = {A2, A0}, ECH[2] = {A3, A1}, SRV[2] = {10, 9};

const int DIT[4] = {-3, 0, 3, 0};

// ASYMMETRIC BLIND SPOT & SERVO SPEED:
// BL[0] = 5.0cm (Sensor Y looking at Sensor X)
// BL[1] = 7.5cm (Enlarged blind spot for Sensor X looking at Sensor Y so it never catches it while sweeping)
const float BL[2] = {5.0, 7.5};

// SL[0] = 16 (Faster step limit for Left Servo Y to compensate for 18cm short axis geometry)
// SL[1] = 12 (Bottom Servo X step limit)
const float SL[2] = {16.0, 12.0};

const float GT = 8.5, SWS = 2; 
const int AMIN = 30, AMAX = 150, ML = 6;
const unsigned long TO = 5500; 

// Increased velocity responsiveness (VA = 0.65, LEAD = 3.5) so when one sensor sees 
// the hand move closer, the other servo reacts immediately
const float LEAD = 3.5, VA = 0.65, VMAX = 3.5;
float vx = 0, vy = 0;

Servo sv[2];
float ang[2] = {90, 90}, ba[2][4], bd[2][4], tx = 16, ty = 9, sw = 90;
int wr[2] = {90, 90}, swd = 1;
bool bo[2][4], lock = false;
byte wi = 0, dk = 0, miss = 0;

float rad(byte i, float a) {
  return (a + OFF[i]) * DEG_TO_RAD;
}

float ping(byte i) {
  digitalWrite(TRG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(TRG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRG[i], LOW);
  unsigned long t = pulseIn(ECH[i], HIGH, TO);
  return t ? t * K[i] * 0.5 : -1;
}

// Dynamic max distance from sensor tip (ox, oy) to the 32x18 walls based on angle r
float lim(byte i, float ox, float oy, float r, int servoDeg) {
  float dx = cos(r), dy = sin(r), t = 1000;
  if (dx > 0.001) t = min(t, (W - ox) / dx);
  else if (dx < -0.001) t = min(t, -ox / dx);
  if (dy > 0.001) t = min(t, (H - oy) / dy);
  else if (dy < -0.001) t = min(t, -oy / dy);

  // Sensor 0 (Left / Y @ 0,9) looking down toward Sensor 1 (Bottom / X @ 16,0)
  if (i == 0 && servoDeg <= 74) {
    t = min(t, 13.0f); 
  }
  
  // ENLARGED CUTOUT FOR SENSOR X (i == 1, Bottom @ 16,0) looking left toward Sensor Y (0,9):
  // Acoustic cone is 15° wide, so it starts grazing Sensor Y around 122°.
  // Capping distance to 12.0cm for all angles >= 122° completely stops sweep false-locks.
  if (i == 1 && servoDeg >= 122) {
    t = min(t, 12.0f);
  }

  return t;
}

bool acc(byte i, float d, float ox, float oy, float r, float qx, float qy, int servoDeg) {
  if (d < 2) return false;
  
  float px = ox + d * cos(r), py = oy + d * sin(r);
  
  // 1. Asymmetric Coordinate Blind Spot around the opposite sensor's tip (qx, qy) and base (PX, PY)
  if (hypot(px - qx, py - qy) < BL[i] || hypot(px - PX[1 - i], py - PY[1 - i]) < BL[i]) return false;
  
  // 2. Rectangular Box Exclusion around Sensor Y (X < 5.5cm and Y between 3.5cm and 14.5cm)
  // Guarantees Sensor X ignores the entire physical mounting zone of Sensor Y
  if (i == 1 && px < 5.5 && py > 3.5 && py < 14.5) return false;
  
  // 3. Dynamic Angle-Based Max Distance check
  float l = lim(i, ox, oy, r, servoDeg);
  if (d > l - EM[i]) return false;

  // 4. Tracking Gate when locked
  if (lock) return fabs(d - hypot(tx - ox, ty - oy)) < GT;
  return true;
}

void setup() {
  Serial.begin(9600);
  for (byte i = 0; i < 2; i++) {
    pinMode(TRG[i], OUTPUT);
    pinMode(ECH[i], INPUT);
    sv[i].attach(SRV[i]);
    sv[i].write(90);
    delay(500);
  }
}

void loop() {
  float d[2], r[2], sx[2], sy[2], hx[2], hy[2], ux[2], uy[2], tgt[2];
  byte n[2];

  // Reduced inter-ping delay to 15ms for faster loop rate & matched latency
  d[0] = ping(0);
  delay(15);
  d[1] = ping(1);
  delay(15);

  for (byte i = 0; i < 2; i++) {
    r[i] = rad(i, wr[i]);
    sx[i] = PX[i] + R * cos(r[i]);
    sy[i] = PY[i] + R * sin(r[i]);
  }

  float liveF[2];
  bool liveValid[2];

  for (byte i = 0; i < 2; i++) {
    liveF[i] = d[i] > 2 ? sqrt(d[i] * d[i] - ZD[i] * ZD[i]) : -1;
    liveValid[i] = acc(i, liveF[i], sx[i], sy[i], r[i], sx[1 - i], sy[1 - i], wr[i]);
    bo[i][wi] = liveValid[i];
    ba[i][wi] = r[i];
    bd[i][wi] = liveF[i];
  }
  wi = (wi + 1) & 3;

  for (byte i = 0; i < 2; i++) {
    float sa = 0, sdd = 0;
    n[i] = 0;
    for (byte k = 0; k < 4; k++) {
      if (bo[i][k]) {
        n[i]++;
        sa += ba[i][k];
        sdd += bd[i][k];
      }
    }
    if (n[i]) {
      // Blend live reading (70% weight) with buffer average (30% weight)
      // so when hand moves closer to one sensor, the coordinate updates immediately
      // and pulls the other servo without waiting 4 loops for the buffer to flush
      float a = liveValid[i] ? (0.7f * r[i] + 0.3f * (sa / n[i])) : (sa / n[i]);
      float dm = liveValid[i] ? (0.7f * liveF[i] + 0.3f * (sdd / n[i])) : (sdd / n[i]);
      ux[i] = cos(a);
      uy[i] = sin(a);
      hx[i] = PX[i] + (R + dm) * ux[i];
      hy[i] = PY[i] + (R + dm) * uy[i];
    }
  }

  byte tot = n[0] + n[1];

  if (tot >= 1) { 
    miss = 0;
    float fx, fy;
    if (n[0] && n[1]) {
      fx = (hx[0] + hx[1]) / 2;
      fy = (hy[0] + hy[1]) / 2;
      if (hypot(hx[0] - hx[1], hy[0] - hy[1]) > 2 * GT) {
        byte b = hypot(hx[0] - tx, hy[0] - ty) <= hypot(hx[1] - tx, hy[1] - ty) ? 0 : 1;
        fx = hx[b];
        fy = hy[b];
      } else {
        float c = ux[0] * uy[1] - uy[0] * ux[1];
        if (fabs(c) > 0.25) {
          float t0 = ((PX[1] - PX[0]) * uy[1] - (PY[1] - PY[0]) * ux[1]) / c;
          float ix = PX[0] + t0 * ux[0], iy = PY[0] + t0 * uy[0];
          if (ix > 0 && ix < W && iy > 0 && iy < H && hypot(ix - fx, iy - fy) < GT) {
            fx = (fx + ix) / 2;
            fy = (fy + iy) / 2;
          }
        }
      }
    } else if (n[0]) {
      // Only Left Sensor (Y) sees the hand: update X & Y from its ray
      fx = hx[0];
      fy = hy[0];
    } else {
      // Only Bottom Sensor (X) sees the hand moving closer/farther:
      // Immediately pull Y (and therefore Left Servo Y) along the ray!
      fx = hx[1];
      fy = hy[1];
    }
    
    float ox0 = tx, oy0 = ty;
    if (!lock) {
      tx = fx;
      ty = fy;
      vx = vy = 0;
      lock = true;
    } else {
      // Faster alpha gain (0.25 to 0.80) so single-sensor distance changes pull the other servo snappily
      float a = constrain(hypot(fx - tx, fy - ty) / 5.0, 0.25, 0.80);
      tx += a * (fx - tx);
      ty += a * (fy - ty);
      vx += VA * (constrain(tx - ox0, -VMAX, VMAX) - vx);
      vy += VA * (constrain(ty - oy0, -VMAX, VMAX) - vy);
    }
    tx = constrain(tx, 0, W);
    ty = constrain(ty, 0, H);
    
  } else if (lock && ++miss >= ML) {
    lock = false;
    sw = constrain(ang[0], 40, 140);
    memset(bo, 0, sizeof bo);
  } else if (lock) {
    tx = constrain(tx + vx, 0, W);
    ty = constrain(ty + vy, 0, H);
    vx *= 0.8;
    vy *= 0.8;
  }

  if (lock) {
    float ax = constrain(tx + LEAD * vx, 0, W);
    float ay = constrain(ty + LEAD * vy, 0, H);
    
    for (byte i = 0; i < 2; i++) {
      tgt[i] = atan2(ay - PY[i], ax - PX[i]) * RAD_TO_DEG - OFF[i] + DIT[dk];
      // Prevent a servo from spinning into itself if the hand is right on top of its own mount
      if (hypot(ax - PX[i], ay - PY[i]) < 3.5) tgt[i] = ang[i];
    }
    dk = (dk + 1) & 3;
    
    Serial.print(tx, 2);
    Serial.print(',');
    Serial.println(ty, 2);
  } else {
    sw += swd * SWS;
    if (sw >= 140) swd = -1;
    if (sw <= 40) swd = 1;
    tgt[0] = sw;
    tgt[1] = 180 - sw;
    Serial.println("OUT");
  }

  for (byte i = 0; i < 2; i++) {
    // Applies individual SL[i] speed limits so Servo Y (SL=16) keeps pace with Servo X (SL=12)
    ang[i] = constrain(ang[i] + constrain(tgt[i] - ang[i], -SL[i], SL[i]), AMIN, AMAX);
    int w = round(ang[i]);
    if (w != wr[i]) {
      sv[i].write(w);
      wr[i] = w;
    }
  }
}