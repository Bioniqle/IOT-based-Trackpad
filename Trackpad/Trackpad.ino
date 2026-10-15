#include <Servo.h>

const float W = 32, H = 18, R = 2.5;
const float PX[2] = {0, 16}, PY[2] = {9, 0}, OFF[2] = {-90, 0};
const float K[2] = {0.0343, 0.0343}, ZD[2] = {0.5, 0.5}, EM[2] = {1.5, 1.5};
const byte TRG[2] = {A2, A0}, ECH[2] = {A3, A1}, SRV[2] = {10, 9};
const int DIT[4] = {-3, 0, 3, 0};
const float BL = 4, GT = 7, SWS = 2, SL = 6;
const int AMIN = 35, AMAX = 145, ML = 6;
const unsigned long TO = 4000;

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

float lim(float ox, float oy, float r) {
  float dx = cos(r), dy = sin(r), t = 1000;
  if (dx > 0.001) t = min(t, (W - ox) / dx);
  else if (dx < -0.001) t = min(t, -ox / dx);
  if (dy > 0.001) t = min(t, (H - oy) / dy);
  else if (dy < -0.001) t = min(t, -oy / dy);
  return t;
}

bool acc(byte i, float d, float ox, float oy, float r, float qx, float qy) {
  if (d < 2) return false;
  float hx = ox + d * cos(r) - qx, hy = oy + d * sin(r) - qy;
  if (hx * hx + hy * hy < BL * BL) return false;
  float l = lim(ox, oy, r);
  if (lock) return fabs(d - hypot(tx - ox, ty - oy)) < GT && d < l + 1;
  return d < l - EM[i];
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

  d[0] = ping(0);
  delay(20);
  d[1] = ping(1);
  delay(20);

  for (byte i = 0; i < 2; i++) {
    r[i] = rad(i, wr[i]);
    sx[i] = PX[i] + R * cos(r[i]);
    sy[i] = PY[i] + R * sin(r[i]);
  }

  for (byte i = 0; i < 2; i++) {
    float f = d[i] > 2 ? sqrt(d[i] * d[i] - ZD[i] * ZD[i]) : -1;
    bo[i][wi] = acc(i, f, sx[i], sy[i], r[i], sx[1 - i], sy[1 - i]);
    ba[i][wi] = r[i];
    bd[i][wi] = f;
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
      float a = sa / n[i], dm = sdd / n[i];
      ux[i] = cos(a);
      uy[i] = sin(a);
      hx[i] = PX[i] + (R + dm) * ux[i];
      hy[i] = PY[i] + (R + dm) * uy[i];
    }
  }

  byte tot = n[0] + n[1];

  if (tot >= (lock ? 1 : 2)) {
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
    } else {
      byte b = n[0] ? 0 : 1;
      fx = hx[b];
      fy = hy[b];
    }
    if (!lock) {
      tx = fx;
      ty = fy;
      lock = true;
    } else {
      float a = constrain(hypot(fx - tx, fy - ty) / 6, 0.15, 0.7);
      tx += a * (fx - tx);
      ty += a * (fy - ty);
    }
    tx = constrain(tx, 0, W);
    ty = constrain(ty, 0, H);
  } else if (lock && ++miss >= ML) {
    lock = false;
    sw = constrain((ang[0] + ang[1]) / 2, 45, 135);
    memset(bo, 0, sizeof bo);
  }

  if (lock) {
    for (byte i = 0; i < 2; i++)
      tgt[i] = atan2(ty - PY[i], tx - PX[i]) * RAD_TO_DEG - OFF[i] + DIT[dk];
    dk = (dk + 1) & 3;
    Serial.print(tx, 2);
    Serial.print(',');
    Serial.println(ty, 2);
  } else {
    sw += swd * SWS;
    if (sw >= 135) swd = -1;
    if (sw <= 45) swd = 1;
    tgt[0] = tgt[1] = sw;
    Serial.println("OUT");
  }

  for (byte i = 0; i < 2; i++) {
    ang[i] = constrain(ang[i] + constrain(tgt[i] - ang[i], -SL, SL), AMIN, AMAX);
    int w = round(ang[i]);
    if (w != wr[i]) {
      sv[i].write(w);
      wr[i] = w;
    }
  }
}