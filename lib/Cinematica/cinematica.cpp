#include "Cinematica.h"
#include <math.h>

Cinematica::Cinematica(int stepPin1, int dirPin1,
                                 int stepPin2, int dirPin2,
                                 float L1_mm, float L2_mm,
                                 float theta1_min_deg, float theta1_max_deg,
                                 float theta2_min_deg, float theta2_max_deg,
                                 float step_deg, float microstep,
                                 unsigned int us_on, unsigned int us_off)
: L1_(L1_mm), L2_(L2_mm),
  th1_min_(theta1_min_deg), th1_max_(theta1_max_deg),
  th2_min_(theta2_min_deg), th2_max_(theta2_max_deg),
  STEP1_(stepPin1), DIR1_(dirPin1),
  STEP2_(stepPin2), DIR2_(dirPin2),
  step_deg_(step_deg), microstep_(microstep),
  us_on_(us_on), us_off_(us_off)
{
  step_eff_deg_ = step_deg_ * microstep_;
}

void Cinematica::begin() {
  pinMode(DIR1_, OUTPUT);
  pinMode(STEP1_, OUTPUT);
  pinMode(DIR2_, OUTPUT);
  pinMode(STEP2_, OUTPUT);
}

void Cinematica::setPoint(uint8_t idx, float x_mm, float y_mm) {
  if (idx < 1 || idx > 10) return;
  points_[idx] = {x_mm, y_mm};
}

void Cinematica::setPointsMap(const std::map<int, Coord>& m) {
  points_ = m;
}

bool Cinematica::goToIndex(uint8_t idx) {
  auto it = points_.find(idx);
  if (it == points_.end()) {
    Serial.println(F("[SCARA] Índice não encontrado no mapa."));
    return false;
  }
  return goToXY(it->second.x, it->second.y);
}

bool Cinematica::goToXY(float x_mm, float y_mm) {
  float th1_deg, th2_deg;
  if (!solveIK(x_mm, y_mm, th1_deg, th2_deg)) {
    Serial.println(F("[SCARA] Ponto fora do alcance ou fora de limites."));
    return false;
  }

  long target1 = degToSteps(th1_deg);
  long target2 = degToSteps(th2_deg);

  Serial.print(F("[SCARA] IK -> th1=")); Serial.print(th1_deg);
  Serial.print(F(" deg, th2=")); Serial.print(th2_deg);
  Serial.print(F(" deg | steps1=")); Serial.print(target1);
  Serial.print(F(", steps2=")); Serial.println(target2);

  // Ordem de movimento: pode alternar, sincronizar, etc. Aqui: 1 depois 2.
  moveMotor(pos1_steps, target1, STEP1_, DIR1_);
  moveMotor(pos2_steps, target2, STEP2_, DIR2_);

  return true;
}

void Cinematica::setPulseTimings(unsigned int us_on, unsigned int us_off) {
  us_on_  = us_on;
  us_off_ = us_off;
}

void Cinematica::setCurrentAnglesDeg(float th1_deg, float th2_deg) {
  pos1_steps = degToSteps(th1_deg);
  pos2_steps = degToSteps(th2_deg);
}

bool Cinematica::solveIK(float x, float y, float& th1_deg_out, float& th2_deg_out) {
  // Fórmula clássica de 2R:
  // D = (x^2 + y^2 - L1^2 - L2^2)/(2 L1 L2)
  float r2 = x*x + y*y;
  float num = r2 - L1_*L1_ - L2_*L2_;
  float den = 2.0f * L1_ * L2_;
  float D = num / den;

  if (fabsf(D) > 1.0f) return false; // inalcançável

  float theta2_up = atan2f( sqrtf(1.0f - D*D), D );
  float theta2_dn = atan2f(-sqrtf(1.0f - D*D), D );

  // theta1 = atan2(y,x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
  auto th1_from_th2 = [&](float t2) {
    return atan2f(y, x) - atan2f(L2_ * sinf(t2), L1_ + L2_ * cosf(t2));
  };

  float th1_up = th1_from_th2(theta2_up);
  float th1_dn = th1_from_th2(theta2_dn);

  // rad -> deg
  float th1_up_deg = th1_up * 180.0f / PI;
  float th2_up_deg = theta2_up * 180.0f / PI;
  float th1_dn_deg = th1_dn * 180.0f / PI;
  float th2_dn_deg = theta2_dn * 180.0f / PI;

  bool valid_up = inRange(th1_up_deg, th1_min_, th1_max_) &&
                  inRange(th2_up_deg, th2_min_, th2_max_);
  bool valid_dn = inRange(th1_dn_deg, th1_min_, th1_max_) &&
                  inRange(th2_dn_deg, th2_min_, th2_max_);

  if (!valid_up && !valid_dn) return false;

  // Escolhe solução mais próxima da posição atual (em graus)
  float cur1_deg = pos1_steps * step_eff_deg_;
  float cur2_deg = pos2_steps * step_eff_deg_;

  float dist_up = fabsf(th1_up_deg - cur1_deg) + fabsf(th2_up_deg - cur2_deg);
  float dist_dn = fabsf(th1_dn_deg - cur1_deg) + fabsf(th2_dn_deg - cur2_deg);

  if (valid_up && (!valid_dn || dist_up <= dist_dn)) {
    th1_deg_out = th1_up_deg;
    th2_deg_out = th2_up_deg;
  } else {
    th1_deg_out = th1_dn_deg;
    th2_deg_out = th2_dn_deg;
  }

  return true;
}

long Cinematica::degToSteps(float deg) const {
  // arredonda para o passo mais próximo
  return lroundf(deg / step_eff_deg_);
}

void Cinematica::moveMotor(long& currentPos, long targetPos, int STEP_PIN, int DIR_PIN) {
  long delta = targetPos - currentPos;
  int dir = (delta >= 0) ? HIGH : LOW;
  digitalWrite(DIR_PIN, dir);

  long steps = labs(delta);
  for (long i = 0; i < steps; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(us_on_);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(us_off_);
  }
  currentPos = targetPos;
}

bool Cinematica::inRange(float v, float vmin, float vmax) {
  return (v >= vmin) && (v <= vmax);
}