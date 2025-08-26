#include "Cinematica.h"
#include <math.h>

// =================== CONSTRUTOR ===================
Cinematica::Cinematica(int step1, int dir1, int step2, int dir2, int enPin) {
  STEP1_ = step1;
  DIR1_  = dir1;
  STEP2_ = step2;
  DIR2_  = dir2;
  EN_    = enPin;
  step_eff_deg_ = motor_step_deg_ / (float)microstep_div_;
  loadDefaultPoints();
}

// =================== BEGIN / ENABLE ===================
void Cinematica::begin(bool enable) {
  pinMode(DIR1_, OUTPUT);  pinMode(STEP1_, OUTPUT);
  pinMode(DIR2_, OUTPUT);  pinMode(STEP2_, OUTPUT);

  if (EN_ >= 0) { pinMode(EN_, OUTPUT); enableDriver(enable); }

  m1_ = new AccelStepper(AccelStepper::DRIVER, STEP1_, DIR1_);
  m2_ = new AccelStepper(AccelStepper::DRIVER, STEP2_, DIR2_);

  m1_->setPinsInverted(inv_dir1_, false, false);
  m2_->setPinsInverted(inv_dir2_, false, false);

  m1_->setMaxSpeed(max_speed_);
  m1_->setAcceleration(accel_);
  m2_->setMaxSpeed(max_speed_);
  m2_->setAcceleration(accel_);

  m1_->setCurrentPosition(pos1_steps_);
  m2_->setCurrentPosition(pos2_steps_);
}

void Cinematica::enableDriver(bool enable) {
  if (EN_ < 0) return;
  digitalWrite(EN_, enable ? LOW : HIGH);
}

// =================== MOVIMENTO (bloqueante) ===================
bool Cinematica::goToIndex(uint8_t idx) {
  if (idx < 1 || idx > 10) {
    Serial.println(F("[SCARA] Índice inválido (use 1..10)."));
    return false;
  }
  const Coord c = points_[idx];
  return goToXY(c.x, c.y);
}

bool Cinematica::goToXY(float x_mm, float y_mm) {
  float th1_deg, th2_deg;
  if (!solveIK(x_mm, y_mm, th1_deg, th2_deg)) {
    Serial.println(F("[SCARA] Ponto fora do alcance/limites."));
    return false;
  }
  long target1 = degToSteps(th1_deg);
  long target2 = degToSteps(th2_deg);

  Serial.print(F("[SCARA] IK -> th1=")); Serial.print(th1_deg);
  Serial.print(F(" deg, th2=")); Serial.print(th2_deg);
  Serial.print(F(" deg | s1->")); Serial.print(target1);
  Serial.print(F(" s2->")); Serial.println(target2);

  return runToBothTargets(target1, target2);
}

// =================== AJUSTES ===================
void Cinematica::setCurrentAnglesDeg(float th1_deg, float th2_deg) {
  pos1_steps_ = degToSteps(th1_deg);
  pos2_steps_ = degToSteps(th2_deg);
  if (m1_) m1_->setCurrentPosition(pos1_steps_);
  if (m2_) m2_->setCurrentPosition(pos2_steps_);
}

void Cinematica::invertDir(bool joint1_invert, bool joint2_invert) {
  inv_dir1_ = joint1_invert;
  inv_dir2_ = joint2_invert;
  if (m1_) m1_->setPinsInverted(inv_dir1_, false, false);
  if (m2_) m2_->setPinsInverted(inv_dir2_, false, false);
}

// =================== INTERNOS ===================
bool Cinematica::solveIK(float x, float y, float& th1_deg_out, float& th2_deg_out) {
  float r2 = x*x + y*y;
  float D = (r2 - L1_*L1_ - L2_*L2_) / (2.0f * L1_ * L2_);
  if (fabsf(D) > 1.0f) return false;

  float t2_up = atan2f( sqrtf(1.0f - D*D), D );
  float t2_dn = atan2f(-sqrtf(1.0f - D*D), D );

  auto th1_from_t2 = [&](float t2){
    return atan2f(y, x) - atan2f(L2_ * sinf(t2), L1_ + L2_ * cosf(t2));
  };

  float t1_up = th1_from_t2(t2_up);
  float t1_dn = th1_from_t2(t2_dn);

  float t1_up_d = t1_up * 180.0f/PI, t2_up_d = t2_up * 180.0f/PI;
  float t1_dn_d = t1_dn * 180.0f/PI, t2_dn_d = t2_dn * 180.0f/PI;

  bool up_ok = inRange(t1_up_d, th1_min_, th1_max_) && inRange(t2_up_d, th2_min_, th2_max_);
  bool dn_ok = inRange(t1_dn_d, th1_min_, th1_max_) && inRange(t2_dn_d, th2_min_, th2_max_);
  if (!up_ok && !dn_ok) return false;

  float cur1 = pos1_steps_ * step_eff_deg_;
  float cur2 = pos2_steps_ * step_eff_deg_;
  float d_up = fabsf(t1_up_d - cur1) + fabsf(t2_up_d - cur2);
  float d_dn = fabsf(t1_dn_d - cur1) + fabsf(t2_dn_d - cur2);

  if (up_ok && (!dn_ok || d_up <= d_dn)) { th1_deg_out = t1_up_d; th2_deg_out = t2_up_d; }
  else                                   { th1_deg_out = t1_dn_d; th2_deg_out = t2_dn_d; }

  return true;
}

long Cinematica::degToSteps(float deg) const {
  return lroundf(deg / step_eff_deg_);
}

bool Cinematica::runToBothTargets(long target1, long target2) {
  if (!m1_ || !m2_) return false;

  // Prepara direção (garante DIR estável antes dos pulsos STEP)
  delayMicroseconds(DEF_DIR_SETUP_US);

  m1_->moveTo(target1);
  m2_->moveTo(target2);

  // Loop bloqueante: acelera, move e desacelera ambos até chegar
  while ( (m1_->distanceToGo() != 0) || (m2_->distanceToGo() != 0) ) {
    m1_->run();
    m2_->run();
  }

  // Atualiza nosso estado discreto (caso o AccelStepper arredonde diferente)
  pos1_steps_ = m1_->currentPosition();
  pos2_steps_ = m2_->currentPosition();
  return true;
}

bool Cinematica::inRange(float v, float vmin, float vmax) {
  return (v >= vmin) && (v <= vmax);
}


void Cinematica::moveSteps(uint8_t joint, long steps) {
  if (!m1_ || !m2_) return;

  if (joint == 1) {
    long target = pos1_steps_ + steps;
    m1_->moveTo(target);
    while (m1_->distanceToGo() != 0) {
      m1_->run();
    }
    pos1_steps_ = m1_->currentPosition();
  }
  else if (joint == 2) {
    long target = pos2_steps_ + steps;
    m2_->moveTo(target);
    while (m2_->distanceToGo() != 0) {
      m2_->run();
    }
    pos2_steps_ = m2_->currentPosition();
  }
}

void Cinematica::setOrigin() {
  if (m1_) {
    m1_->setCurrentPosition(0);
    pos1_steps_ = 0;              
  }
  if (m2_) {
    m2_->setCurrentPosition(0);
    pos2_steps_ = 0;
  }
  Serial.println(F("[SCARA] Origem redefinida em (0,0)."));
}

void Cinematica::loadDefaultPoints() {
  points_[1]  = { 50.0f,   188.0f };
  points_[2]  = { 48.0f,   252.0f };
  points_[3]  = { 47.0f,   319.0f };
  points_[4]  = { 82.0f,   376.0f };
  points_[5]  = { 146.0f,  400.0f };
  points_[6]  = { 200.0f,  150.0f };
  points_[7]  = { 200.0f, -150.0f };
  points_[8]  = { 380.0f,  -10.0f };
  points_[9]  = { 260.0f,   40.0f };
  points_[10] = { 385.95f,-183.37f };
}

//void Cinematica::setMicrostepDivider(uint8_t divider) {
//  microstep_div_ = divider ? divider : 1;
//  step_eff_deg_  = motor_step_deg_ / (float)microstep_div_;
//}
//
//void Cinematica::setPins(int step1, int dir1, int step2, int dir2, int enPin) {
//  STEP1_ = step1; DIR1_ = dir1; STEP2_ = step2; DIR2_ = dir2; EN_ = enPin;
//}
//
//void Cinematica::setGeometry(float L1_mm, float L2_mm) {
//  L1_ = L1_mm; L2_ = L2_mm;
//}
//
//void Cinematica::setLimits(float th1_min, float th1_max, float th2_min, float th2_max) {
//  th1_min_ = th1_min; th1_max_ = th1_max;
//  th2_min_ = th2_min; th2_max_ = th2_max;
//}
//
//void Cinematica::setMotorStep(float motor_step_deg) {
//  motor_step_deg_ = motor_step_deg;
//  step_eff_deg_   = motor_step_deg_ / (float)microstep_div_;
//}
//
//void Cinematica::setMapPoint(uint8_t idx, float x_mm, float y_mm) {
//  if (idx < 1 || idx > 10) return;
//  points_[idx] = {x_mm, y_mm};
//}
//
//void Cinematica::setMaxSpeed(float steps_per_s) {
//  max_speed_ = steps_per_s;
//  if (m1_) m1_->setMaxSpeed(max_speed_);
//  if (m2_) m2_->setMaxSpeed(max_speed_);
//}
//
//void Cinematica::setAcceleration(float steps_per_s2) {
//  accel_ = steps_per_s2;
//  if (m1_) m1_->setAcceleration(accel_);
//  if (m2_) m2_->setAcceleration(accel_);
//}