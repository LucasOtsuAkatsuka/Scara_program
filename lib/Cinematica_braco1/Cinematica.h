#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <map>

struct Coord { float x, y; };

class Cinematica {
public:
  // ====== PARÂMETROS PADRÃO DE GEOMETRIA ======
  static constexpr float DEF_L1 = 238.0f;
  static constexpr float DEF_L2 = 207.0f;
  static constexpr float DEF_TH1_MIN = -70.0f;
  static constexpr float DEF_TH1_MAX =  70.0f;
  static constexpr float DEF_TH2_MIN = -135.0f;
  static constexpr float DEF_TH2_MAX =  135.0f;

  static constexpr float   DEF_MOTOR_STEP_DEG = 1.8f;
  static constexpr uint8_t DEF_MICROSTEP_DIV  = 16; 
  static constexpr unsigned int DEF_DIR_SETUP_US = 2;

  static constexpr float DEF_MAX_SPEED_STEPS_S = 500.0f;
  static constexpr float DEF_ACCEL_STEPS_S2    = 200.0f;

  Cinematica(int step1, int dir1, int step2, int dir2, int enPin = -1);

  void begin(bool enable = true);

  void enableDriver(bool enable);

  bool goToIndex(uint8_t idx);
  bool goToXY(float x_mm, float y_mm);

  void moveSteps(uint8_t joint, long steps);

  void setOrigin();

  void setCurrentAnglesDeg(float th1_deg, float th2_deg);
  //void setMicrostepDivider(uint8_t divider);
  //void setPins(int step1, int dir1, int step2, int dir2, int enPin = -1);
  //void setGeometry(float L1_mm, float L2_mm);
  //void setLimits(float th1_min, float th1_max, float th2_min, float th2_max);
  //void setMotorStep(float motor_step_deg);
  //void setMapPoint(uint8_t idx, float x_mm, float y_mm);

  //void setMaxSpeed(float steps_per_s);
  //void setAcceleration(float steps_per_s2);
  void invertDir(bool joint1_invert, bool joint2_invert);

  long  currentStepsJoint1() const { return pos1_steps_; }
  long  currentStepsJoint2() const { return pos2_steps_; }
  float currentTheta1Deg()   const { return pos1_steps_ * step_eff_deg_; }
  float currentTheta2Deg()   const { return pos2_steps_ * step_eff_deg_; }

private:

  int STEP1_ = -1, DIR1_ = -1;
  int STEP2_ = -1, DIR2_ = -1;
  int EN_    = -1;

  float L1_ = DEF_L1, L2_ = DEF_L2;
  float th1_min_ = DEF_TH1_MIN, th1_max_ = DEF_TH1_MAX;
  float th2_min_ = DEF_TH2_MIN, th2_max_ = DEF_TH2_MAX;

  float   motor_step_deg_ = DEF_MOTOR_STEP_DEG;
  uint8_t microstep_div_  = DEF_MICROSTEP_DIV;
  float   step_eff_deg_   = DEF_MOTOR_STEP_DEG / DEF_MICROSTEP_DIV;

  long pos1_steps_ = 0, pos2_steps_ = 0;

  AccelStepper* m1_ = nullptr;
  AccelStepper* m2_ = nullptr;
  float max_speed_ = DEF_MAX_SPEED_STEPS_S;
  float accel_     = DEF_ACCEL_STEPS_S2;
  bool inv_dir1_ = false, inv_dir2_ = false;

  std::map<int, Coord> points_;

  bool  solveIK(float x, float y, float& th1_deg_out, float& th2_deg_out);
  long  degToSteps(float deg) const;
  bool  runToBothTargets(long target1, long target2);
  static bool inRange(float v, float vmin, float vmax);
  void  loadDefaultPoints();
};