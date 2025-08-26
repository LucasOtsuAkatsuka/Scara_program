#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include <map.h>

struct Coord { float x, y; };

class Cinematica {
public:
  // =================== DEFAULTS EDITÁVEIS ===================
  static constexpr int   DEF_DIR1  = 3;
  static constexpr int   DEF_STEP1 = 2;
  static constexpr int   DEF_DIR2  = 5;
  static constexpr int   DEF_STEP2 = 4;
  static constexpr int   DEF_EN    = 8;     // -1 se não usar ENABLE por pino (A4988: LOW habilita)

  static constexpr float DEF_L1 = 238.0f;   // mm
  static constexpr float DEF_L2 = 207.0f;   // mm
  static constexpr float DEF_TH1_MIN = -70.0f;
  static constexpr float DEF_TH1_MAX =  70.0f;
  static constexpr float DEF_TH2_MIN = -135.0f;
  static constexpr float DEF_TH2_MAX =  135.0f;

  static constexpr float   DEF_MOTOR_STEP_DEG = 1.8f; // passo nominal do motor
  static constexpr uint8_t DEF_MICROSTEP_DIV  = 16;   // MS1/2/3 amarrados em 5V => 1/16
  static constexpr unsigned int DEF_DIR_SETUP_US = 2; // A4988 precisa ~200ns; margem aqui

  // Parâmetros dinâmicos do AccelStepper (ajustáveis)
  static constexpr float DEF_MAX_SPEED_STEPS_S = 2000.0f;  // passos/s (ajuste conforme sua mecânica)
  static constexpr float DEF_ACCEL_STEPS_S2    = 1500.0f;  // passos/s^2

  // =================== CONSTRUTOR ===================
  Cinematica();

  // =================== CICLO DE VIDA ===================
  void begin(bool enable = true);  // cria steppers, configura vel/acc, habilita EN (LOW)

  // Liga/desliga driver via EN (LOW=ligado). Ignorado se EN_ == -1.
  void enableDriver(bool enable);

  // =================== MOVIMENTO (bloqueante) ===================
  bool goToIndex(uint8_t idx);         // 1..10
  bool goToXY(float x_mm, float y_mm); // coordenada arbitrária

  // =================== AJUSTES ===================
  void setCurrentAnglesDeg(float th1_deg, float th2_deg); // após homing
  void setMicrostepDivider(uint8_t divider);              // se mudar hardware
  void setPins(int step1, int dir1, int step2, int dir2, int enPin = DEF_EN);
  void setGeometry(float L1_mm, float L2_mm);
  void setLimits(float th1_min, float th1_max, float th2_min, float th2_max);
  void setMotorStep(float motor_step_deg);
  void setMapPoint(uint8_t idx, float x_mm, float y_mm);

  // AccelStepper params
  void setMaxSpeed(float steps_per_s);     // para ambos
  void setAcceleration(float steps_per_s2);// para ambos
  void invertDir(bool joint1_invert, bool joint2_invert); // inverte DIR lógico (se preciso)

  // =================== CONSULTAS ===================
  long  currentStepsJoint1() const { return pos1_steps_; }
  long  currentStepsJoint2() const { return pos2_steps_; }
  float currentTheta1Deg()   const { return pos1_steps_ * step_eff_deg_; }
  float currentTheta2Deg()   const { return pos2_steps_ * step_eff_deg_; }

private:
  // ---------- Configurações internas ----------
  int STEP1_ = DEF_STEP1, DIR1_ = DEF_DIR1;
  int STEP2_ = DEF_STEP2, DIR2_ = DEF_DIR2;
  int EN_    = DEF_EN;

  float L1_ = DEF_L1, L2_ = DEF_L2;
  float th1_min_ = DEF_TH1_MIN, th1_max_ = DEF_TH1_MAX;
  float th2_min_ = DEF_TH2_MIN, th2_max_ = DEF_TH2_MAX;

  float   motor_step_deg_ = DEF_MOTOR_STEP_DEG;
  uint8_t microstep_div_  = DEF_MICROSTEP_DIV;             // 1,2,4,8,16
  float   step_eff_deg_   = DEF_MOTOR_STEP_DEG / DEF_MICROSTEP_DIV;

  long pos1_steps_ = 0, pos2_steps_ = 0;

  // AccelStepper
  AccelStepper* m1_ = nullptr;
  AccelStepper* m2_ = nullptr;
  float max_speed_ = DEF_MAX_SPEED_STEPS_S;
  float accel_     = DEF_ACCEL_STEPS_S2;
  bool inv_dir1_ = false, inv_dir2_ = false;

  // Mapa 1..10
  std::map<int, Coord> points_;

  // ---------- Internos ----------
  bool  solveIK(float x, float y, float& th1_deg_out, float& th2_deg_out);
  long  degToSteps(float deg) const;
  bool  runToBothTargets(long target1, long target2); // loop de execução (bloqueante)
  static bool inRange(float v, float vmin, float vmax);
  void  loadDefaultPoints();
};