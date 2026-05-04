// Copyright (c) Takao Akaki
#include "Stackchan_servo.h"

#include <ServoEasing.hpp>

static long convertSCS0009Pos(int16_t degree) {
  //Serial.printf("Degree: %d\n", degree);
  return map(degree, 0, 300, 1023, 0);
}

static long convertDYNIXELXL330(int16_t degree) {
  M5_LOGI("Degree: %d\n", degree);
  
  long ret =  map(degree, 0, 360, 0, 4095);
  M5_LOGI("Position: %d\n", ret);
  return ret;
}

static long convertDYNIXELXL330_RT(int16_t degree) {
  M5_LOGI("Degree: %d\n", degree);
  
  long ret =  map(degree, -360, 720, -4095, 8191);
  M5_LOGI("Position: %d\n", ret);
  return ret;
}

// シリアルサーボ用のEasing関数
float quadraticEaseInOut(float p) {
  //return p;
  if(p < 0.5)
	{
		return 2 * p * p;
	}
	else
	{
		return (-2 * p * p) + (4 * p) - 1;
	}
}


StackchanSERVO::StackchanSERVO() {}

StackchanSERVO::~StackchanSERVO() {}

float StackchanSERVO::getPosition(int x){
  if (_servo_type == RT_DYN_XL330){
    return _dxl.getPresentPosition(x);;
  } else {
    M5_LOGI("getPosition::Command is only supprted in RT_DYN_XL330");
  }
};

void StackchanSERVO::attachServos() {
  if (_servo_type == ServoType::SCS || _servo_type == ServoType::SCSCL_M5) {
    // SCS0009 / SCSCL (FEETECH SC series, common protocol)
    Serial2.begin(1000000, SERIAL_8N1, _init_param.servo[AXIS_X].pin, _init_param.servo[AXIS_Y].pin);
    delay(500);
    _sc.pSerial = &Serial2;

    if (_servo_type == ServoType::SCSCL_M5) {
      // Phase 2 safety: 3-step boot sequence for SCSCL_M5
      //   Step 1: Disable torque to safely reset prior state
      //   Step 2: Force position-control mode (clear EEPROM PWM/wheel mode if any)
      //   Step 3: Re-enable torque so subsequent WritePos commands work
      // Why all three: SCSCL::WritePos does NOT implicitly enable torque (verified in
      // SCServo lib source SCSCL.cpp). Without Step 3, servo would not respond to commands.

      // Step 1: torque off (safety reset, prevents unexpected motion during PWMMode change)
      _sc.EnableTorque(AXIS_X + 1, 0);
      _sc.EnableTorque(AXIS_Y + 1, 0);
      delay(100);
      M5_LOGI("SCSCL_M5: step 1 - torque disabled for safety reset");

      // Step 2: force position-control mode (NOT PWM/wheel mode).
      // M5Stack official firmware sets yaw to PWM mode (enablePwmMode=true) and stores
      // it in EEPROM. If left in PWM mode, our position-control WritePos commands would
      // cause unexpected continuous rotation.
      _sc.PWMMode(AXIS_X + 1, false);  // yaw   → position-control mode
      _sc.PWMMode(AXIS_Y + 1, false);  // pitch → position-control mode (likely already)
      delay(100);
      M5_LOGI("SCSCL_M5: step 2 - forced position-control mode (PWM/wheel mode disabled)");

      // Step 3: re-enable torque in safe (position-control) mode so WritePos works.
      // Now any WritePos call will be acted upon, but constrained to angle_limit clamp.
      _sc.EnableTorque(AXIS_X + 1, 1);
      _sc.EnableTorque(AXIS_Y + 1, 1);
      delay(100);
      M5_LOGI("SCSCL_M5: step 3 - torque re-enabled in position-control mode");

      // Phase 2 safety: set default angle limits (degree) for SCSCL if user did not specify.
      // These conservative limits prevent moveX/moveY from exceeding physical stops.
      // Based on M5Stack official hal_servo.cpp center values (yaw=460 raw≈165deg, pitch=620≈118deg)
      // with ±45 degree safe margin. Adjust empirically per individual unit.
      if (_init_param.servo[AXIS_X].lower_limit == 0 && _init_param.servo[AXIS_X].upper_limit == 0) {
        _init_param.servo[AXIS_X].lower_limit = 120;  // yaw  center ≈ 165 - 45
        _init_param.servo[AXIS_X].upper_limit = 210;  // yaw  center ≈ 165 + 45
        M5_LOGI("SCSCL: default yaw limit applied (120 - 210 deg)");
      }
      if (_init_param.servo[AXIS_Y].lower_limit == 0 && _init_param.servo[AXIS_Y].upper_limit == 0) {
        _init_param.servo[AXIS_Y].lower_limit = 90;   // pitch center ≈ 118 - 28 (downward limited)
        _init_param.servo[AXIS_Y].upper_limit = 160;  // pitch center ≈ 118 + 42 (upward)
        M5_LOGI("SCSCL: default pitch limit applied (90 - 160 deg)");
      }

      // NOTE: Servo will not move until external code explicitly calls moveX/moveY/moveXY,
      // which calls _sc.WritePos() — that command implicitly re-enables torque.
      // First move should be small (close to current physical position) to avoid jerks.
      _last_degree_x = _init_param.servo[AXIS_X].start_degree;
      _last_degree_y = _init_param.servo[AXIS_Y].start_degree;
      return;
    }

    // SCS0009 default behavior: immediately move to start_degree
    _sc.WritePos(AXIS_X + 1, convertSCS0009Pos(_init_param.servo[AXIS_X].start_degree + _init_param.servo[AXIS_X].offset), 1000);
    _sc.WritePos(AXIS_Y + 1, convertSCS0009Pos(_init_param.servo[AXIS_Y].start_degree + _init_param.servo[AXIS_Y].offset), 1000);
    vTaskDelay(1000/portTICK_PERIOD_MS);

  } else if (_servo_type == ServoType::DYN_XL330) {
    M5_LOGI("DYN_XL330");
    Serial2.begin(1000000, SERIAL_8N1, _init_param.servo[AXIS_X].pin, _init_param.servo[AXIS_Y].pin);
    _dxl = Dynamixel2Arduino(Serial2);
    _dxl.begin(1000000);
    _dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    _dxl.ping(AXIS_X + 1);
    _dxl.ping(AXIS_Y + 1);
    _dxl.setOperatingMode(AXIS_X + 1, OP_POSITION);
    _dxl.setOperatingMode(AXIS_Y + 1, OP_POSITION);
    _dxl.writeControlTableItem(DRIVE_MODE, AXIS_X + 1, 4);  // Velocityのパラメータを移動時間(msec)で指定するモードに変更
    _dxl.writeControlTableItem(DRIVE_MODE, AXIS_Y + 1, 4);  // Velocityのパラメータを移動時間(msec)で指定するモードに変更
    _dxl.torqueOn(AXIS_X + 1);
    delay(100); // ここでWaitを入れないと、Y(tilt)サーボが動かない場合がある。
    _dxl.torqueOn(AXIS_Y + 1);
    delay(100);
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_X + 1, 1000);
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_Y + 1, 1000);
    delay(100);
    _dxl.setGoalPosition(AXIS_X + 1, 2048);
    _dxl.setGoalPosition(AXIS_Y + 1, 3073);
    //_dxl.torqueOff(AXIS_X + 1);
    //_dxl.torqueOff(AXIS_Y + 1);
    
  } else if (_servo_type == ServoType::RT_DYN_XL330){
    M5_LOGI("RT_DYN_XL330");
    Serial2.begin(1000000, SERIAL_8N1, _init_param.servo[AXIS_X].pin, _init_param.servo[AXIS_Y].pin);
    _dxl = Dynamixel2Arduino(Serial2);
    _dxl.begin(1000000);
    _dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    _dxl.ping(AXIS_X + 1);
    _dxl.ping(AXIS_Y + 1);
    _dxl.setOperatingMode(AXIS_X + 1, OP_EXTENDED_POSITION);
    _dxl.setOperatingMode(AXIS_Y + 1, OP_EXTENDED_POSITION);
    _dxl.writeControlTableItem(DRIVE_MODE, AXIS_X + 1, 4);  // Velocityのパラメータを移動時間(msec)で指定するモードに変更
    _dxl.writeControlTableItem(DRIVE_MODE, AXIS_Y + 1, 4);  // Velocityのパラメータを移動時間(msec)で指定するモードに変更
    _dxl.torqueOn(AXIS_X + 1);
    delay(10); // ここでWaitを入れないと、Y(tilt)サーボが動かない場合がある。
    _dxl.torqueOn(AXIS_Y + 1);
    delay(100);
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_X + 1, 1000);
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_Y + 1, 1000);
    delay(100);

    M5_LOGI("CurrentPosition X:%f, Y:%f",  _dxl.getPresentPosition(AXIS_X + 1), _dxl.getPresentPosition(AXIS_Y + 1));

    if (_dxl.getPresentPosition(AXIS_X + 1) > 4096) {
      _init_param.servo[AXIS_X].offset = _init_param.servo[AXIS_X].offset + 360;
    }
    if ((_dxl.getPresentPosition(AXIS_Y + 1)-convertDYNIXELXL330_RT(_init_param.servo[AXIS_Y].lower_limit + _init_param.servo[AXIS_Y].offset)) > convertDYNIXELXL330_RT(270)) {
      _init_param.servo[AXIS_Y].offset = _init_param.servo[AXIS_Y].offset + 360;
    }
    //_init_param.servo[AXIS_Y].offset = 360;
    
    M5_LOGI("Current Offset X:%d, Y:%d", _init_param.servo[AXIS_X].offset, _init_param.servo[AXIS_Y].offset);

    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330_RT(_init_param.servo[AXIS_X].start_degree + _init_param.servo[AXIS_X].offset));
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330_RT(_init_param.servo[AXIS_Y].start_degree + _init_param.servo[AXIS_Y].offset));
    //_dxl.torqueOff(AXIS_X + 1);
    //_dxl.torqueOff(AXIS_Y + 1);

  } else {
    // SG90 PWM
    if (_servo_x.attach(_init_param.servo[AXIS_X].pin, 
                        _init_param.servo[AXIS_X].start_degree + _init_param.servo[AXIS_X].offset,
                        DEFAULT_MICROSECONDS_FOR_0_DEGREE,
                        DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
      Serial.print("Error attaching servo x");
    }
    if (_servo_y.attach(_init_param.servo[AXIS_Y].pin, 
                        _init_param.servo[AXIS_Y].start_degree + _init_param.servo[AXIS_Y].offset,
                        DEFAULT_MICROSECONDS_FOR_0_DEGREE,
                        DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
      Serial.print("Error attaching servo y");
    }

    _servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
    _servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);
  }
  _last_degree_x = _init_param.servo[AXIS_X].start_degree;
  _last_degree_y = _init_param.servo[AXIS_Y].start_degree;
}

void StackchanSERVO::begin(stackchan_servo_initial_param_s init_param) {
  _init_param = init_param;
  attachServos();
}

void StackchanSERVO::begin(int servo_pin_x, int16_t start_degree_x, int16_t offset_x,
                           int servo_pin_y, int16_t start_degree_y, int16_t offset_y,
                           ServoType servo_type) {
  // SAFETY: Explicitly zero-init lower/upper_limit. Caller does not pass these,
  // so without this, _init_param.servo[].lower_limit / upper_limit hold garbage values.
  // Garbage values cause constrain() in moveX/Y/XY to clamp to bizarre numbers
  // (e.g. "165 clamped to 16477"), which could damage the servo.
  // For SCSCL_M5, attachServos() will then set safe defaults (yaw 120-210, pitch 90-160).
  _init_param.servo[AXIS_X].pin          = servo_pin_x;
  _init_param.servo[AXIS_X].start_degree = start_degree_x;
  _init_param.servo[AXIS_X].offset       = offset_x;
  _init_param.servo[AXIS_X].lower_limit  = 0;
  _init_param.servo[AXIS_X].upper_limit  = 0;
  _init_param.servo[AXIS_Y].pin          = servo_pin_y;
  _init_param.servo[AXIS_Y].start_degree = start_degree_y;
  _init_param.servo[AXIS_Y].offset       = offset_y;
  _init_param.servo[AXIS_Y].lower_limit  = 0;
  _init_param.servo[AXIS_Y].upper_limit  = 0;
  _servo_type = servo_type;
  attachServos();
}

void StackchanSERVO::moveX(int x, uint32_t millis_for_move) {
  // Phase 2 safety: clamp angle to user-defined or default safe range
  if ((_init_param.servo[AXIS_X].lower_limit < _init_param.servo[AXIS_X].upper_limit)) {
    int clamped = constrain(x, _init_param.servo[AXIS_X].lower_limit, _init_param.servo[AXIS_X].upper_limit);
    if (clamped != x) {
      M5_LOGW("moveX: angle %d clamped to %d (limit %d-%d)",
              x, clamped, _init_param.servo[AXIS_X].lower_limit, _init_param.servo[AXIS_X].upper_limit);
      x = clamped;
    }
  }
  if (_servo_type == SCS || _servo_type == SCSCL_M5) {
    _sc.WritePos(AXIS_X + 1, convertSCS0009Pos(x + _init_param.servo[AXIS_X].offset), millis_for_move);
    _isMoving = true;
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
  } else if (_servo_type == ServoType::DYN_XL330) {
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_X + 1, millis_for_move);
    vTaskDelay(10/portTICK_PERIOD_MS);
    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330(x + _init_param.servo[AXIS_X].offset));
    vTaskDelay(10/portTICK_PERIOD_MS);
    _isMoving = true;
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
  } else if (_servo_type == ServoType::RT_DYN_XL330) {
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_X + 1, millis_for_move);
    vTaskDelay(10/portTICK_PERIOD_MS);
    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330_RT(x + _init_param.servo[AXIS_X].offset));
    vTaskDelay(10/portTICK_PERIOD_MS);
    _isMoving = true;
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
    M5_LOGI("X:%f", getPosition(AXIS_X+1));
  }else {
    if (millis_for_move == 0) {
      _servo_x.easeTo(x + _init_param.servo[AXIS_X].offset);
    } else {
      _servo_x.easeToD(x + _init_param.servo[AXIS_X].offset, millis_for_move);
    }
    _isMoving = true;
    synchronizeAllServosStartAndWaitForAllServosToStop();
    _isMoving = false;
  }
  _last_degree_x = x;
}

void StackchanSERVO::moveX(servo_param_s servo_param_x) {
  _init_param.servo[AXIS_X].offset = servo_param_x.offset;
  moveX(servo_param_x.degree, servo_param_x.millis_for_move);
}

void StackchanSERVO::moveY(int y, uint32_t millis_for_move) {
  // Phase 2 safety: clamp angle to user-defined or default safe range
  if ((_init_param.servo[AXIS_Y].lower_limit < _init_param.servo[AXIS_Y].upper_limit)) {
    int clamped = constrain(y, _init_param.servo[AXIS_Y].lower_limit, _init_param.servo[AXIS_Y].upper_limit);
    if (clamped != y) {
      M5_LOGW("moveY: angle %d clamped to %d (limit %d-%d)",
              y, clamped, _init_param.servo[AXIS_Y].lower_limit, _init_param.servo[AXIS_Y].upper_limit);
      y = clamped;
    }
  }
  if (_servo_type == ServoType::SCS || _servo_type == ServoType::SCSCL_M5) {
    _sc.WritePos(AXIS_Y + 1, convertSCS0009Pos(y + _init_param.servo[AXIS_Y].offset), millis_for_move);
    _isMoving = true;
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
  } else if (_servo_type == ServoType::DYN_XL330) {
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_Y + 1, millis_for_move);
    vTaskDelay(10/portTICK_PERIOD_MS);
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330(y + _init_param.servo[AXIS_Y].offset)); // RT版に合わせて+180°しています。
    vTaskDelay(10/portTICK_PERIOD_MS);
    _isMoving = true;
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
  } else if (_servo_type == ServoType::RT_DYN_XL330) {
    _dxl.writeControlTableItem(PROFILE_VELOCITY, AXIS_Y + 1, millis_for_move);
    vTaskDelay(10/portTICK_PERIOD_MS);
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330_RT(y + _init_param.servo[AXIS_Y].offset)); // RT版に合わせて+180°しています。
    vTaskDelay(10/portTICK_PERIOD_MS);
    _isMoving = true;
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
    M5_LOGI("Y:%f", getPosition(AXIS_Y+1));
  } else {
    if (millis_for_move == 0) {
      _servo_y.easeTo(y + _init_param.servo[AXIS_Y].offset);
    } else {
      _servo_y.easeToD(y + _init_param.servo[AXIS_Y].offset, millis_for_move);
    }
    _isMoving = true;
    synchronizeAllServosStartAndWaitForAllServosToStop();
    _isMoving = false;
  }
  _last_degree_y = y;
}

void StackchanSERVO::moveY(servo_param_s servo_param_y) {
  _init_param.servo[AXIS_Y].offset = servo_param_y.offset;
  moveY(servo_param_y.degree, servo_param_y.millis_for_move);
}
void StackchanSERVO::moveXY(int x, int y, uint32_t millis_for_move) {
  // Phase 2 safety: clamp both axes to user-defined or default safe range
  if ((_init_param.servo[AXIS_X].lower_limit < _init_param.servo[AXIS_X].upper_limit)) {
    int cx = constrain(x, _init_param.servo[AXIS_X].lower_limit, _init_param.servo[AXIS_X].upper_limit);
    if (cx != x) { M5_LOGW("moveXY: x %d clamped to %d", x, cx); x = cx; }
  }
  if ((_init_param.servo[AXIS_Y].lower_limit < _init_param.servo[AXIS_Y].upper_limit)) {
    int cy = constrain(y, _init_param.servo[AXIS_Y].lower_limit, _init_param.servo[AXIS_Y].upper_limit);
    if (cy != y) { M5_LOGW("moveXY: y %d clamped to %d", y, cy); y = cy; }
  }
  if (_servo_type == ServoType::SCS || _servo_type == ServoType::SCSCL_M5) {
    int increase_degree_x = x - _last_degree_x;
    int increase_degree_y = y - _last_degree_y;
    uint32_t division_time = millis_for_move / SERIAL_EASE_DIVISION;
    _isMoving = true;
    //M5_LOGI("SCS: %d, %d, %d", increase_degree_x, increase_degree_y, division_time);
    for (float f=0.0f; f<1.0f; f=f+(1.0f/SERIAL_EASE_DIVISION)) {
      int x_pos = _last_degree_x + increase_degree_x * quadraticEaseInOut(f);
      int y_pos = _last_degree_y + increase_degree_y * quadraticEaseInOut(f);
      _sc.WritePos(AXIS_X + 1, convertSCS0009Pos(x_pos + _init_param.servo[AXIS_X].offset), division_time);
      _sc.WritePos(AXIS_Y + 1, convertSCS0009Pos(y_pos + _init_param.servo[AXIS_Y].offset), division_time);
      vTaskDelay(division_time);
    }
    _isMoving = false;
  } else if (_servo_type == ServoType::DYN_XL330) {
    _isMoving = true;
    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330(x + _init_param.servo[AXIS_X].offset)); 
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330(y + _init_param.servo[AXIS_Y].offset)); // RT版に合わせて+180°しています。
    _isMoving = false;
  } else if (_servo_type == ServoType::RT_DYN_XL330) {
    _isMoving = true;
    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330_RT(x + _init_param.servo[AXIS_X].offset)); 
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330_RT(y + _init_param.servo[AXIS_Y].offset)); // RT版に合わせて+180°しています。
    _isMoving = false;
    M5_LOGI("X:%f, Y:%f", getPosition(AXIS_X+1), getPosition(AXIS_Y+1));
  } else {
    _servo_x.setEaseToD(x + _init_param.servo[AXIS_X].offset, millis_for_move);
    _servo_y.setEaseToD(y + _init_param.servo[AXIS_Y].offset, millis_for_move);
    _isMoving = true;
    synchronizeAllServosStartAndWaitForAllServosToStop();
    _isMoving = false;
  }
  _last_degree_x = x;
  _last_degree_y = y;
  //M5_LOGI("SCS: %d, %d", _last_degree_x, _last_degree_y);
}

void StackchanSERVO::moveXY(servo_param_s servo_param_x, servo_param_s servo_param_y) {
  // Phase 2 safety: clamp both axes to user-defined or default safe range
  if ((_init_param.servo[AXIS_X].lower_limit < _init_param.servo[AXIS_X].upper_limit)) {
    int cx = constrain(servo_param_x.degree, _init_param.servo[AXIS_X].lower_limit, _init_param.servo[AXIS_X].upper_limit);
    if (cx != servo_param_x.degree) { M5_LOGW("moveXY(p): x %d clamped to %d", servo_param_x.degree, cx); servo_param_x.degree = cx; }
  }
  if ((_init_param.servo[AXIS_Y].lower_limit < _init_param.servo[AXIS_Y].upper_limit)) {
    int cy = constrain(servo_param_y.degree, _init_param.servo[AXIS_Y].lower_limit, _init_param.servo[AXIS_Y].upper_limit);
    if (cy != servo_param_y.degree) { M5_LOGW("moveXY(p): y %d clamped to %d", servo_param_y.degree, cy); servo_param_y.degree = cy; }
  }
  if (_servo_type == ServoType::SCS || _servo_type == ServoType::SCSCL_M5) {
    _sc.WritePos(AXIS_X + 1, convertSCS0009Pos(servo_param_x.degree + servo_param_x.offset), servo_param_x.millis_for_move);
    _sc.WritePos(AXIS_Y + 1, convertSCS0009Pos(servo_param_y.degree + servo_param_y.offset), servo_param_y.millis_for_move);
    _isMoving = true;
    vTaskDelay(max(servo_param_x.millis_for_move, servo_param_y.millis_for_move)/portTICK_PERIOD_MS);
    _isMoving = false;
  } else if (_servo_type == ServoType::DYN_XL330) {
    _isMoving = true;
    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330(servo_param_x.degree + _init_param.servo[AXIS_X].offset)); 
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330(servo_param_y.degree + _init_param.servo[AXIS_Y].offset)); // RT版に合わせて+180°しています。
    _isMoving = false;
  } else if (_servo_type == ServoType::RT_DYN_XL330) {
    _isMoving = true;
    _dxl.setGoalPosition(AXIS_X + 1, convertDYNIXELXL330_RT(servo_param_x.degree + _init_param.servo[AXIS_X].offset)); 
    _dxl.setGoalPosition(AXIS_Y + 1, convertDYNIXELXL330_RT(servo_param_y.degree + _init_param.servo[AXIS_Y].offset)); // RT版に合わせて+180°しています。
    _isMoving = false;
    M5_LOGI("X:%f, Y:%f", getPosition(AXIS_X+1), getPosition(AXIS_Y+1));
  } else {
    if (servo_param_x.degree != 0) {
      _servo_x.setEaseToD(servo_param_x.degree + servo_param_x.offset, servo_param_x.millis_for_move);
    }
    if (servo_param_y.degree != 0) {
      _servo_y.setEaseToD(servo_param_y.degree + servo_param_y.offset, servo_param_y.millis_for_move);
    }
    _isMoving = true;
    synchronizeAllServosStartAndWaitForAllServosToStop();
    _isMoving = false;
  }
  _last_degree_x = servo_param_x.degree;
  _last_degree_y = servo_param_y.degree;
}

// @uint32_t speed 0〜1000
void StackchanSERVO::turnX(uint32_t speed, bool is_cw, uint32_t millis_for_move) {
    if (speed >= 1000) {
      speed = 1000;
    }
    if (is_cw) {
      speed += 1000; // 逆回転時は+1000
    }
    Serial.printf("speed: %d\n", speed);
    _sc.PWMMode(1, true); // 回転モード
    _isMoving = true;
    _sc.WritePWM(1, speed);
    vTaskDelay(millis_for_move/portTICK_PERIOD_MS);
    _isMoving = false;
    _sc.PWMMode(1, false); // 位置決めモードへ戻す 
  return;
}

void StackchanSERVO::motion(Motion motion_number) {
    if (motion_number == nomove) return; 
    moveXY(90, 75, 500);
    switch(motion_number) {
        case greet: 
            moveY(90, 1000);
            moveY(75, 1000);
            break;
        case laugh:
            for (int i=0; i<5; i++) {
                moveY(80, 500);
                moveY(60, 500);
            }
            break;
        case nod:
            for (int i=0; i<5; i++) {
                moveY(90, 1000);
                moveY(60, 1000);
            }
            break;
        case refuse:
            for (int i=0; i<2; i++) {
                moveX(70,  500);
                moveX(110, 500);
            }
            break;
        case test:
            moveX(45,  1000);
            moveX(135, 1000);
            moveX(90, 1000);
            moveY(50, 1000);
            moveY(90, 1000);
            break;
        default:
            Serial.printf("invalid motion number: %d\n", motion_number);
            break;
    }
    delay(1000);
    moveXY(_init_param.servo[AXIS_X].start_degree, _init_param.servo[AXIS_Y].degree, 1000);
}

