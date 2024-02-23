#pragma once

class PID
{

public:
  PID(float p, float i, float d, float max, float min, float dt)
  {
    this->p = p;
    this->i = i;
    this->d = d;
    this->max = max;
    this->min = min;
    this->integral = 0;
    this->previous_error = 0;
    this->dt = dt;
  }

  float calculate(float setpoint, float pv)
  {
    float error = setpoint - pv;
    float p_term = p * error;
    integral += error * dt;
    float i_term = i * integral;
    float d_term = d * ((error - previous_error) / dt);
    previous_error = error;
    float output = p_term + i_term + d_term;
    if (output > max) {
      output = max;
    } else if (output < min) {
      output = min;
    }
    return output;
  }

  void reconfigure(float p, float i, float d, float max, float min)
  {
    this->p = p;
    this->i = i;
    this->d = d;
    this->max = max;
    this->min = min;

    reset();
  }

  void reset()
  {
    integral = 0;
    previous_error = 0;
  }

private:
  float p;
  float i;
  float d;
  float max;
  float min;
  float integral;
  float previous_error;
  float dt;
};
