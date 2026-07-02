#include <stdint.h>

/**
 * Structure pour contrôleur PID
 */
typedef struct {
    float Kp;          // Gain proportionnel
    float Ki;          // Gain intégral
    float Kd;          // Gain dérivé
    
    float error_sum;   // Accumule les erreurs (intégral)
    float last_error;  // Erreur précédente (pour dérivé)
    
    float max_output;  // Limite de sortie
    float min_output;
} PID_Controller;

/**
 * Initialiser le contrôleur PID
 */
void pid_init(PID_Controller *pid, float Kp, float Ki, float Kd, 
              float max_out, float min_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->error_sum = 0;
    pid->last_error = 0;
    pid->max_output = max_out;
    pid->min_output = min_out;
}

/**
 * Calculer la sortie du PID
 */
float pid_calculate(PID_Controller *pid, float error, float dt) {
    // Terme Proportionnel
    float p_term = pid->Kp * error;
    
    // Terme Intégral
    pid->error_sum += error * dt;
    float i_term = pid->Ki * pid->error_sum;
    
    // Terme Dérivé
    float d_error = (error - pid->last_error) / dt;
    float d_term = pid->Kd * d_error;
    
    // Sortie totale
    float output = p_term + i_term + d_term;
    
    // Saturation
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    
    pid->last_error = error;
    
    return output;
}

/**
 * Réinitialiser le PID
 */
void pid_reset(PID_Controller *pid) {
    pid->error_sum = 0;
    pid->last_error = 0;
}
