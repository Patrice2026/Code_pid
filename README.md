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
 * @param pid Pointeur vers la structure PID
 * @param error Erreur actuelle (setpoint - measured)
 * @param dt Intervalle de temps en secondes
 * @return Sortie du PID (commande moteur)
 */
float pid_calculate(PID_Controller *pid, float error, float dt) {
    // Terme Proportionnel
    float p_term = pid->Kp * error;
    
    // Terme Intégral - accumule l'erreur
    pid->error_sum += error * dt;
    float i_term = pid->Ki * pid->error_sum;
    
    // Terme Dérivé - regarde la vitesse du changement
    float d_error = (error - pid->last_error) / dt;
    float d_term = pid->Kd * d_error;
    
    // Sortie totale
    float output = p_term + i_term + d_term;
    
    // Saturation (limiter la sortie)
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    
    // Mémoriser l'erreur pour le prochain calcul
    pid->last_error = error;
    
    return output;
}

/**
 * Réinitialiser le PID (utile entre les tests)
 */
void pid_reset(PID_Controller *pid) {
    pid->error_sum = 0;
    pid->last_error = 0;
}

/**
 * Exemple d'utilisation pour drone
 * Fréquence : 500 Hz (dt = 2ms)
 */
/*
int main() {
    // Créer 3 PID (Roll, Pitch, Yaw)
    PID_Controller roll_pid, pitch_pid, yaw_pid;
    
    // Initialiser avec Kp, Ki, Kd, max_out, min_out
    pid_init(&roll_pid, 4.5, 0.05, 2.0, 500, -500);
    pid_init(&pitch_pid, 4.5, 0.05, 2.0, 500, -500);
    pid_init(&yaw_pid, 4.0, 0.1, 1.5, 500, -500);
    
    // Dans la boucle de contrôle (500 Hz)
    while(1) {
        // Lire les capteurs
        float roll_angle = read_imu_roll();
        float pitch_angle = read_imu_pitch();
        float yaw_rate = read_gyro_z();
        
        // Erreurs
        float roll_error = 0 - roll_angle;  // On veut Roll = 0
        float pitch_error = 0 - pitch_angle;
        float yaw_error = desired_yaw_rate - yaw_rate;
        
        // Calculer corrections
        float roll_out = pid_calculate(&roll_pid, roll_error, 0.002);
        float pitch_out = pid_calculate(&pitch_pid, pitch_error, 0.002);
        float yaw_out = pid_calculate(&yaw_pid, yaw_error, 0.002);
        
        // Appliquer aux moteurs
        apply_pid_to_motors(roll_out, pitch_out, yaw_out);
    }
}
*/
