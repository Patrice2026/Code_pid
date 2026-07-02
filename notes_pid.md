# Notes d'apprentissage PID

## Concepts clés

### P (Proportionnel)
- Plus l'erreur est grande, plus on corrige
- Kp contrôle la force de correction
- Formule : P_out = Kp × Error

### I (Intégral)
- Accumule les petites erreurs persistantes
- Ki élimine l'erreur résiduelle
- Formule : I_out = Ki × ∑Error

### D (Dérivé)
- Regarde la vitesse du changement
- Kd stabilise le système
- Formule : D_out = Kd × dError/dt

## Formule complète du PID
