## Mögliche Probleme

1. Distance Map hat zu geringen Kontrast
2. Zustandstransferfunktion funktioniert nicht richtig
3. Orientation mit Steering-Angle verwechselt

## ToDos
1. Simulationsdaten checken
    1. Accelerometer checken
    2. Koordinatensysteme in der Simulation darstellen
    3. Weightberechnung für Acceleration anpassen
    4. Distance Map verfeinern
    5. Eventuell nur auf Accelerometer gehen und verrauschtes nicht nehmen


# Notes for Sensors
## Orientation Sensor
compass	float - Orientation in radians. North is (0.0, -1.0, 0.0) in UE.

## Accelerometer
accelerometer - carla.Vector3D	Measures linear acceleration in m/s^2.