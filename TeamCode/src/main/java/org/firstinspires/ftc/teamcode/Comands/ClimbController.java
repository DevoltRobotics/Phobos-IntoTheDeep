package org.firstinspires.ftc.teamcode.Comands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ClimbController {

    private double power;
    private double palanca;// Potencia actual del motor
    private final double maxPower; // Potencia máxima del motor
    private final double minPower; // Potencia máxima del motor
    private final double increaseRate; // Tasa de aumento de la potencia
    private final double decreaseRate; // Tasa de disminución de la potencia
    // Estado de la palanca

    public ClimbController(double maxPower, double minPower, double palanca, double increaseRate, double decreaseRate) {
        this.power = 0;
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.palanca = palanca;
        this.increaseRate = increaseRate;
        this.decreaseRate = decreaseRate;

    }


    public void update() {
        if (palanca > 0.5) {
            // Aumenta la potencia gradualmente hasta el máximo
            if (power < maxPower) {
                power += increaseRate;
                if (power > maxPower) {
                    power = maxPower;
                }
            }

        } else if (palanca < -0.5) {

            if (power > minPower) {
                power -= increaseRate;
                if (power < minPower) {
                    power = minPower;
                }
            }

        } else {
            // Disminuye la potencia gradualmente hasta cero
            if (power < 0) {
                power += decreaseRate;

            } else if (power > 0) {
                power -= decreaseRate;

            }
        }
    }

    public double getPower() {
        return power;
    }


}
