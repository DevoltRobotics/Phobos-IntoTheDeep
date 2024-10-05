package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.Nullable;

public class ArmController {
    
    public static final class PIDCoefficients {
            public double kP, kI, kD;
        }

        public interface FeedforwardFun {
            double compute(double position, @Nullable Double velocity);
        }

        private static PIDCoefficients pid;
        private static double kV, kA, kStatic;
        public static FeedforwardFun kF;

        private static double errorSum;
        private static long lastUpdateTs;

        private static boolean inputBounded;
        private static double minInput, maxInput;

        private static boolean outputBounded;
        private static double minOutput, maxOutput;

        /**
         * Target position (that is, the controller setpoint).
         */
        public static double targetPosition;

        /**
         * Target velocity.
         */
        public static double targetVelocity;

        /**
         * Target acceleration.
         */
        public static double targetAcceleration;

        /**
         * Error computed in the last call to {@link #update(long, double, Double)}
         */
        public static double lastError;

        /**
         * Feedforward parameters {@code kV}, {@code kA}, and {@code kStatic} correspond with a basic
         * kinematic model of DC motors. The general function {@code kF} computes a custom feedforward
         * term for other plants.
         *
         * @param pid traditional PID coefficients
         * @param kV feedforward velocity gain
         * @param kA feedforward acceleration gain
         * @param kStatic additive feedforward constant
         * @param kF custom feedforward that depends on position and velocity
         */
        public ArmController(
                PIDCoefficients pid,
                double kV,
                double kA,
                double kStatic,
                FeedforwardFun kF
        ) {
            this.pid = pid;
            this.kV = kV;
            this.kA = kA;
            this.kStatic = kStatic;
            this.kF = kF;
        }

        public ArmController(
                PIDCoefficients pid,
                double kV,
                double kA,
                double kStatic
        ) {
            this(pid, kV, kA, kStatic, (x, v) -> 0);
        }

        public ArmController(
                PIDCoefficients pid,
                FeedforwardFun kF
        ) {
            this(pid, 0, 0, 0, kF);
        }

        public ArmController(
                PIDCoefficients pid
        ) {
            this(pid, 0, 0, 0);
        }



        /**
         * Sets bound on the input of the controller. When computing the error, the min and max are
         * treated as the same value. (Imagine taking the segment of the real line between min and max
         * and attaching the endpoints.)
         *
         * @param min minimum input
         * @param max maximum input
         */
        public static void setInputBounds(double min, double max) {
            if (min < max) {
                inputBounded = true;
                minInput = min;
                maxInput = max;
            }
        }

        /**
         * Sets bounds on the output of the controller.
         *
         * @param min minimum output
         * @param max maximum output
         */
        public static void setOutputBounds(double min, double max) {
            if (min < max) {
                outputBounded = true;
                minOutput = min;
                maxOutput = max;
            }
        }

        public static double getPositionError(double measuredPosition) {
            double error = targetPosition - measuredPosition;
            if (inputBounded) {
                final double inputRange = maxInput - minInput;
                while (Math.abs(error) > inputRange / 2.0) {
                    error -= Math.copySign(inputRange, error);
                }
            }
            return error;
        }

        /**
         * Run a single iteration of the controller.
         *
         * @param timestamp measurement timestamp as given by {@link System#nanoTime()}
         * @param measuredPosition measured position (feedback)
         * @param measuredVelocity measured velocity
         */
        public static double update(
                long timestamp,
                double measuredPosition,
                @Nullable Double measuredVelocity
        ) {
            final double error = getPositionError(measuredPosition);

            if (lastUpdateTs == 0) {
                lastError = error;
                lastUpdateTs = timestamp;
                return 0;
            }

            final double dt = timestamp - lastUpdateTs;
            errorSum += 0.5 * (error + lastError) * dt;
            final double errorDeriv = (error - lastError) / dt;

            lastError = error;
            lastUpdateTs = timestamp;

            double velError;
            if (measuredVelocity == null) {
                velError = errorDeriv;
            } else {
                velError = targetVelocity - measuredVelocity;
            }

            double baseOutput = pid.kP * error + pid.kI * errorSum + pid.kD * velError +
                    kV * targetVelocity + kA * targetAcceleration +
                    kF.compute(measuredPosition, measuredVelocity);

            double output = 0;
            if (Math.abs(baseOutput) > 1e-6) {
                output = baseOutput + Math.copySign(kStatic, baseOutput);
            }

            if (outputBounded) {
                return Math.max(minOutput, Math.min(output, maxOutput));
            }

            return output;
        }

        public static double  update(
                long timestamp,
                double measuredPosition
        ) {
            return update(timestamp, measuredPosition, null);
        }

        public static double update(
                double measuredPosition
        ) {
            return update(System.nanoTime(), measuredPosition, null);
        }

        /**
         * Reset the controller's integral sum.
         */
        public static void reset() {
            errorSum = 0;
            lastError = 0;
            lastUpdateTs = 0;
        }


    }