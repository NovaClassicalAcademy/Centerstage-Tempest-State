package org.firstinspires.ftc.teamcode;

public class BasicPIDController {
    private double kP;
    private double kI;
    private double kD;
    private double previousError;
    private double totalError;
    private double velocityError;
    private double lastTimeStamp;
    public BasicPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public double calculate(int measuredPosition, int targetPosition) {
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        int positionError = targetPosition - measuredPosition; //position
        totalError += ((positionError + previousError) / 2) * period; //integral
        velocityError = (positionError - previousError) / period; //derivative
        previousError = positionError;

        return positionError * kP + totalError * kI + velocityError * kD;
    }
    public void setP(double p) {
        this.kP = p;
    }

    public void setI(double i) {
        this.kI = i;
    }
    public void setD(double d) {
        this.kD = d;
    }
}
