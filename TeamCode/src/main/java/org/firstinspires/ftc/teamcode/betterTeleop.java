package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
@TeleOp

public class betterTeleop extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx liftL, liftR;
    public int targetPosition = 0;
    public double p = 0.02, i = 0, d = 0.0001;
    public double driveMultiplier = 1;
    @Override
    public void init() {

        controller = new BasicPIDController(p, i, d);
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        /*
        test code for slow mode
        if(gamepad1.left_trigger > 0.1){
            driveMultiplier = 0.5;
        }

        else {
            driveMultiplier = 1;
        }
        */

        double frontLeftPower = (y + x + rx) * driveMultiplier;
        double backLeftPower = (y - x + rx) * driveMultiplier;
        double frontRightPower = (y - x - rx) * driveMultiplier;
        double backRightPower = (y + x - rx) * driveMultiplier;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        controller.setP(p);

        if(gamepad1.dpad_up) {
            targetPosition = 1500;
        }

        if(gamepad1.dpad_down) {
            targetPosition = 5;
        }

        double liftPowerL = controller.calculate(liftL.getCurrentPosition(), -targetPosition);

        liftL.setPower(liftPowerL);
        liftR.setPower(-liftPowerL);

        telemetry.addData("currentPose", liftL.getCurrentPosition());
        telemetry.addData("currentPose", liftR.getCurrentPosition());
        telemetry.update();
    }
}
