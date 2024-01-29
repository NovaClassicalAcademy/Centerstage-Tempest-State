package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Config
@TeleOp (name = "Tele-Op", group = "alpha")

public class Teleop extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, liftL, liftR;

    public Servo axonL, axonR;

    public CRServo outtake;
    public static int targetPosition = 0;
    public static double p = 0.004, i = 0, d = 0;
    public double driveMultiplier = 1;
    private IMU imu;

    @Override
    public void init() {

        controller = new BasicPIDController(p, i, d);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        axonL = hardwareMap.get(Servo.class, "axonL");
        axonR = hardwareMap.get(Servo.class, "axonR");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

    }
    @Override
    public void loop() {

        controller.setP(p);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        if(gamepad1.left_trigger > 0.1){
            driveMultiplier = 0.4;
        }else {
            driveMultiplier = 1;
        }

        double frontLeftPower = (y + x + rx) * driveMultiplier;
        double backLeftPower = (y - x + rx) * driveMultiplier;
        double frontRightPower = (y - x - rx) * driveMultiplier;
        double backRightPower = (y + x - rx) * driveMultiplier;

        /*     Field Centric
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double frontLeftPower = (rotY + rotX + rx) * driveMultiplier;
        double backLeftPower = (rotY - rotX + rx) * driveMultiplier;
        double frontRightPower = (rotY - rotX - rx) * driveMultiplier;
        double backRightPower = (rotY + rotX - rx) * driveMultiplier;
        */

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        if(gamepad2.dpad_up) {
            targetPosition += 10;
        }

        if(gamepad2.dpad_down) {
            targetPosition -= 10;
        }

        if(gamepad2.triangle) {
            axonL.setPosition(0.5);
            axonR.setPosition(0.44);
        } else if(gamepad2.square){
            axonL.setPosition(0.77);
            axonR.setPosition(0.71);
        }

        if(gamepad2.left_trigger > 0.1){
            outtake.setPower(0.3);
        } else if(gamepad2.right_trigger > 0.1){
            outtake.setPower(-0.3);
        } else{
            outtake.setPower(0);
        }

        double liftPower = controller.calculate(liftL.getCurrentPosition(), -targetPosition);

        liftL.setPower(liftPower);
        liftR.setPower(-liftPower);

        telemetry.addData("currentPose", liftL.getCurrentPosition());
        telemetry.addData("currentPose", liftR.getCurrentPosition());
        telemetry.update();
    }
}
