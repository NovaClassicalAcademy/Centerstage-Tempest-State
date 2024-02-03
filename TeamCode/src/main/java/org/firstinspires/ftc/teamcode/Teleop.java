package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
@Config
@TeleOp (name = "Tele-Op", group = "alpha")

public class Teleop extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, liftL, liftR, intake;
    public Servo axonL, axonR, hammerL, hammerR;
    public CRServo outtake;
    public static int targetPosition = 0;
    public static double p = 0.006, i = 0, d = 0;
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
        axonL = hardwareMap.get(Servo.class, "axonL");
        axonR = hardwareMap.get(Servo.class, "axonR");
        hammerL = hardwareMap.get(Servo.class, "hammerL");
        hammerR = hardwareMap.get(Servo.class, "hammerR");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        axonL.setPosition(0.51);
        axonR.setPosition(0.45);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if(gamepad1.left_trigger > 0.1){
            driveMultiplier = 0.6;
        }else if(gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1){
            driveMultiplier = 0.3;
        }else {
            driveMultiplier = 1;
        }
        double frontLeftPower = (y + x + rx) * driveMultiplier;
        double backLeftPower = (y - x + rx) * driveMultiplier;
        double frontRightPower = (y - x - rx) * driveMultiplier;
        double backRightPower = (y + x - rx) * driveMultiplier;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        if(gamepad2.dpad_up) {
            targetPosition += 14;
            if(targetPosition >= 2100){             //top lift regulator
                targetPosition = 2100;
            }
        } else if(gamepad2.dpad_down){
            targetPosition -= 14;
            if(targetPosition <= 0){                //bottom lift regulator
                targetPosition = 0;
            }
        }
        if(targetPosition <= 900){                  //outtake zero position
            axonL.setPosition(0.51);
            axonR.setPosition(0.45);
        } else{                                     //scoring pos
            if (!gamepad2.share) {
                axonL.setPosition(0.77);
                axonR.setPosition(0.71);
            }else{
                axonL.setPosition(0.5);
                axonR.setPosition(0.44);
            }
        }
        if(gamepad2.left_trigger >= 0.3){            //intake
            outtake.setPower(-0.8);
            intake.setPower(-1);
        }else if(gamepad2.right_trigger >= 0.3){    //outtake
            outtake.setPower(0.8);
            intake.setPower(1);
        }else{
            outtake.setPower(0);                    //idle
            intake.setPower(0);
        }
        if(gamepad1.left_bumper){                   //hammer zero
            hammerL.setPosition(0.79);
            hammerR.setPosition(0.23);
        }else if(gamepad1.right_bumper){           //hammer out
            hammerL.setPosition(0.4);
            hammerR.setPosition(0.8);
        }
        double liftPower = controller.calculate(liftL.getCurrentPosition(), -targetPosition);

        liftL.setPower(liftPower);
        liftR.setPower(-liftPower);
    }
}
