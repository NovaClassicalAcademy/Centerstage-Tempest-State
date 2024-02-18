package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
@Config
@TeleOp (name = "Tele-Op", group = "alpha")

public class Teleop extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, liftL, liftR, intake;
    public Servo axonL, axonR, hammerL, hammerR, drone;
    public CRServo outtake;
    public static int targetPosition = 0;
    public static double p = 0.005, i = 0, d = 0;
    public double driveMultiplier = 1;
    public ElapsedTime gametime = new ElapsedTime();

    @Override
    public void init() {

        controller = new BasicPIDController(p, i, d);

        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        axonL = hardwareMap.get(Servo.class, "axonL");
        axonR = hardwareMap.get(Servo.class, "axonR");
        hammerL = hardwareMap.get(Servo.class, "hammerL");
        hammerR = hardwareMap.get(Servo.class, "hammerR");
        drone = hardwareMap.get(Servo.class, "drone");
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

        gametime.reset();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if(gamepad1.left_trigger > 0.1){
            driveMultiplier = 0.4;
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
            targetPosition += 16;
            if(targetPosition >= 2100){             //top lift regulator
                targetPosition = 2100;
            }
        } else if(gamepad2.dpad_down){
            targetPosition -= 16;
            if(targetPosition <= 0){                //bottom lift regulator
                targetPosition = 0;
            }
        }
        if(targetPosition <= 950){                  //outtake zero position
            axonL.setPosition(0.53);
            axonR.setPosition(0.47);
        } else{                                     //scoring pos
            if (!gamepad2.a) {
                axonL.setPosition(0.77);
                axonR.setPosition(0.71);
            }else{
                axonL.setPosition(0.53);
                axonR.setPosition(0.47);
            }
        }
        if(gamepad2.left_trigger >= 0.3){            //intake
            outtake.setPower(-0.65);
            intake.setPower(-1);
        }else if(gamepad2.right_trigger >= 0.3){    //outtake
            outtake.setPower(0.65);
            intake.setPower(1);
        }else{
            outtake.setPower(0);                    //idle
            intake.setPower(0);
        }
        if(gamepad1.left_bumper){                   //hammer zero
            hammerL.setPosition(0.1);
            hammerR.setPosition(0.7);
        }else if(gamepad1.right_bumper){           //hammer out
            hammerL.setPosition(0.7);
            hammerR.setPosition(0.1);
        }
        if(gamepad1.dpad_up){
            drone.setPosition(0.5);
        } else if(gamepad1.dpad_down){
            drone.setPosition(0.2);
        }
        if(!gamepad2.b){
            double liftPower = controller.calculate(liftL.getCurrentPosition(), -targetPosition);
            liftR.setPower(-liftPower);
            liftL.setPower(liftPower);
        } else{
            double liftPower = gamepad2.left_stick_y;
            liftL.setPower(liftPower);
            liftR.setPower(-liftPower);
            if(gamepad2.share){
                liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }


        telemetry.addData("gametime", gametime.time());

        /*// <-- this works but its annoying to have in teleop for now
        if(gametime.time() > 118){
            controller.setP(0.008);
        }
*/

    }
}
