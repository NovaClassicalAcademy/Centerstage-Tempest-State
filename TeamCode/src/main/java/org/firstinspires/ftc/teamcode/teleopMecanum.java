package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tele-Op")
public class teleopMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl"); //port 3
        DcMotor backLeft = hardwareMap.dcMotor.get("bl"); //port 1
        DcMotor frontRight = hardwareMap.dcMotor.get("fr"); // port 2
        DcMotor backRight = hardwareMap.dcMotor.get("br"); //port 0
        DcMotor lift1 = hardwareMap.dcMotor.get("liftLeft");
        DcMotor lift2 = hardwareMap.dcMotor.get("liftRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double lift1power = gamepad2.left_stick_y;
            double lift2power = gamepad2.right_stick_y;
            double denominator = 1.5;
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            lift1.setPower(lift1power);
            lift2.setPower(lift2power);
            telemetry.update();
        }
    }
}

