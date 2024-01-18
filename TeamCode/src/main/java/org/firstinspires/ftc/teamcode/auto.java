package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp

public class auto extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx lift1, lift2;
    public static int targetPosition = 0;
    public static double p = 0.01, i = 0, d = 0;
    @Override
    public void init() {
        controller = new BasicPIDController(p, i, d);
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
    }
    @Override
    public void loop() {
        controller.setP(p);
        double liftPower1 = controller.calculate(lift1.getCurrentPosition(), targetPosition);
        double liftPower2 = controller.calculate(lift2.getCurrentPosition(), targetPosition);
        lift1.setPower(liftPower1);
        lift2.setPower(liftPower2);
        telemetry.addData("currentPose", lift1.getCurrentPosition());
        telemetry.addData("currentPose", lift2.getCurrentPosition());
        telemetry.update();
    }
}
