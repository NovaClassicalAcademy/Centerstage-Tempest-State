package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp

public class auto extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx lift;
    public static int targetPosition = 0;
    public static double p = 0.01, i = 0, d = 0;
    @Override
    public void init() {
        controller = new BasicPIDController(p, i, d);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
    }
    @Override
    public void loop() {
        controller.setP(p);
        double liftPower = controller.calculate(lift.getCurrentPosition(), targetPosition);
        lift.setPower(liftPower);
        telemetry.addData("currentPose", lift.getCurrentPosition());
        telemetry.update();
    }
}
