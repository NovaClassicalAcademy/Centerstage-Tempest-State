package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.tuning.BasicPIDController;

public class auto extends OpMode {
    public BasicPIDController controller;
    private DcMotorEx lift;
    public static int targetPosition = 0;
    @Override
    public void init() {
        controller = new BasicPIDController(0, 0, 0); //tune pid
        lift = hardwareMap.get(DcMotorEx.class, "lift");
    }
    @Override
    public void loop() {
        double liftPower = controller.calculate(lift.getCurrentPosition(), targetPosition);
        lift.setPower(liftPower);
    }
}
