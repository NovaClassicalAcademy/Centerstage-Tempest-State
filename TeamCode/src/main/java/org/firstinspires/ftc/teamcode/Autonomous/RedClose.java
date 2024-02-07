package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BasicPIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous
public class RedClose extends OpMode {
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    public BasicPIDController controller;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, liftL, liftR, intake;
    public Servo axonL, axonR, hammerL, hammerR;
    public CRServo outtake;
    public static int targetPosition = 0;
    public static double p = 0.005, i = 0, d = 0;

    @Override
    public void init() {


        Scalar lower = new Scalar(90, 50, 70); // the lower hsv threshold for Blue
        Scalar upper = new Scalar(128, 255, 255); // the upper hsv threshold for Blue

        double minArea = 200;

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colourMassDetectionProcessor)
                .build();

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
        //hammerL = hardwareMap.get(Servo.class, "hammerL");
        //hammerR = hardwareMap.get(Servo.class, "hammerR");
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
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
    }
    @Override
    public void start() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, 60, Math.toRadians(90)));

        Action TrajectoryLeft = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11.5, -30, Math.toRadians(180)), Math.PI / 2)
                .waitSeconds(1)
                .lineToXSplineHeading(48, Math.toRadians(180))
                .waitSeconds(1)
                .build();
        Action TrajectoryMiddle = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11.5, -35, Math.toRadians(90)), Math.PI / 2)
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.PI / 2)
                .waitSeconds(1)
                .build();
        Action TrajectoryRight = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11.5, -33, Math.toRadians(0)), Math.PI / 2)
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(48, -44, Math.toRadians(180)), Math.PI / 2)
                .waitSeconds(1)
                .build();

        Action trajectoryActionChosen;
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }
        switch (recordedPropPosition) {
            case LEFT:
                Actions.runBlocking(TrajectoryLeft);
                break;
            case MIDDLE:
                Actions.runBlocking(TrajectoryMiddle);
                break;
            case RIGHT:
                Actions.runBlocking(TrajectoryRight);
                break;
        }
    }
    @Override
    public void loop() {
        double liftPower = controller.calculate(liftL.getCurrentPosition(), -targetPosition);

        liftL.setPower(liftPower);
        liftR.setPower(-liftPower);
    }
    @Override
    public void stop() {
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }
}