package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
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
@Config
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
    public static int lowerVal = 165; //160 before
    public static int upperVal = 180;

    @Override
    public void init() {

        Scalar lower = new Scalar(lowerVal,50,50); // the lower hsv threshold for Red
        Scalar upper = new Scalar(upperVal,255,255); // the upper hsv threshold for Red

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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, -60, Math.toRadians(90)));

        Action TrajectoryLeft1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(180)), Math.PI / 2)
                .waitSeconds(0.2)
                .build();
        //lift up and axons move to scoring pose
        Action TrajectoryLeft2 = drive.actionBuilder(drive.pose)
                .lineToXSplineHeading(48, Math.toRadians(180))
                .build();
        //score
        //bring arm back down
        /*
        Action TrajectoryLeft3 = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(11.5, -58), Math.PI)
                .splineToConstantHeading(new Vector2d(-40, -45), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-56, -36), Math.PI)
                .build();
        //intake pixels
        Action TrajectoryLeft4 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -58), 0)
                .splineToConstantHeading(new Vector2d(11.5, -58), 0)
                .splineToConstantHeading(new Vector2d(48, -30), 0)
                .build();
        //score pixels
        Action TrajectoryMiddle1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11.5, -33, Math.toRadians(90)), Math.PI / 2)
                .build();
        Action TrajectoryMiddle2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(48, -33, Math.toRadians(180)), Math.PI / 2)
                .build();
        Action TrajectoryRight1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(31, -33, Math.toRadians(180)), Math.PI / 2)
                .build();
        Action TrajectoryRight2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49, -42), 0)
                .build();
*/
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
                Actions.runBlocking(TrajectoryLeft1);
                intake.setPower(-0.8);

                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                intake.setPower(0);
                targetPosition = 1000;
                axonL.setPosition(0.77);
                axonR.setPosition(0.71);

                Actions.runBlocking(TrajectoryLeft2);

                outtake.setPower(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtake.setPower(0);

                axonL.setPosition(0.51);
                axonR.setPosition(0.45);
                targetPosition = 0;

                //Actions.runBlocking(TrajectoryLeft3);
                //Actions.runBlocking(TrajectoryLeft4);

                break;
            case MIDDLE:
                /*Actions.runBlocking(TrajectoryMiddle1);
                intake.setPower(-0.8);

                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                intake.setPower(0);
                targetPosition = 1000;
                axonL.setPosition(0.77);
                axonR.setPosition(0.71);
*/
                /*Actions.runBlocking(TrajectoryMiddle2);

                outtake.setPower(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtake.setPower(0);

                axonL.setPosition(0.51);
                axonR.setPosition(0.45);
                targetPosition = 0;
                //Actions.runBlocking(TrajectoryLeft3);
                break;
                */

            case RIGHT:
                /*Actions.runBlocking(TrajectoryRight1);
                intake.setPower(-0.8);

                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                intake.setPower(0);
                targetPosition = 1000;
                axonL.setPosition(0.77);
                axonR.setPosition(0.71);

                /*Actions.runBlocking(TrajectoryRight2);

                outtake.setPower(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtake.setPower(0);

                axonL.setPosition(0.51);
                axonR.setPosition(0.45);
                targetPosition = 0;
                //Actions.runBlocking(TrajectoryLeft3);
                break;
                 */

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