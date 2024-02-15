package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BasicPIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.List;
@Config
@Autonomous
public class RedFar_Backup extends OpMode {
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, intake;
    public Servo hammerL, hammerR;
    public CRServo outtake;
    public static int lowerVal = 165; //160 before
    public static int upperVal = 180;

    public class Lift {
        private DcMotorEx liftL, liftR;

        public Servo axonL, axonR;

        public Lift(HardwareMap hardwareMap) {
            liftL = hardwareMap.get(DcMotorEx.class, "liftL");
            liftR = hardwareMap.get(DcMotorEx.class, "liftR");
            axonL = hardwareMap.get(Servo.class, "axonL");
            axonR = hardwareMap.get(Servo.class, "axonR");

            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.REVERSE);

            liftR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(0.5);
                    liftR.setPower(0.5);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1100) {
                    return true;
                } else {
                    liftR.setPower(0);
                    liftL.setPower(0);
                    axonL.setPosition(0.82);
                    axonR.setPosition(0.76);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    axonL.setPosition(0.51);
                    axonR.setPosition(0.45);
                    liftL.setPower(-0.3);
                    liftR.setPower(-0.3);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 2) {
                    return true;
                } else {
                    liftL.setPower(0);
                    liftR.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }
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

        //controller = new BasicPIDController(p, i, d);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        hammerL = hardwareMap.get(Servo.class, "hammerL");
        hammerR = hardwareMap.get(Servo.class, "hammerR");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addLine("initialized");
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
        Lift lift = new Lift(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-40, -60, Math.toRadians(90)));
        drive.pose = new Pose2d(-40, -60, Math.toRadians(90));

        Action TrajectoryLeft1 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-45, -10, Math.toRadians(270)), Math.PI/2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -21, Math.toRadians(270)), Math.PI/2)
                .build();
        Action TrajectoryLeft2 = drive.actionBuilder(new Pose2d(-48, -21, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-63.5, -11, Math.toRadians(180)), Math.toRadians(270))
                .build();
        Action TrajectoryLeft3 = drive.actionBuilder(new Pose2d(-63.5, -11, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -11), 0)
                .splineToConstantHeading(new Vector2d(44, -29), 0) //-52
                .build();
        Action TrajectoryLeft4 = drive.actionBuilder(new Pose2d(44, -29, Math.PI))
                .splineToConstantHeading(new Vector2d(30, -6), Math.PI)
                .splineToConstantHeading(new Vector2d(-67.5, -7) , Math.PI)
                .build();
        Action TrajectoryLeft5 = drive.actionBuilder(new Pose2d(-67.5, -7, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(28, -6), 0)
                .splineToConstantHeading(new Vector2d(45, -30), 0) //-52
                .build();

        Action TrajectoryMiddle1 = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(-47, -24, Math.toRadians(0)), Math.PI/2)
                .build();
        Action TrajectoryMiddle2 = drive.actionBuilder(new Pose2d(-47, -24, Math.toRadians(0)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-63.5, -11, Math.toRadians(180)), Math.PI/2)
                .build();
        Action TrajectoryMiddle3 = drive.actionBuilder(new Pose2d(-63.5, -11, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -11), 0)
                .splineToConstantHeading(new Vector2d(44, -29), 0) //-52
                .build();
        Action TrajectoryMiddle4 = drive.actionBuilder(new Pose2d(44, -29, Math.PI))
                .splineToConstantHeading(new Vector2d(30, -6), Math.PI)
                .splineToConstantHeading(new Vector2d(-67.5, -7) , Math.PI)
                .build();
        Action TrajectoryMiddle5 = drive.actionBuilder(new Pose2d(-67.5, -7, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(28, -6), 0)
                .splineToConstantHeading(new Vector2d(45, -30), 0) //-52
                .build();

        Action TrajectoryRight1 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-33, -33, Math.toRadians(0)), Math.PI/2)
                .build();
        Action TrajectoryRight2 = drive.actionBuilder(new Pose2d(-33, -33, Math.toRadians(0)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -11, Math.toRadians(180)), Math.PI)
                .splineToSplineHeading(new Pose2d(-63.5, -11, Math.toRadians(180)), 0)
                .build();
        Action TrajectoryRight3 = drive.actionBuilder(new Pose2d(-63.5, -11, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -11), 0)
                .splineToConstantHeading(new Vector2d(44, -29), 0) //-52
                .build();
        Action TrajectoryRight4 = drive.actionBuilder(new Pose2d(44, -29, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(30, -6), Math.PI)
                .splineToConstantHeading(new Vector2d(-67.5, -7) , Math.PI)
                .build();
        Action TrajectoryRight5 = drive.actionBuilder(new Pose2d(-67.5, -7, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(28, -6), 0)
                .splineToConstantHeading(new Vector2d(45, -30), 0) //-52
                .build();

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
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        TrajectoryLeft2
                );

                intake.setPower(1);
                outtake.setPower(1);

                hammerL.setPosition(0.7);
                hammerR.setPosition(0.1);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryLeft3,
                                lift.liftUp()
                        )
                );

                outtake.setPower(-1);
                intake.setPower(0);
                hammerL.setPosition(0.1);
                hammerR.setPosition(0.7);

                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtake.setPower(0);

                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftDown()
                        )
                );

                break;
            case MIDDLE:
                Actions.runBlocking(TrajectoryMiddle1);

                intake.setPower(-0.8);

                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        TrajectoryMiddle2
                );

                intake.setPower(1);
                outtake.setPower(1);

                hammerL.setPosition(0.7);
                hammerR.setPosition(0.1);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryMiddle3,
                                lift.liftUp()
                        )
                );

                outtake.setPower(-1);
                intake.setPower(0);
                hammerL.setPosition(0.1);
                hammerR.setPosition(0.7);

                try {
                    Thread.sleep(1250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                outtake.setPower(0);

                Actions.runBlocking(
                                lift.liftDown()
                );


                break;

            case RIGHT:
                Actions.runBlocking(TrajectoryRight1);

                intake.setPower(-0.8);

                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        TrajectoryRight2
                );

                intake.setPower(1);
                outtake.setPower(1);

                hammerL.setPosition(0.7);
                hammerR.setPosition(0.1);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryRight3,
                                lift.liftUp()
                        )
                );

                outtake.setPower(-1);
                intake.setPower(0);
                hammerL.setPosition(0.1);
                hammerR.setPosition(0.7);

                try {
                    Thread.sleep(1250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                outtake.setPower(0);

                Actions.runBlocking(

                                lift.liftDown()
                );

                break;
        }
    }
    @Override
    public void loop() {
    }
    @Override
    public void stop() {
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }
}