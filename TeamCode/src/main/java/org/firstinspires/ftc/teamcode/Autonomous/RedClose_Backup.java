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
public class RedClose_Backup extends OpMode {
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
                    liftL.setPower(-0.2);
                    liftR.setPower(-0.2);
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

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, -60, Math.toRadians(90)));
        drive.pose = new Pose2d(11.5, -60, Math.toRadians(90));

        Action TrajectoryLeft1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11, -28, Math.toRadians(180)), Math.PI / 2)
                .build();
        //lift up and axons move to scoring pose
        Action TrajectoryLeft2 = drive.actionBuilder(new Pose2d(11, -28, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49, -25), Math.PI / 2)
                .build();
        //score
        //bring arm back down
        Action TrajectoryLeft3 = drive.actionBuilder(new Pose2d(49, -25, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(11.5, -51), Math.PI) //-52
                .splineToConstantHeading(new Vector2d(-40, -51), Math.PI) //-52
                .splineToConstantHeading(new Vector2d(-57, -26.5), Math.PI)
                .lineToX(-60)
                .build();
        //intake pixels
        Action TrajectoryLeft4 = drive.actionBuilder(new Pose2d(-57, -26, Math.PI))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -45), 0)
                .splineToConstantHeading(new Vector2d(11.5, -45), 0)
                .splineToConstantHeading(new Vector2d(48.5, -28), 0)
                .build();

        //score pixels
        Action TrajectoryMiddle1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(11.5, -31, Math.toRadians(90)), Math.PI / 2)
                .build();
        Action TrajectoryMiddle2 = drive.actionBuilder(new Pose2d(11.5, -31, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(49, -33, Math.toRadians(180)), Math.PI / 2)
                .build();
        Action TrajectoryMiddle3 = drive.actionBuilder(new Pose2d(49, -33, Math.toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(11.5, -52), Math.PI)
                .splineToConstantHeading(new Vector2d(-40, -52), Math.PI)
                .splineToConstantHeading(new Vector2d(-57, -27), Math.PI)
                .lineToX(-59)
                .build();
        Action TrajectoryMiddle4 = drive.actionBuilder(new Pose2d(-59, -27, Math.PI))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -48), 0)
                .splineToConstantHeading(new Vector2d(11.5, -48), 0)
                .splineToConstantHeading(new Vector2d(48.5, -34), 0)
                .build();

        Action TrajectoryRight1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(32, -29, Math.toRadians(180)), Math.PI / 2)
                .build();
        Action TrajectoryRight2 = drive.actionBuilder(new Pose2d(32, -29, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49, -42), 0)
                .build();
        Action TrajectoryRight3 = drive.actionBuilder(new Pose2d(49, -42, Math.toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(11.5, -53), Math.PI)
                .splineToConstantHeading(new Vector2d(-40, -53), Math.PI)
                .splineToConstantHeading(new Vector2d(-58, -26.5), Math.PI)
                .lineToX(-60.3)
                .build();
        Action TrajectoryRight4 = drive.actionBuilder(new Pose2d(-60.3, -26.5, Math.PI))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -48), 0)
                .splineToConstantHeading(new Vector2d(11.5, -48), 0)
                .splineToConstantHeading(new Vector2d(48, -22), 0)
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
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftUp(),
                                TrajectoryLeft2
                        )
                );

                intake.setPower(0);
                outtake.setPower(-0.65);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtake.setPower(0);

                Actions.runBlocking(

                                lift.liftDown()
                );

                break;
            case MIDDLE:
                Actions.runBlocking(TrajectoryMiddle1);
                intake.setPower(-0.8);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }



                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftUp(),
                                TrajectoryMiddle2
                        )
                );
                intake.setPower(0);
                outtake.setPower(-0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                outtake.setPower(0);


                        new ParallelAction(
                                lift.liftDown()
                );

                break;

            case RIGHT:
                Actions.runBlocking(TrajectoryRight1);
                intake.setPower(-0.8);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftUp(),
                                TrajectoryRight2
                        )
                );

                outtake.setPower(-0.8);
                intake.setPower(0);

                try {
                    Thread.sleep(1000);
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