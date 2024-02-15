package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BasicPIDController;
import org.firstinspires.ftc.vision.VisionPortal;
/*
@Disabled
@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class RedCloseSide extends LinearOpMode {
    public static int lowerVal = 165;
    public static int upperVal = 180;
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    public BasicPIDController controller;
    public static int targetPosition = 0;

    public static double p = 0.005, i = 0, d = 0;

    public class Lift {
        private DcMotorEx liftL, liftR;

        public Servo axonL, axonR;

        public Lift(HardwareMap hardwareMap) {
            liftL = hardwareMap.get(DcMotorEx.class, "liftL");
            liftR = hardwareMap.get(DcMotorEx.class, "liftR");

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
                    liftL.setPower(-0.3);
                    liftR.setPower(0.3);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 900) {
                    return true;
                } else {
                    liftR.setPower(0);
                    liftL.setPower(0);
                    axonL.setPosition(0.77);
                    axonR.setPosition(0.71);
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
                    liftL.setPower(0.3);
                    liftR.setPower(-0.3);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 5) {
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

    public class Intake {
        private DcMotorEx intake;
        public CRServo outtake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            outtake = hardwareMap.get(CRServo.class, "outtake");
        }

        public class IntakeIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(1);
                    outtake.setPower(0.8);
                    initialized = true;
                }
                return false;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(-1);
                    outtake.setPower(-0.65);
                    initialized = true;
                }
                return false;
        }
        public Action intakeOut() {
            return new IntakeOut();
        }
    }
}

    public class Outtake {
        private CRServo outtake;

        public Outtake(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(CRServo.class, "outtake");
        }

        public class OuttakeDeposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPower(0.65);
                return false;
            }
        }
        public Action Deposit() {
            return new OuttakeDeposit();
        }

        public class OuttakeIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPower(-0.65);
                return false;
            }
        }
        public Action DepositIntake() {
            return new OuttakeIntake();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Outtake outtake = new Outtake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake((hardwareMap));

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryLeftAction1;
        Action trajectoryLeftAction2;
        Action trajectoryLeftAction3;
        Action trajectoryLeftAction4;

        trajectoryLeftAction1 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(10.5, -28, Math.toRadians(180)), Math.PI / 2)
                .build();
        trajectoryLeftAction2 = drive.actionBuilder(new Pose2d(10, -22, Math.toRadians(180)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48, -30), Math.PI / 2)
                .build();
        trajectoryLeftAction3 = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(11.5, -52), Math.PI)
                //.splineToConstantHeading(new Vector2d(-40, -45), Math.PI / 2)
                //.splineToConstantHeading(new Vector2d(-56, -36), Math.PI)
                .build();
        trajectoryLeftAction4 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -58), 0)
                .splineToConstantHeading(new Vector2d(11.5, -58), 0)
                .splineToConstantHeading(new Vector2d(48, -30), 0)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(trajectoryLeftAction1);

                        lift.liftUp(),
                        trajectoryLeftAction2
                )
        );
    }
}*/
