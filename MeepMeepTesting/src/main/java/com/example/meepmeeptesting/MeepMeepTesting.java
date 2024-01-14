package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, -62, Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(11.5, -30, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(180)))
                                //reposition with tag
                                //.lineToLinearHeading(new Pose2d(0, -12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-58, -12), Math.toRadians(180))
                                //reposition with tag
                                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(180))
                                //reposition with tag
                                .splineToConstantHeading(new Vector2d(-58, -12), Math.toRadians(180))
                                //reposition with tag
                                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(180))
                                //reposition with tag
                                .splineToConstantHeading(new Vector2d(-58, -12), Math.toRadians(180))
                                //reposition with tag
                                .splineToConstantHeading(new Vector2d(48, -30), Math.toRadians(180))
                                //reposition with tag

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}