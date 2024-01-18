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

                                .splineToSplineHeading(new Pose2d(34.4,-35.6, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(33.6,-56.8, Math.toRadians(-90)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(66.4,-40.4, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(61.2,-28.8, Math.toRadians(93)), Math.toRadians(93))
                                .splineToSplineHeading(new Pose2d(-35.6,-57.6, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-62,-20, Math.toRadians(-90)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-64.4,-29.6, Math.toRadians(-90)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(33.6,-51.2, Math.toRadians(-90)), Math.toRadians(-90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}