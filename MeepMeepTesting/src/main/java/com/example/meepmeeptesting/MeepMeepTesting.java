package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -60, Math.toRadians(90)))
                .lineToY(-57)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(31, -33, Math.toRadians(180)), Math.PI / 2)
                .waitSeconds(1)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49, -42), 0)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(11.5, -58), Math.PI)
                .splineToConstantHeading(new Vector2d(-40, -45), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-56, -36), Math.PI)


                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}