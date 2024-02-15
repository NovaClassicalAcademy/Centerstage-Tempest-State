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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, 60, Math.toRadians(270)))
                //1
                .splineToLinearHeading(new Pose2d(-45, 10, Math.toRadians(90)), Math.PI/2)

                .splineToSplineHeading(new Pose2d(-48, 21, Math.toRadians(90)), Math.PI/2)
                                .waitSeconds(1)
                //2
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-63.5, 11, Math.toRadians(180)), Math.toRadians(270))
                //3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, 11), 0)
                .splineToConstantHeading(new Vector2d(44, 29), 0) //-52
                                .waitSeconds(1)
                //4
                .splineToConstantHeading(new Vector2d(30, 6), Math.PI)
                .splineToConstantHeading(new Vector2d(-67.5, 7) , Math.PI)
                                .waitSeconds(1)
                //5
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(28, 6), 0)
                .splineToConstantHeading(new Vector2d(45, 30), 0) //-52
                                .waitSeconds(1)






                /*

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -58), 0)
                .splineToConstantHeading(new Vector2d(11.5, -58), 0)


                 */
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}