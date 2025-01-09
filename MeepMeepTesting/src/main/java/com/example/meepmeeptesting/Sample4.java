package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Sample4 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(270), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -60, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-38, -57), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-57, -57), Math.toRadians(225))

                //FIRST_PUT

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-35, -25, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(170))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(225)), Math.toRadians(250))

                //SECOND_PUT

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-46, -25, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(170))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(225)), Math.toRadians(250))

                .strafeToLinearHeading(new Vector2d(-56, -25), Math.toRadians(180))
                .setTangent(Math.toRadians(170))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(225)), Math.toRadians(250))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24, -10, Math.toRadians(0)), Math.toRadians(340))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(250))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24, -10, Math.toRadians(0)), Math.toRadians(340))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(250))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}