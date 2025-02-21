package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Specimen5 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(270), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(34, -50, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(0, -34))
                .splineToSplineHeading(new Pose2d(0, -22.3, Math.toRadians(270)), Math.toRadians(180))

                /* .strafeToLinearHeading(new Vector2d(-10, -34), Math.toRadians(270))
>>>>>>> d596bb47e69abf9b456e126abca9ef5c11a82b85
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36, -30), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(36, -10))

/*
                .strafeToLinearHeading(new Vector2d(30, -40), Math.toRadians(40))
                        .turnTo(Math.toRadians(-50))

                        .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(40))
                        .turnTo(Math.toRadians(-60))
                        .strafeToLinearHeading(new Vector2d(50, -40), Math.toRadians(40))
                        .turnTo(Math.toRadians(-70))

                //PUT_THREE

                .strafeToLinearHeading(new Vector2d(40, -48), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40, -62), Math.toRadians(270))


                .strafeToLinearHeading(new Vector2d(-6, -34), Math.toRadians(270))

                .setTangent(Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(40, -62, Math.toRadians(270)), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(-2, -34), Math.toRadians(270))
                .setTangent(Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(40, -62, Math.toRadians(270)), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(2, -34), Math.toRadians(270))
                .setTangent(Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(40,   -62, Math.toRadians(270)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(6, -34), Math.toRadians(270))
*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}