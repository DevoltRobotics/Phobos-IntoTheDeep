package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Base_autonomo_chido {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, -58, Math.PI / 2))
                //brazo_specimen
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(0, -41), Math.PI / 2)
                .lineToY(-35)
                //wrist_down && open_claw && arm_down
                .lineToY(-45)

                //FIRST SPECIMEN

                .splineToLinearHeading(new Pose2d(36, -45, Math.PI * 1.5), Math.PI / 2)
                .lineToY(-20)
                .splineToConstantHeading(new Vector2d(45, -10), Math.PI * 1.5)
                .lineToY(-55)
                //close_claw && arm_medium
                .lineToY(-50)
                //extend_arm
                .splineToLinearHeading(new Pose2d (3, -41, Math.PI / 2), Math.PI)
                .setTangent(Math.PI / 2)
                .lineToY(-35)
                //open_claw
                .lineToY(-45)

                //SECOND_SPECIMEN

                .splineToLinearHeading(new Pose2d(36, -45, Math.PI * 1.5), Math.PI / 2)
                .lineToY(-20)
                .splineToConstantHeading(new Vector2d(52, -10), Math.PI * 1.5)
                .lineToY(-55)
                .lineToY(-45)
                .splineToConstantHeading(new Vector2d(45, -55), Math.PI * 1.5)
                //close_claw && arm_medium
                .lineToY(-50)
                //extend_arm
                .splineToLinearHeading(new Pose2d (6, -41, Math.PI / 2), Math.PI)
                .setTangent(Math.PI / 2)
                .lineToY(-35)
                //open_claw
                .lineToY(-45)
                //arm_contract && arm_down
                .splineToConstantHeading(new Vector2d(40, -54), 0)

                //THIRD SPECIMEN

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}