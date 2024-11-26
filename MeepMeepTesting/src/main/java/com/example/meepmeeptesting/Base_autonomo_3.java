package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Base_autonomo_3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(25, -58, Math.PI / 2))
                //brazo_specimen
                .splineToConstantHeading(new Vector2d(0, -41), Math.PI / 2)
                .lineToY(-35)
                //wrist_down && open_claw && contract_arm
                .lineToY(-47)
                //arm_down

                //FIRST_SPECIMEN

                .splineToLinearHeading(new Pose2d(36, -40, Math.PI * 1.5), Math.PI / 2)
                .lineToY(-55)
                //close_claw && arm_medium
                .lineToY(-47)
                .strafeToLinearHeading(new Vector2d(3, -41), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .lineToY(-35)
                //wrist_down && open_claw && contract_arm
                .lineToY(-47)
                //arm_down

                //SECOND_SPECIMEN

                .splineToLinearHeading(new Pose2d(36, -40, Math.PI * 1.5), Math.PI / 2)
                .lineToY(-55)
                //close_claw && arm_medium
                .lineToY(-47)
                .strafeToLinearHeading(new Vector2d(6, -41), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .lineToY(-35)
                //wrist_down && open_claw && contract_arm
                .lineToY(-47)
                //arm_down
                .splineToConstantHeading(new Vector2d(40, -54), 0)

                //THIRD_SPECIMEN


                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}