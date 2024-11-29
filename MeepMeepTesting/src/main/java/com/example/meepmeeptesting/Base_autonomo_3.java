package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Base_autonomo_3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(27, -60, Math.toRadians(270)))
                //arm && wrist _specimen
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(10, -33), Math.toRadians(270))
                //extend rode
                .strafeToLinearHeading(new Vector2d(10, -35), Math.toRadians(270))
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //arm_down && wrist_medium

                //FIRST_SPECIMEN

                .strafeToLinearHeading(new Vector2d(32, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270))
                //close_claw
                .strafeToLinearHeading(new Vector2d(5, -33), Math.toRadians(270))
                //arm && wrist_specimen
                .strafeToLinearHeading(new Vector2d(5, -35), Math.toRadians(270))
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //SECOND_SPECIMEN

                .strafeToLinearHeading(new Vector2d(32, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270))
                //close_claw
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(270))
                //arm && wrist_specimen
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(270))
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //THIRD_SPECIMEN

                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}