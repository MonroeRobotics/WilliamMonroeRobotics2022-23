package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40.5828, 40.5828, 0.6133, 0.6133, 12.95)
                .setDimensions(14.5, 16.0625)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(41, -63.96875, Math.toRadians(270.0)))
                                .lineToConstantHeading(new Vector2d(37, -59))
                                .lineToConstantHeading(new Vector2d(33, -37))
                                .lineToLinearHeading(new Pose2d(10, -35, Math.toRadians(315)))
                                .lineToConstantHeading(new Vector2d(7, -30))
                                .lineToLinearHeading(new Pose2d(10, -35, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(36, -35))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(60, -35))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}