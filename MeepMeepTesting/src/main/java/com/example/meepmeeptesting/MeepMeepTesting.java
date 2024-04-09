package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        Pose2d startPose = new Pose2d(-36.75, -62.5, Math.toRadians(0));//11.25-48(two tiles)

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 20, Math.toRadians(120), Math.toRadians(120), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(-38.0, -33, Math.toRadians(90))) //x:18-48(two tiles)-8 (other side of prop)
                                .lineToLinearHeading(new Pose2d(-42.0, -58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
                                .splineToConstantHeading(new Vector2d(0, -58.5), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, -35), Math.toRadians(30))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}