package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();
        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();
        RoadRunnerBotEntity myFourthBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(-61.25, -37.5, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(-34,-12),Math.toRadians(235))
                        .waitSeconds(2.5)
                        .strafeToLinearHeading(new Vector2d(-11.5,-28),Math.toRadians(90))
                        .strafeTo(new Vector2d(-11.5,-53))
                        .strafeToLinearHeading(new Vector2d(-34,-12), Math.toRadians(235))
                        .waitSeconds(2.5)
                        .strafeToLinearHeading(new Vector2d(12.25,-28),Math.toRadians(90))
                        .strafeTo(new Vector2d(12.25,-53))
                        .strafeToLinearHeading(new Vector2d(-34,-12), Math.toRadians(235))
                         .waitSeconds(2.5)
                        .strafeToLinearHeading(new Vector2d(35.25,-12),Math.toRadians(90))
                        .strafeTo(new Vector2d(35.25,-53))
                .build());
        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(-61.25, 37.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-34,12),Math.toRadians(-235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-11.5,28),Math.toRadians(-90))
                .strafeTo(new Vector2d(-11.5,53))
                .strafeToLinearHeading(new Vector2d(-34,12), Math.toRadians(-235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(12.25,28),Math.toRadians(-90))
                .strafeTo(new Vector2d(12.25,53))
                .strafeToLinearHeading(new Vector2d(-34,12), Math.toRadians(-235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(35.25,12),Math.toRadians(-90))
                .strafeTo(new Vector2d(35.25,53))
                .build());
        myThirdBot.runAction(myThirdBot.getDrive().actionBuilder(new Pose2d(61.25,-12,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-34,-12),Math.toRadians(235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-11.5,-28),Math.toRadians(90))
                .strafeTo(new Vector2d(-11.5,-53))
                .strafeToLinearHeading(new Vector2d(-34,-12), Math.toRadians(235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(12.25,-28),Math.toRadians(90))
                .strafeTo(new Vector2d(12.25,-53))
                .strafeToLinearHeading(new Vector2d(-34,-12), Math.toRadians(235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(35.25,-12),Math.toRadians(90))
                .strafeTo(new Vector2d(35.25,-53))
                        .build());
        myFourthBot.runAction(myFourthBot.getDrive().actionBuilder(new Pose2d(61.25,12,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-34,12),Math.toRadians(-235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-11.5,28),Math.toRadians(-90))
                .strafeTo(new Vector2d(-11.5,53))
                .strafeToLinearHeading(new Vector2d(-34,12), Math.toRadians(-235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(12.25,28),Math.toRadians(-90))
                .strafeTo(new Vector2d(12.25,53))
                .strafeToLinearHeading(new Vector2d(-34,12), Math.toRadians(-235))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(35.25,12),Math.toRadians(-90))
                .strafeTo(new Vector2d(35.25,53))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .addEntity(myThirdBot)
                .addEntity(myFourthBot)
                .start();
    }
}