package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(-46, -48, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270)) // Preload

                .strafeToLinearHeading(new Vector2d(-12,-28),Math.toRadians(270)) // first spike
                .strafeTo(new Vector2d(-12,-53)) // first spike intake
                .strafeToLinearHeading(new Vector2d(-12,-13), Math.toRadians(270)) //launch first spike
                        .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(12,-28,Math.toRadians(270)),Math.toRadians(270))// seconds spike
                .strafeTo(new Vector2d(12,-53))//second spike intake
                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-13,-12,Math.toRadians(270)),Math.toRadians(90))
                .build());
        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(-46, 48, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(90)) // Preload

                .strafeToLinearHeading(new Vector2d(-12,28),Math.toRadians(90)) // first spike
                .strafeTo(new Vector2d(-12,53)) // first spike intake
                .strafeToLinearHeading(new Vector2d(-12,13), Math.toRadians(90)) //launch first spike
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(12,28,Math.toRadians(90)),Math.toRadians(90))// seconds spike
                .strafeTo(new Vector2d(12,53))//second spike intake
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-13,12,Math.toRadians(90)),Math.toRadians(270))

                .build());
        myThirdBot.runAction(myThirdBot.getDrive().actionBuilder(new Pose2d(61.25,-12,Math.toRadians(270)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12,-38),Math.toRadians(270))
                .strafeTo(new Vector2d(12,-52))
                .strafeTo(new Vector2d(10,-56))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61.25,-12),Math.toRadians(350))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(35,-57),Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61.25,-12),Math.toRadians(350))

                        .build());
        myFourthBot.runAction(myFourthBot.getDrive().actionBuilder(new Pose2d(61.25,12,Math.toRadians(90)))

                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12,38),Math.toRadians(90))
                .strafeTo(new Vector2d(12,52))
                .strafeTo(new Vector2d(10,56))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(61.25,12),Math.toRadians(10))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(35,57),Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(61.25,12),Math.toRadians(10))




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