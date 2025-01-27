package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class autoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(24, -61, Math.toRadians(90)))
                        .strafeTo(new Vector2d(8,-33))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(8,-37))
                        .setTangent(Math.toRadians(350))
                        .splineToLinearHeading(new Pose2d(48,-10,Math.toRadians(270)),Math.toRadians(270))
                        .strafeTo(new Vector2d(48,-52))
                        .strafeTo(new Vector2d(48,-15))
                        .strafeTo(new Vector2d(58,-15))
                        .strafeTo(new Vector2d(58,-52))
                        .strafeTo(new Vector2d(58,-15))
                        .strafeTo(new Vector2d(64,-15))
                        .strafeTo(new Vector2d(64,-52))

//                        .strafeToLinearHeading(new Vector2d(51,-62), Math.toRadians(270))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d(53,-62), Math.toRadians(270))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d(58,-25), Math.toRadians(0))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(new Vector2d(60,-62), Math.toRadians(270))



                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)

                .start();


    }
}

