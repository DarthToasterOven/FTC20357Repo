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

public class RedCloseAutoMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(100, 100, Math.PI, Math.PI, 14.25)
                .build();
        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(-48, -50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-28,28),Math.toRadians(90), new TranslationalVelConstraint(15.0))

                .strafeTo(new Vector2d(-13,28), new TranslationalVelConstraint(100.0))
                .strafeTo(new Vector2d(-13,49), new TranslationalVelConstraint(100.0))
                .strafeTo(new Vector2d(-2,42), new TranslationalVelConstraint(100.0))
                .strafeTo(new Vector2d(-2,53), new TranslationalVelConstraint(100.0))

                .strafeTo(new Vector2d(-13,13))

                .strafeToLinearHeading(new Vector2d(14,28),Math.toRadians(90))
                .strafeTo(new Vector2d(14,55), new TranslationalVelConstraint(100.0))

                .strafeToLinearHeading(new Vector2d(-13,13), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(35,20),Math.toRadians(90), new TranslationalVelConstraint(100.0))
                .strafeTo(new Vector2d(35,50), new TranslationalVelConstraint(100.0))

                .strafeToLinearHeading(new Vector2d(-13,13),Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(0,30),Math.toRadians(0))
                .build()
        );



                meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)

                //       .addEntity(myThirdBot)

                .start();
    }
}