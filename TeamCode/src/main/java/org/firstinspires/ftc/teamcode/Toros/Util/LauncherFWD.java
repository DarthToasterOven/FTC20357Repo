package org.firstinspires.ftc.teamcode.Toros.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class LauncherFWD extends LinearOpMode {
    private SimpleMotorFeedforward controller;
    public static double kV = 0.95, kS=5, kA=-1    ;
    public static int accel = 10;
    public static int targetVel = -600;
    private DcMotorEx launcher;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcher = hardwareMap.get(DcMotorEx.class,"launch");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()){
            controller = new SimpleMotorFeedforward(kS,kV,kA);
            double launchVel = launcher.getVelocity();
            double ffwd = controller.calculate(targetVel, accel);
            //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
            double power = ffwd;
//
            launcher.setVelocity(power);
//
//


//            telemetry.addData("Slide Left P/os", slidePos);
//            telemetry.addData("Slide Right Pos", slideRight.getCurrentPosition());
//            telemetry.addData("Slide Target", target1);
            telemetry.addData("max",controller.maxAchievableVelocity(12,20));
            telemetry.addData("ffwd",ffwd);
            telemetry.addData("launcher velocity", launchVel);
            telemetry.addData("V target", targetVel);
            telemetry.update();

        }
    }
}
