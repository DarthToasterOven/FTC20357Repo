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

public class LauncherPidF extends LinearOpMode {
    private PIDController controller;


    public static double p1 = 0, i1 = 0, d1 = 0;
    public static double kS = 0,kV = 0,kA = 0;
    public static int target1;

    private DcMotorEx launcher;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p1,i1,d1);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS,kV,kA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){

            controller.setPID(p1,i1,d1);
            double turretVel = launcher.getVelocity();
            double pid2 = controller.calculate(turretVel, target1);
            double ff = feedforward.calculate(target1);

            launcher.setPower(pid2 + ff);

            telemetry.addData("launcher vel", turretVel);
            telemetry.addData("launcher target",target1);
            telemetry.update();

        }
    }
}
