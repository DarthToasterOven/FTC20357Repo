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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
@TeleOp

public class TurretPidF extends LinearOpMode {
    private PIDController controller;


    public static double p1 = 0, i1 = 0, d1 = 0;
    public static double kS = 0,kV = 0,kA = 0;
    public static int target1;

    private DcMotorEx turretMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p1,i1,d1);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS,kV,kA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){

            controller.setPID(p1,i1,d1);
            double turretVel = turretMotor.getVelocity();
            double pid2 = controller.calculate(turretVel, target1);
            double ff = feedforward.calculate(target1);

            turretMotor.setPower(pid2 + ff);

            telemetry.addData("turret vel", turretVel);
            telemetry.addData("turret target",target1);
            telemetry.update();

        }
    }
}
