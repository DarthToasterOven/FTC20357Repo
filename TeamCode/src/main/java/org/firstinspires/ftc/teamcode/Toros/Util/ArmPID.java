package org.firstinspires.ftc.teamcode.Toros.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class ArmPID extends LinearOpMode {
    private PIDController controller;
    private PIDController controller2;



    public static double p1 = 0.006, i1 = 0.001, d1 = 0.00005;

    public static double f1 = 0.004;

    public static int target1 = 425;
    public static double p2 = 0.0095, i2 = 0.04, d2 = 0.00008;

    public static double f2 = 0.004;

    public static int target2 = 250;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx pivot, slideLeft,slideRight;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p1,i1,d1);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p1,i1,d1);
            int slidePos = slideLeft.getCurrentPosition();
            double pid = controller.calculate(slidePos, target1);
            double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

            double slidePower = pid + ff;

            slideLeft.setPower(slidePower);

            slideRight.setPower(slidePower);

            controller2.setPID(p2,i2,d2);
            int armPos = pivot.getCurrentPosition();
            double pid2 = controller2.calculate(armPos, target2);
            double ff2 = Math.cos(Math.toRadians(target2/ticks_in_degrees)) * f2;

            double pivotPower = pid2 + ff2;

            pivot.setPower(pivotPower);


            telemetry.addData("Slide Left Pos", slidePos);
            telemetry.addData("Slide Right Pos", slideRight.getCurrentPosition());
            telemetry.addData("Slide Target", target1);
            telemetry.addData("Pivot pos", armPos);
            telemetry.addData("Pivot target",target2);
            telemetry.update();

        }
    }
}
