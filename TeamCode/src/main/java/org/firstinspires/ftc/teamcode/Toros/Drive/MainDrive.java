package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.BatteryClass;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {
    Slides slides;
    DriveTrain drivetrain;
    Pivot pivot;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new DriveTrain(hardwareMap,gamepad1);
        slides = new Slides(hardwareMap,gamepad2);
        pivot = new Pivot(hardwareMap,gamepad2);
        intake = new Intake(hardwareMap,gamepad2);
        waitForStart();
        while (opModeIsActive()){
            drivetrain.drive();
            slides.runSlides();
            pivot.runPivot();
            intake.runClaw();
            initTelemetry();
        }
    }
    private void initTelemetry () {

        BatteryClass battery = new BatteryClass(hardwareMap);
        telemetry.addData("Slide",slides.getSlidePos());
        telemetry.addData("Battery", battery.getBatteryPercent());
        telemetry.addData("Basket",intake.getToggle());
        telemetry.addData("Garbage",intake.getPower());
        telemetry.addData("Toggle",drivetrain.getXToggle());
        telemetry.addData("Toggle",drivetrain.getYToggle());
        telemetry.update();
    }
}
