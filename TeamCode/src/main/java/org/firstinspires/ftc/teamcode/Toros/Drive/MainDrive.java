package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Toros.Autonomous.AprilTagsys;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.BatteryClass;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {
    // In our drive class we broke it down into subsystems to make it easier to read
    // All that needs to be done in the code is construct the subsystems and run their systems with a method
    DriveTrain drivetrain;
    IntakeV2 intake;
    public static double p1 = 0.009, i1 = 0.45, d1 = 0;

    public static double f1 = 0;
    private PIDController controller;

    public static int targetVel = -1610;
    @Override
    public void runOpMode() throws InterruptedException {
        //Constructs the systems and makes them objects allowing to use a method to run the system and allows for other methods to be used
        drivetrain = new DriveTrain(hardwareMap,gamepad1);
        intake = new IntakeV2(hardwareMap, gamepad1);
        controller = new PIDController(p1,i1,d1);


        waitForStart();
        // runs all of the systems
        while (opModeIsActive()){
            drivetrain.drive();
            intake.runIntake();
            initTelemetry();
        }
    }
    //Telemetry which is good for debugging and seeing how we preform
    private void initTelemetry () {
        telemetry.addData("Toggle",drivetrain.getXToggle());
        telemetry.addData("Toggle",drivetrain.getRToggle());
        telemetry.addData("launcher vel", intake.getLauncherSpeed());
        telemetry.addData("gamepad trigger0",gamepad1.left_trigger);
        telemetry.update();
    }
}
