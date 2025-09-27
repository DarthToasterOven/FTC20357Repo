package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        //Constructs the systems and makes them objects allowing to use a method to run the system and allows for other methods to be used
        drivetrain = new DriveTrain(hardwareMap,gamepad1);
        IntakeV2 intake = new IntakeV2(hardwareMap, gamepad2);
        Launcher launcher = new Launcher(hardwareMap, gamepad2);
        waitForStart();
        // runs all of the systems
        while (opModeIsActive()){
            drivetrain.drive();
            intake.runIntake();
            launcher.runLauncher();
            initTelemetry();
        }
    }
    //Telemetry which is good for debugging and seeing how we preform
    private void initTelemetry () {
        telemetry.addData("Toggle",drivetrain.getXToggle());
        telemetry.addData("Toggle",drivetrain.getYToggle());
        telemetry.update();
    }
}
