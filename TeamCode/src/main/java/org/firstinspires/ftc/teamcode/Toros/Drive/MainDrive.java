package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Toros.Autonomous.AprilTagsys;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.BatteryClass;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {
    // In our drive class we broke it down into subsystems to make it easier to read
    // All that needs to be done in the code is construct the subsystems and run their systems with a method
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    DriveTrain drivetrain;
    IntakeV2 intake;
    Turret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        //Constructs the systems and makes them objects allowing to use a method to run the system and allows for other methods to be used
        drivetrain = new DriveTrain(hardwareMap,gamepad1);
        intake = new IntakeV2(hardwareMap, gamepad1);
        turret = new Turret(hardwareMap,gamepad2);
        initAprilTag();
        waitForStart();
        // runs all of the systems
        while (opModeIsActive()){
            drivetrain.drive();
            intake.runIntake();
            intake.runlauncher();
            initTelemetry();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    //Telemetry which is good for debugging and seeing how we preform
    private void initTelemetry () {
        telemetry.addData("Toggle",drivetrain.getXToggle());
        telemetry.addData("Toggle",drivetrain.getRToggle());
        telemetry.addData("launcher vel", intake.getLauncherSpeed());
        telemetry.addData("gamepad trigger0",gamepad1.left_trigger);
        telemetry.update();
    }
    private void lockOn(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection: currentDetections){
            if(detection.metadata != null && gamepad2.yWasPressed()){
                turret.setAngle((int)(turret.getTurretAngle() + detection.ftcPose.yaw));
            }
        }
    }
}
