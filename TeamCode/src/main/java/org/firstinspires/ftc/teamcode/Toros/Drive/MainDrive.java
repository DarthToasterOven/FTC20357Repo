package org.firstinspires.ftc.teamcode.Toros.Drive;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {
    // In our drive class we broke it down into subsystems to make it easier to read
    // All that needs to be done in the code is construct the subsystems and run their systems with a method
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    DriveTrain drivetrain;
    IntakeV2 intake;
    Turret turret;
    public ColorSensor c3;
    List<LynxModule> allHubs;
    private boolean lockedOn = false;
    private boolean driveMode = false;
    @Override
    public void runOpMode() throws InterruptedException {
        //Constructs the systems and makes them objects allowing to use a method to run the system and allows for other methods to be used
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.setMsTransmissionInterval(500);
        drivetrain = new DriveTrain(hardwareMap,gamepad1);
        intake = new IntakeV2(hardwareMap, gamepad1, gamepad2);
        turret = new Turret(hardwareMap,gamepad2);
        initAprilTag();
        waitForStart();
        // runs all of the systems
        while (opModeIsActive()){
            if(driveMode) {
                drivetrain.driveRobotCentric();
            }
            else{
                drivetrain.driveFieldCentric();
            }
            if(gamepad1.bWasPressed()){
                driveMode = !driveMode;
            }
            initTelemetry();
            telemetryAprilTag();
            turret.runTurret();
            lockOn();
            intake.runlauncher();
            intake.runIntake();
            intake.runTrans();

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
                .setLensIntrinsics(909.963833736,909.963833736,634.495942075,347.541434786)

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 720));
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
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Translational brake",drivetrain.getXToggle());
        telemetry.addData("Rotational brake ",drivetrain.getRToggle());
        telemetry.addData("Color sensor red", intake.c3.red());
        telemetry.addData("Color sensor green", intake.c3.green());
        telemetry.addData("Color sensor blue", intake.c3.blue());
        telemetry.addData("Launcher vel", intake.getLauncherSpeed());
        telemetry.addData("Turret angle", turret.getTurretAngle());
        telemetry.addData("Robot heading", drivetrain.getHeading());
        telemetry.update();
    }
    private void lockOn(){

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for(AprilTagDetection detection: currentDetections) {

            if (gamepad2.yWasPressed()) {
                lockedOn = true;
            } else if (gamepad2.bWasPressed()) {
                lockedOn = false;
            }
            if (detection.metadata != null && lockedOn && (detection.id == 24 || detection.id == 23)) {// Checks if there is a detection and that the lockon is active
                if(Math.abs(detection.ftcPose.bearing) < 35 ) {
                    turret.setAngle(turret.getTurretAngle() - detection.ftcPose.bearing * 0.75);
                }

            }
        }
    }
    /*
    public static int[] calcLaunch(){

       // int hoodAngle = (distance); // close = high,   far = 40 degrees
        int[] launchVelocities = new int[2];
        int k = 1;

        int translationalFAST = Distance(Math.sqrt(9.81 / 2*Math.pow(Math.cos(hoodAngle), 2) * (distance*Math.tan(hoodAngle) - h)));
        int translationalSLOW = formula; (hoodAngle: higher)

        launchVelocities[0] = k * (translationalFAST * radius) * (28/2Math.PI);
        launchVelocities[1] = k * (translationalSLOW * radius) * (28/2Math.PI);




        return launchVelocities; //velocity
    }*/
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
///:3