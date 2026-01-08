package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoBlueNear")
public class Auto2025BlueNear extends LinearOpMode {
    public DcMotorEx launch, turretMotor, trans;
    public ColorSensor c1,c2,c3;
    private DcMotor intake;
    private PIDController controller;

    public static double p1 = 0.009, i1 = 0.45, d1 = 0;

    public static double p2 = 0.0025, i2 = 0.000001, d2 =0.0001;

    public static int targetVel = -1600;
    public static int targetAngle = 0;
    public class Launcher {
        public Launcher(HardwareMap hardwareMap) {
            launch = hardwareMap.get(DcMotorEx.class, "launch");
            launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            controller = new PIDController(p1,i1,d1);
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            trans = hardwareMap.get(DcMotorEx.class, "trans");
            trans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class launcherAction implements Action {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!init) {
                    init = true;
                    controller.setPID(p1, i1, d1);
                }
                double launchVel = launch.getVelocity();
                double pid = controller.calculate(launchVel, targetVel);
                //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
                double power = pid;
                launch.setPower(power);
                if (launch.getVelocity() <= -1590) { //1585
                    trans.setPower(1);
                    intake.setPower(-0.6);
                } else if (launch.getVelocity() >= -1590) {
                    trans.setPower(0);
                    intake.setPower(0);
                }
                telemetryPacket.put("time",timer.seconds());
                if(timer.seconds() < 8){
                    return true;
                }
                else{
                    launch.setPower(0);
                    return false;
                }
            }
        }

        public Action fireBall() {
            return new launcherAction();
        }
    }

    IMU imu;
    public class Turret{
        public Turret(HardwareMap hardwareMap){
            turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            controller = new PIDController(p2, i2, d2);
            controller.setPID(p2, i2, d2);
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            ));

            imu.initialize(parameters);
            imu.resetYaw();

        }
        public class turretAction implements Action{
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double Currentangle = (turretMotor.getCurrentPosition()/384.5)*360*(2.0/5.0) + botHeading;
                double ticks = (384.5 * targetAngle + botHeading) / 360.0 * (5.0 / 2.0);;
                double motorPosition = turretMotor.getCurrentPosition();
                double pid = controller.calculate(motorPosition, ticks);
                double power = pid;
                turretMotor.setPower(power);
                if(Currentangle > 360){
                    targetAngle = 0;
                }
                return true;
            }
        }
        public Action turretAction(){return  new turretAction();}
        public Action changeAngle(int target){return new InstantAction(()-> targetAngle = target);
        }
    }
    public class sensors implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public class Intake {

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            trans = hardwareMap.get(DcMotorEx.class, "trans");
            trans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //add color sensor
        }
        public class intakeAction implements Action {
            private boolean init = false;
            ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    trans.setPower(1);
                    intake.setPower(1);
                    init = true;
                    timer = new ElapsedTime();
                }

                if(timer.seconds() < 10 ){
                    return true;
                }
                else{
                    trans.setPower(0);
                    intake.setPower(0);
                    return false;
                }
            }
        }
        public Action takeBall() { return new intakeAction(); }
    }
    public class colorSensors {
        public colorSensors(HardwareMap hardwareMap){
            c1 = hardwareMap.get(ColorSensor.class,"c1");
            c2 = hardwareMap.get(ColorSensor.class,"c2");
            c3 = hardwareMap.get(ColorSensor.class,"c3");
        }
        public class flash implements Action {
            private boolean init = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //logic here for color sensors
                return stored.size() == 3;
            }
        }
    }

    public class scan implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!init) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                initAprilTag();
                for(AprilTagDetection detection: currentDetections){
                    if(detection != null){
                        if (detection.id == 21){
                            motif.add("g");
                            motif.add("p");
                            motif.add("p");
                            //turn table swap to position of green
                            //swap to purple
                            //swap to purple
                        } else if (detection.id == 22) {
                            motif.add("p");
                            motif.add("g");
                            motif.add("p");
                            //swap to purple
                            //swap to green
                            //swap to purple
                        }
                        else if (detection.id == 23){
                            motif.add("p");
                            motif.add("p");
                            motif.add("g");
                            //swap to purple
                            //swap to purple
                            //swap to green
                        }
                    }
                }

            }
            return motif.size() != 3;
        }
    }

    public Action scanMotif(){return new scan();}
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    ArrayList<String> stored = new ArrayList<>();
    ArrayList<String> motif = new ArrayList<>();
    @Override
    public void runOpMode() {


//        initAprilTag();
        Pose2d initialPose = new Pose2d(-48, -50, Math.toRadians(270)); //225
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        Action tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-13,-13),Math.toRadians(270))//change to 180 once april tag and color sensing system works /-2
//                .stopAndAdd(scanMotif())
//                .turn(Math.toRadians(70))
                        .build();
        Action tab2 = drive.actionBuilder(drive.localizer.getPose())//set var constraint later
//                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(-12,-28),Math.toRadians(270))
                .strafeTo(new Vector2d(-12,-53))
                .build();
        Action tab3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-13,-13),Math.toRadians(270))
                .build();
        Action tab4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(12,-28),Math.toRadians(270))
                .strafeTo(new Vector2d(12,-53))
//                .waitSeconds(2.5)
                .build();
        Action tab5 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-13,-13), Math.toRadians(270))
                .build();
        Action tab6 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(36,-12),Math.toRadians(270))
                .strafeTo(new Vector2d(36,-53))
                .build();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            turret.turretAction(),
                            new SequentialAction(
                                    turret.changeAngle(-135), //135 is a placeholder, change to angle of where the motif is
                                    scanMotif(),
                                    turret.changeAngle(45),
                                    tab1, // move to launch position
                                    launcher.fireBall(),
                                    new ParallelAction(
                                            intake.takeBall(),
                                            tab2
                                    ),
                                    tab3,
                                    launcher.fireBall(),
                                    new ParallelAction(
                                            intake.takeBall(),
                                            tab4
                                    ),
                                    tab5,
                                    launcher.fireBall(),
                                    new ParallelAction(
                                            intake.takeBall(),
                                            tab6
                                    )
                            )
                    )
            );
            while (opModeIsActive()) {

                //telemetryAprilTag();

                // Push telemetry to the Driver Station.


                // Save CPU resources; can resume streaming when needed.
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }


                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
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
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
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


    /**
     * Add telemetry about AprilTag detections.
     */
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
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));

            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()


}




