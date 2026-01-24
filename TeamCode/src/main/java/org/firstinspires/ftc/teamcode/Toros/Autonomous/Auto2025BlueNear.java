package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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

@Autonomous(name = "Auto2025BlueNear")
public class Auto2025BlueNear extends LinearOpMode {
    public DcMotorEx launch, turretMotor, trans;
    public Servo hood;
    public ColorSensor c1,c2,c3;
    private DcMotor intake;
    private PIDController controller;

    public static double p1 = 0.01, i1 = 0.6, d1 = 0;
    public static double kS = 0.025,kV = 0.001,kA = -0.025;
    public static double accel = 20;

    public static double p2 = 0.006012 , i2 = 0.00065, d2 = 0.0004314;

    public static int targetVel = -1200;
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
            hood = hardwareMap.get(Servo.class, "hood");
        }

        public class launcherAction implements Action {
            private boolean init = false;
            ElapsedTime timer = new ElapsedTime();
            //timer.reset();
            @Override

            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                telemetry.addData("Launch Velocoty", launch.getVelocity());
                telemetry.addData("Launch pwer", launch.getPower());
                telemetry.addData("timer", timer);


                telemetry.update();
                if(!init) {
                    timer.reset();
                    init = true;
                    hood.setPosition(0.8);
                }
                if (launch.getVelocity() <= -1100) { //1585

                    trans.setPower(-1);
                    intake.setPower(-0.7);

                } else if (launch.getVelocity() >= -1100) {
                    trans.setPower(0);
                    intake.setPower(0);
                }
                telemetryPacket.put("time",timer.seconds());
                if(timer.seconds() < 4){
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

        public class revLaunch implements Action{
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!init){
                    controller.setPID(p1, i1, d1);
                }
                double launchVel = launch.getVelocity();
                SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS,kV,kA);
                double pid = controller.calculate(launchVel, targetVel);
                double ff = feedforward.calculate(targetVel,accel);
                double power = pid + ff;
                launch.setPower(power);
                return true;
            }
        }
        public Action revMotor() {return  new revLaunch();}
    }

    IMU imu;
    public class Turret{
        public Turret(HardwareMap hardwareMap){
            turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            controller = new PIDController(p2, i2, d2);
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            ));

            imu.initialize(parameters);

        }
        public class turretAction implements Action{

            private boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!init){
                    controller.setPID(p2,i2,d2);
                    init = true;
                }
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                //Calculates the turret's angle and converts the targetAngle to the motor ticks
                double currentAngle = (turretMotor.getCurrentPosition() / 384.5) * 180.0 * (2.0/5.0) + (botHeading/2);

                double targetPos = (384.5 * targetAngle + (int)botHeading/2) / 180.0 * (5.0 / 2.0);
                int motorPosition = turretMotor.getCurrentPosition();

                //Hard limit originally was supposed to be a wrap around


                double power = controller.calculate(motorPosition, targetPos);
                turretMotor.setPower(power);
                return true;
            }
        }
        public Action turretGo(){return new turretAction();}
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
                    trans.setPower(-0.18);
                    intake.setPower(-1);
                    init = true;
                    timer = new ElapsedTime();
                }

                if(timer.seconds() < 3 ){
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

        public class runTrans implements Action{
            boolean init = false;
            ElapsedTime timer;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!init){
                    timer = new ElapsedTime();
                    trans.setPower(-0.1);
                    init = true;
                }
                if(timer.seconds() < 3){
                    return true;
                }
                else{
                    trans.setPower(0);
                    return false;

                }
            }
        }
        public Action transRun(){return new runTrans();}

        public class runIntake implements Action {
            private boolean init = false;
            ElapsedTime timer;

            @Override

            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    intake.setPower(-1);
                    init = true;
                    timer = new ElapsedTime();
                }

                if(timer.seconds() < 3 ){
                    return true;
                }
                else{
                    intake.setPower(0);
                    return false;
                }
            }
        }
        public Action intakeRun(){return new runIntake();}
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
        Pose2d initialPose = new Pose2d(-48, -50, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        Action tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-13,-13),Math.toRadians(270), new TranslationalVelConstraint(15.0))
//                .stopAndAdd(scanMotif())
//                .turn(Math.toRadians(70))
                .build();
        Action tab2 = drive.actionBuilder(new Pose2d(-13,-13,Math.toRadians(270)))//set var constraint later

//              .waitSeconds(5)

                .strafeTo(new Vector2d(-13,-58), new TranslationalVelConstraint(15.0))
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(-14,-53,Math.toRadians(270)))
                //.waitSeconds(1.5)
                .strafeTo(new Vector2d(-13,-13))

                .build();
        Action tab4 = drive.actionBuilder(new Pose2d(-13,-13,Math.toRadians(270)))
                //.waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(13,-28),Math.toRadians(270))
                .strafeTo(new Vector2d(13,63), new TranslationalVelConstraint(15.0))
//                .waitSeconds(2.5)
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(13,-60,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-13,-13), Math.toRadians(270))
                .build();
        Action tab6 = drive.actionBuilder(new Pose2d(-13,-13,Math.toRadians(270)))
                //.waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(10,-35),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(38,-35),Math.toRadians(270))

                .strafeTo(new Vector2d(38,-53), new TranslationalVelConstraint(15.0))
                .build();
        Action tab7 = drive.actionBuilder(new Pose2d(38,-53,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-13,-13),Math.toRadians(270))
                .build();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            launcher.revMotor(),
                            new SequentialAction(

                                    tab1, // move to launch position
                                    launcher.fireBall(), // +3 (preloaded)
                                    new ParallelAction(//1st spike
                                            intake.takeBall(),
                                            tab2
                                    ),

                                    tab3, // move to launch
                                    launcher.fireBall(), // +6
                                    new ParallelAction( // 2nd spike
                                            intake.takeBall(),
                                            tab4
                                    ),
                                    tab5,
                                    launcher.fireBall(), // + 9
                                    new ParallelAction( // 3rd spike
                                            intake.takeBall(),
                                            tab6
                                    ),
                                    tab7
                            )
                    )
            );

            while (opModeIsActive()) {
                telemetry.addData("motif",motif.get(1));
                //telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.addData("Launch Velocoty", launch.getVelocity());
                telemetry.update();

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
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()


}




