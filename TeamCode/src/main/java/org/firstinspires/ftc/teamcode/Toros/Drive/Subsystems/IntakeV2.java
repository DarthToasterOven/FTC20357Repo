package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.arcrobotics.ftclib.util.LUT;

@Config
public class IntakeV2 {
    private DcMotorEx intakeMotor;
    public DcMotorEx launch;
    private Servo hood;
    private DcMotorEx trans;
    public ColorSensor c1, c2, c3;
    private AprilTagProcessor aprilTag;


    Gamepad gamepad1;
    private PIDController controller;

    public static double p1 = 0.0045, i1 = 0, d1 = 0;
    public static double kS = 0.001,kV = 0.00055,kA = 0;
    public static double accel = -30;

    public static double f1 = 0;
    public static double targetVel = -1800;
    private Gamepad gamepad2;
    public int threshold = 80;

    public double ticksPerSecond = 1500;



    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad, Gamepad gamepadA, AprilTagProcessor aprilTag) {
        gamepad1 = gamepad;
        gamepad2 = gamepadA;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood = hardwareMap.get(Servo.class,("hood"));
        launch = hardwareMap.get(DcMotorEx.class, ("launch"));
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trans = hardwareMap.get(DcMotorEx.class, ("trans"));
        trans.setDirection(DcMotorSimple.Direction.REVERSE);
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);
        c1 = hardwareMap.get(ColorSensor.class,"c1");
        c2 = hardwareMap.get(ColorSensor.class,"c2");
        c3 = hardwareMap.get(ColorSensor.class,"c3");
        this.aprilTag = aprilTag;


    }

    public void runlauncher() {
        //targetVel = -1* calcLaunch(0);
        //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
            if (gamepad2.left_bumper)
            {
                //targetVel = airsort;
                targetVel = -1480;
                hood.setPosition(0.8);
            }
            else {
                targetVel = calcLaunch1();
            }
        if (gamepad1.b) {
            intakeMotor.setPower(0);
            trans.setPower(0);
        }

        if(gamepad1.dpadUpWasPressed()){
            targetVel -=50;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetVel +=50;
        }


        //launch normal
            //Laucnhes the ball with PID
            if (gamepad2.left_trigger > 0.1) {
                double launchVel = launch.getVelocity();
                SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS,kV,kA);
                double pid = controller.calculate(launchVel, targetVel);
                double ff = feedforward.calculate(targetVel,accel);
                double power = pid + ff;
                launch.setPower(power);
            }
            if (gamepad2.left_trigger < 0.1) {
                launch.setPower(0);
            }

            if (gamepad2.right_trigger > 0.1) {
                if (Math.abs(launch.getVelocity() - targetVel) <= threshold) { //threshold velocity
                    trans.setPower(1);
                    intakeMotor.setPower(-1);
                }
                else {
                    trans.setPower(0);
                    intakeMotor.setPower(0);

                }
            }
            else {
                trans.setPower(0);
                intakeMotor.setPower(0);

            }

    }



    public void runIntake() {
        //Moves ball into robot
        if (gamepad1.right_trigger > 0.25) {
            intakeMotor.setPower(-gamepad1.right_trigger);
        }

        //Moves ball out of robot
        if (gamepad1.left_trigger > 0.25) {
            intakeMotor.setPower(gamepad1.left_trigger);
        }
        if (gamepad1.left_trigger < 0.25 && gamepad1.right_trigger < 0.25 && gamepad2.right_trigger <0.25) {// turns off the motor if both triggers are not pressed
            intakeMotor.setPower(0);
        }


        //hardstop for all systems
        if (gamepad1.b) {
            intakeMotor.setPower(0);
            trans.setPower(0);
        }

        if(c3.blue() > 150 && c2.blue() > 150 && c1.blue() > 150){
            gamepad1.rumble(1500);
        }



        }

    public void transfer(){
        if(gamepad1.right_trigger > 0.25 &&  c3.blue() < 150){
            trans.setPower(0.25);
        }
        if(gamepad1.left_bumper){
            trans.setPower(-0.35);
        }

        if(gamepad1.leftBumperWasReleased() || gamepad1.rightBumperWasReleased()){
            trans.setPower(0);
        }
    }
    public double getLauncherSpeed() {
        return launch.getVelocity();
    }
    public double getTargetVel() {
        return targetVel;
    }



    LUT<Double, Double> speeds = new LUT<Double, Double>()
    {{
        add(0.0, 900.0);
        add(0.5, 1000.0);
        add(1.4, 1170.0);
        add(3.1, 1560.0);
    }};

    public double calcLaunch1() {
        double distance = lastDistance;
        boolean tagSeen = false;
        double hoodAngleDeg = 60;

        // Get distance
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && (d.id == 24 || d.id == 23)) {
                distance = d.ftcPose.range * 0.0254;
                tagSeen = true;
                break;
            }

        }

        if (tagSeen){
            lastDistance = distance;
        }

        // Define distance
        distance = Math.max(0.1, Math.min(4, distance));

        //get hood angle (degrees)

        hoodAngleDeg = 60 + (distance - 0.6) * (40 - 60) / (1 - 0.5);


        // Define hood angle
        hoodAngleDeg = Math.max(40, Math.min(60, hoodAngleDeg));
        double hoodValue = minServo + ((60-hoodAngleDeg) / 20) * (maxServo - minServo);
        hood.setPosition(hoodValue);


        targetVel = -(speeds.getClosest(distance));

        return targetVel;
    }



    public static double k = -2.6;

    public double lastDistance = 1;
    public static double h = 0.69;
    public static double flywheelRadius = 0.048;
    public static double minAngle = 40;
    public static double maxAngle = 50;
    public static double minServo = 0.0;
    public static double maxServo = 1.0;
    public double calcLaunch2() { // to make air sort: add parameter?? if slow then: hood angle = high, else: calc hood angle

        //vars
        double distance = lastDistance;
        boolean tagSeen = false;
        double hoodAngleDeg = 60;

        // Get distance
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && (d.id == 24 || d.id == 23)) {
                distance = d.ftcPose.range * 0.0254;
                tagSeen = true;
                break;
            }

        }

        if (tagSeen){
            lastDistance = distance;
        }

        // Define distance
        distance = Math.max(0.1, Math.min(4, distance));

        //get hood angle (degrees)

        hoodAngleDeg = 60 + (distance - 0.6) * (40 - 60) / (1 - 0.5);



        // Define hood angle
        hoodAngleDeg = Math.max(40, Math.min(60, hoodAngleDeg));
        double hoodValue = minServo + ((60-hoodAngleDeg) / 20) * (maxServo - minServo);
        hood.setPosition(hoodValue);


        // Convert to rad
        double theta = Math.toRadians(hoodAngleDeg);

        // sqrt not 0 (very annoying)
        double denom = 2*Math.pow(Math.cos(theta), 2) * (distance *Math.tan(theta) - h);
        if (denom <= 0) return targetVel;

        // the actual calculation
        double v = distance * Math.sqrt(9.81 /denom);

        // Linear to angular to ticks/sec
        double omega = v / flywheelRadius;
        ticksPerSecond = omega * 28 / (2 * Math.PI);

        //tuning

        if (distance <= 1) {
            k = -2.7;
        }
        if (distance > 1){
            k = -2.7;
        }
        if (distance > 1.2){
            k = -2.1;
        }
        if (distance > 2.6){
            k = -2.65;
        }


        ticksPerSecond *= k;

        targetVel = (int) ticksPerSecond;

        return (int) ticksPerSecond;
    }



}

//:3