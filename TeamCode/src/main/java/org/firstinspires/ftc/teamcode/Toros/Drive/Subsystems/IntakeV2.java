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

import org.firstinspires.ftc.teamcode.Toros.Drive.MainDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.arcrobotics.ftclib.util.LUT;


@Config
public class IntakeV2 {
    private final DcMotorEx intakeMotor;
    public DcMotorEx launch;
    private final Servo hood;
    private final DcMotorEx trans;
    public ColorSensor c1, c2, c3;
    private final AprilTagProcessor aprilTag;

    public static double turretAngle = 0;
    Gamepad gamepad1;
    private final PIDController controller;

    public static double p1 = 0.0045, i1 = 0, d1 = 0;
    public static double kS = 0.001,kV = 0.00055,kA = 0;
    public static double accel = -30;

    public static double f1 = 0;
    public static double targetVel = -1800;
    private final Gamepad gamepad2;
    public int threshold = 30;


   public static double vel;

   Pinpoint pinpoint;

    double hoodAngle = 0;
    public static double heading;

    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad, Gamepad gamepadA, AprilTagProcessor aprilTag) {
        gamepad1 = gamepad;
        gamepad2 = gamepadA;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood = hardwareMap.get(Servo.class,("hood"));
        hood.setDirection(Servo.Direction.REVERSE);
        launch = hardwareMap.get(DcMotorEx.class, ("launch"));
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trans = hardwareMap.get(DcMotorEx.class, ("trans"));
        trans.setDirection(DcMotorSimple.Direction.FORWARD);
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);
        c1 = hardwareMap.get(ColorSensor.class,"c1");
        c2 = hardwareMap.get(ColorSensor.class,"c2");
        c3 = hardwareMap.get(ColorSensor.class,"c3");
        this.aprilTag = aprilTag;


        pinpoint = new Pinpoint(hardwareMap);
    }

    public void runLauncher() {
        calcShot(heading, MainDrive.startAngle);
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

            if (gamepad2.right_trigger > 0.1 && Math.abs(launch.getVelocity() - targetVel) <= threshold) {
                    trans.setPower(1);
                    intakeMotor.setPower(-1);
            }
            else {
                trans.setPower(0);
                intakeMotor.setPower(0);

            }

    }



    public void runIntake() {
        vel = Math.sqrt(Math.pow(pinpoint.getVelx(),2)+ Math.pow(pinpoint.getVelY(),2));
        heading = pinpoint.getHeading();



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
        if (gamepad1.right_bumper && c3.blue() > 150){
            trans.setPower(-0.1);
        }

        else if(gamepad1.right_bumper){
            trans.setPower(0.2);
        }
        else{
            trans.setPower(0);
        }
        if(gamepad1.left_bumper){
            trans.setPower(-0.15);
        }


    }
    public double getLauncherSpeed() {
        return launch.getVelocity();
    }
    public double getTargetVel() {
        return targetVel;
    }




    public double lastDistance = 1;
    public static double minServo = 0.0;
    public static double maxServo = 1.0;


    LUT<Double, Double> speeds = new LUT<Double, Double>()
    {{
        add(0.0, 900.0);
        add(0.8, 1000.0);
        add(1.4, 1180.0);
        add(3.1, 1450.0);
    }};




    public void calcShot(double robotHeading, int initAngle){
        double g = 32.174 * 12;
        double x =  MainDrive.getDistance() - 3; //distance - shoot past point radius
        double y = 26;
        double a = Math.toRadians(-30);

        hoodAngle = Math.max(Math.toRadians(40),Math.min(Math.toRadians(60),Math.atan(2 *y/x- Math.tan(a))));
        int flywheelSpeed = (int) Math.sqrt(g * x * x / (2* Math.pow(Math.cos(hoodAngle),2) * (x * Math.tan(hoodAngle)-y)));

        double robotVelocity = getVel();

        double coordinateTheta =  Math.atan(pinpoint.getVelY()/pinpoint.getVelx()) - Math.atan(MainDrive.getDistanceY()/ MainDrive.getDistanceX());

        double parallel = -Math.cos(coordinateTheta) * Math.abs(robotVelocity);
        double perpendicular = Math.sin(coordinateTheta) * Math.abs(robotVelocity);

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x /(flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x /time + parallel;
        double nvr = Math.sqrt(ivr*ivr + perpendicular * perpendicular);
        double ndr = nvr * time;

        hoodAngle = Math.max(Math.toRadians(40),Math.min(Math.toRadians(60),Math.atan(vz/nvr)));
        flywheelSpeed = (int) Math.sqrt(g*ndr*ndr / (2*Math.pow(Math.cos(hoodAngle),2) * (ndr * Math.tan(hoodAngle)- y)));

        if(!MainDrive.lockedOn) {
            double turretComp = Math.atan(perpendicular / ivr);
            turretAngle = Math.toDegrees(robotHeading - Math.atan(MainDrive.getDistanceY() / MainDrive.getDistanceX()));

            if (Math.abs(turretAngle) > 150) {
                turretAngle = -turretAngle + Math.copySign(5, turretAngle);
            }

            Turret.setAngle((turretAngle+2) + initAngle);
        }


        setHood(x);

        targetVel =   -4.86091 * flywheelSpeed - 254.80299;

    }

    private void setHood(double angle){
        hood.setPosition(0 + ((70.0 - angle) / 30.0));
    }
    public double getHood(){
        return hoodAngle;
    }





    public static double getHeading(){
        return heading;
    }

    public double calcLaunch1() {
        double distance = lastDistance;
        boolean tagSeen = false;
        double hoodAngleDeg = 60;

        // Get distance
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && (d.id == 24 || d.id == 20)) {
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
        //hood.setPosition(hoodValue);


        targetVel = -(speeds.getClosest(distance));

        return targetVel;
    }


public static double getVel(){
    return vel;
}


}

//:3