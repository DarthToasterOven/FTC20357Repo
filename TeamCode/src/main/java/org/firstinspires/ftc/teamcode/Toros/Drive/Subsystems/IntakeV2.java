package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class IntakeV2 {
    private DcMotorEx intakeMotor;
    public DcMotorEx launch;
    private Servo hood;
    private DcMotorEx trans;
    public ColorSensor c3;
    private AprilTagProcessor aprilTag;


    Gamepad gamepad1;
    private PIDController controller;

    public static double p1 = 0.009, i1 = 0.45, d1 = 0;

    public static double f1 = 0;
    public static int targetVel = -1600;
    private Gamepad gamepad2;
    public int threshold = -1500;

    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad, Gamepad gamepadA) {
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
        c3 = hardwareMap.get(ColorSensor.class,"c3");


    }

    public void runlauncher() {

        //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;

        //Laucnhes the ball with PID
        if (gamepad2.left_trigger > 0.1) {
            double launchVel = launch.getVelocity();
            double pid = controller.calculate(launchVel, targetVel);
            double power = pid;
            launch.setPower(power);
        }
        if(gamepad2.left_trigger < 0.1){
            launch.setPower(0);
        }

        if (gamepad2.right_trigger > 0.1){
            if (launch.getVelocity() <= threshold) { //threshold velocity
                trans.setPower(1);
                intakeMotor.setPower(-1);
            }
        }
        else{
            trans.setPower(0);
        }
        // Allows for increasing and decreasing of launch speed
        if (gamepad2.dpadUpWasPressed()) {
            targetVel -= 50;// increases speed
            threshold -=50;
        } else if (gamepad2.dpadDownWasPressed()) {
            targetVel += 50; // decreases speed
            threshold +=50;
        }


        if(gamepad1.dpadUpWasPressed()){// close side
            targetVel = -1310;
            hood.setPosition(1);
        } else if (gamepad1.dpadDownWasPressed()) { //far side
            targetVel = -1610;
            hood.setPosition(0);
        }

        if(targetVel > 0){ //software limiter for the velocity to not go backwards
            targetVel = 0;
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
        if(gamepad1.left_trigger < 0.1 && gamepad1.right_trigger < 0.1){// turns off the motor if both triggers are not pressed
            intakeMotor.setPower(0);
        }

        //hardstop for all systems
        if (gamepad1.b) {
            intakeMotor.setPower(0);
            trans.setPower(0);
        }

//        if(gamepad1.dpadUpWasPressed()){
//            targetVel -=50;
//        } else if (gamepad1.dpadDownWasPressed()) {
//            targetVel +=50;
//        }
    }

    public void runTrans(){
        if(gamepad1.right_bumper &&  c3.blue() < 150){
            trans.setPower(0.6);
        }
        if(gamepad1.left_bumper){
            trans.setPower(-0.6);
        }

        if(gamepad1.leftBumperWasReleased() || gamepad1.rightBumperWasReleased()){
            trans.setPower(0);
        }
    }
    public double getLauncherSpeed() {
        return launch.getVelocity();
    }

}

