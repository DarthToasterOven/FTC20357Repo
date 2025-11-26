package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV2 {
    private DcMotorEx intakeMotor;
    private DcMotorEx launch;
    private Servo gate;
    private CRServo trans, trans1;

    Gamepad gamepad1;
    boolean toggle = false;
    boolean toggle2 = false;

    private PIDController controller;

    public static double p1 = 0.009, i1 = 0.45, d1 = 0;

    public static double f1 = 0;
    public static int targetVel = -1610;


    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad) {
        gamepad1 = gamepad;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch = hardwareMap.get(DcMotorEx.class, ("launch"));
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gate = hardwareMap.get(Servo.class, ("gate"));
        trans = hardwareMap.get(CRServo.class, ("trans"));
        trans1 = hardwareMap.get(CRServo.class, ("trans1"));
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);

    }

    public void runlauncher() {

        //Laucnhes the ball with PID and activates other systems when running in order to launch
        if (gamepad1.right_trigger > 0.1) {
            double launchVel = launch.getVelocity();
            double pid = controller.calculate(launchVel, targetVel);
            //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
            double power = pid;
            launch.setPower(power);
            //Starts running all systems if the velocity of the motor is sufficient
            if (launch.getVelocity() <= -1590) {
                gate.setPosition(0);
                intakeMotor.setPower(-0.5);
                trans.setPower(1);
                trans1.setPower(1);
            }
            else if (launch.getVelocity() >= -1590) { // Once it bleeds speed it stops the systems allowing for the motor to gain enough speed to launch or stop
            intakeMotor.setPower(0);
            gate.setPosition(0.2);
            trans.setPower(0);
            trans1.setPower(0);
            }
//            else { //if not running it turn off everything
//                launch.setPower(0);
//                intakeMotor.setPower(0);
//                trans.setPower(0);
//                trans1.setPower(0);
//            }

        }
        else if (gamepad1.left_trigger > 0.1) {//reverse launcher if it gets stuck
            launch.setPower(gamepad1.left_trigger);
        }
        // Allows for increasing and decreasing of launch speed
            if (gamepad1.dpadUpWasPressed()) {
            targetVel -= 50;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetVel += 50;
        }

    }

    public void runIntake() {
        //gate controls
        if (gamepad1.xWasPressed()) {
            gate.setPosition(0);
        }
        if (gamepad1.yWasPressed()) {
            gate.setPosition(0.25);
        }

//        if (gamepad1.right_trigger > 0.1) {
//            controller.setPID(p1,i1,d1);
//            double launchVel = launch.getVelocity();
//            double pid = controller.calculate(launchVel, targetVel);
//            //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
//            double power = pid;
//            launch.setPower(power);
//            if(launch.getVelocity() <= -1590){
//                gate.setPosition(0);
//                intakeMotor.setPower(-0.5);
//            }
//        }
//        else if (gamepad1.left_trigger > 0.1) {
//            launch.setPower(gamepad1.left_trigger); // wrong way
//        }
//        else{
//            launch.setPower(0);
//            intakeMotor.setPower(0);
//        }

        //Moves ball into robot
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(-0.5);
            trans.setPower(1);
            trans1.setPower(1);
        }
        if (gamepad1.rightBumperWasReleased()) {
            intakeMotor.setPower(0);
            trans.setPower(0);
            trans.setPower(0);
        }
        //Moves ball out of robot
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(0.5);
            trans.setPower(-1);
            trans1.setPower(-1);
        }
        if (gamepad1.leftBumperWasReleased()) {
            intakeMotor.setPower(0);
            trans.setPower(0);
            trans1.setPower(0);
        }
        //hardstop for all systems
        if (gamepad1.b) {
            intakeMotor.setPower(0);
            trans.setPower(0);
            trans1.setPower(0);
        }

//        if(gamepad1.dpadUpWasPressed()){
//            targetVel -=50;
//        } else if (gamepad1.dpadDownWasPressed()) {
//            targetVel +=50;
//        }
    }

    public double getLauncherSpeed() {
        return launch.getVelocity();
    }
}

