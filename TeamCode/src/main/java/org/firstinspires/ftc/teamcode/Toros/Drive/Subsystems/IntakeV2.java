package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV2 {
    private DcMotorEx intakeMotor;
    private DcMotorEx launch;
    private Servo gate;
    Gamepad gamepad1;
    boolean toggle = false;
    boolean toggle2 = false;

    private PIDController controller;




    public static double p1 = 0.009, i1 = 0.45, d1 = 0;

    public static double f1 = 0;

    public static int targetVel = -1610;
    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad1 = gamepad;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch = hardwareMap.get(DcMotorEx.class, ("launch"));
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gate = hardwareMap.get(Servo.class,("gate"));
        controller = new PIDController(p1,i1,d1);

    }
    public void runIntake() {

        if (gamepad1.xWasPressed()) {
            gate.setPosition(0);
        }
        if (gamepad1.yWasPressed()) {
            gate.setPosition(0.25);
        }
        if (gamepad1.right_trigger > 0.1) {
            controller.setPID(p1,i1,d1);
            double launchVel = launch.getVelocity();
            double pid = controller.calculate(launchVel, targetVel);
            //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
            double power = pid;
            launch.setPower(power);
            if(launch.getVelocity() <= -1590){
                gate.setPosition(0);
                intakeMotor.setPower(-0.5);
            }
//            if(launch.getVelocity() >= -1600) {
//                double Error2 = -1600 - launch.getVelocity();
//                launch.setPower((Error2/-1600)*-1);
//
//            }
        }
        else if (gamepad1.left_trigger > 0.1) {
            launch.setPower(gamepad1.left_trigger); // wrong way
        }
        else{
            launch.setPower(0);
            intakeMotor.setPower(0);
        }

        if (gamepad1.right_bumper) {
            intakeMotor.setPower(-0.5);
        }
        if (gamepad1.rightBumperWasReleased()) {
            intakeMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(0.5);
        }
        if (gamepad1.leftBumperWasReleased()) {
            intakeMotor.setPower(0);
        }
        if (gamepad1.b) {
            intakeMotor.setPower(0);
        }
        if(gamepad1.dpadUpWasPressed()){
            targetVel -=50;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetVel +=50;
        }
    }
    public double getLauncherSpeed(){
        return launch.getVelocity();
    }

}

