package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public class Turret {
    public static double p1 = 0.009, i1 = 0.45, d1 = 0;

    public static int targetPosition = 0;
    private DcMotorEx turretMotor;
    private PIDController controller;
    private double ticks = 384.5/180;
    private double circumference = 203 * Math.PI;
    int direction = 1;
    private
    Gamepad gamepad2;
    public Turret(HardwareMap hardwareMap, Gamepad gamepad){
        turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
        gamepad2 = gamepad;
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);
    }

    private void runTurret(){
        double angle = (turretMotor.getCurrentPosition()/384.5)*360;
        double x = gamepad2.left_stick_x;
        double y = gamepad2.left_stick_y;
        double v = x * Math.cos(angle) + y * Math.sin(angle);
        double power = v /Math.max(Math.abs(v),1);
        turretMotor.setPower(power*direction);

        if(angle > 355 || angle < 5){
            direction*=-1;
        }
//        double motorPosition = turretMotor.getCurrentPosition();
//        double pid = controller.calculate(motorPosition, targetPosition);
//        //double ff = Math.cos(Math.toRadians(targetVel /ticks_in_degrees)) * f1;
//        double power = pid;
//        turretMotor.setPower(power);

    }
    private void setAngle(int angle){
        double Currentangle = (turretMotor.getCurrentPosition()/384.5)*360;
        double ticks = (384.5*angle)/360;
        double motorPosition = turretMotor.getCurrentPosition();
        double pid = controller.calculate(motorPosition, ticks);
        double power = pid;
        turretMotor.setPower(power);
        if(Currentangle > 355 || Currentangle < 5){
            direction*=-1;
        }
    }
}
