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
    int targetAngle = 0;
    private
    Gamepad gamepad2;
    public Turret(HardwareMap hardwareMap, Gamepad gamepad){
        turretMotor = hardwareMap.get(DcMotorEx.class,"turret");
        gamepad2 = gamepad;
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);
    }

    private void runTurret(){
        double Currentangle = (turretMotor.getCurrentPosition()/384.5)*360;
        if(Currentangle > 360){
            targetAngle = 0;
        }
        double ticks = (384.5*targetAngle)/360;
        double motorPosition = turretMotor.getCurrentPosition();


        if(Math.abs(gamepad2.left_stick_x) > 0.1){
            targetAngle += gamepad2.left_stick_x*15;
        }

        double pid = controller.calculate(motorPosition, ticks);
        double power = pid;



        turretMotor.setPower(power);
    }
    public void setAngle(int targetAngle){
        double Currentangle = (turretMotor.getCurrentPosition()/384.5)*360;
        double ticks = (384.5*targetAngle)/360;
        double motorPosition = turretMotor.getCurrentPosition();
        double pid = controller.calculate(motorPosition, ticks);
        double power = pid;
        turretMotor.setPower(power);
        if(Currentangle > 360){
            this.targetAngle = 0;
        }
    }
    public int getTurretAngle(){
        return (int)(turretMotor.getCurrentPosition()/384.5)*360;
    }

}
