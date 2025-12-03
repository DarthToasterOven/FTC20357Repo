package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public class Turret {
    public static double p1 = 0.0025, i1 = 0.000001, d1 = 0.0001;

    private DcMotorEx turretMotor;
    private PIDController controller;
    public int targetAngle = 0;
    public double motorPosition;
    double gearRatio = 2.0 / 5.0;
    private
    Gamepad gamepad2;
    public double power;
    public double targetPos;

    public Turret(HardwareMap hardwareMap, Gamepad gamepad) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gamepad2 = gamepad;
        controller = new PIDController(p1, i1, d1);
    }

    public void runTurret() {
        controller.setPID(p1, i1, d1);
        double currentAngle = (turretMotor.getCurrentPosition() / 384.5) * 360.0 * gearRatio;
        targetPos = (384.5 * targetAngle) / 360.0 * (5.0 / 2.0);
        motorPosition = turretMotor.getCurrentPosition();

//        if(currentAngle > 360){
//            targetAngle = 0;
//        }
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            targetAngle += gamepad2.left_stick_x * 5;
        }

        power = controller.calculate(motorPosition, targetPos);

        turretMotor.setPower(power);
    }

    public void setAngle(double target) {
        targetAngle = (int) target;
    }

    public double getTurretAngle() {
        return (turretMotor.getCurrentPosition() / 384.5) * 360 * gearRatio;
    }
}

