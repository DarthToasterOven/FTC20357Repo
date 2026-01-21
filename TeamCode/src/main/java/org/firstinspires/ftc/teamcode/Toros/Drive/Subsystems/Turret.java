package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Mat;

public class Turret {
    public static double p1 = 0.006012 , i1 = 0.00065, d1 = 0.0004314;
    double error = 0;
    private DcMotorEx turretMotor;
    private PIDController controller;
    public double targetAngle = 0;
    public double motorPosition;
    double gearRatio = 2.0 / 5.0;
    private
    Gamepad gamepad2;
    public double power;
    public double targetPos;
    public double Zero = 0;
    IMU imu;
    public double botHeading;
    public Turret(HardwareMap hardwareMap, Gamepad gamepad) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gamepad2 = gamepad;
        controller = new PIDController(p1, i1, d1);
        controller.setPID(p1, i1, d1);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void runTurret() {



        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //Calculates the turret's angle and converts the targetAngle to the motor ticks
        double currentAngle = (turretMotor.getCurrentPosition() / 384.5) * 180 * gearRatio +botHeading/2;

        targetPos = (384.5 * (targetAngle + (int)botHeading/2) / 180 * (5.0 / 2.0));
        motorPosition = turretMotor.getCurrentPosition();

        //Hard limit originally was supposed to be a wrap around


        power = controller.calculate(motorPosition, targetPos);

        turretMotor.setPower(power);

        if(Math.abs(currentAngle) > 85 || Math.abs(targetAngle) > 85){
            targetAngle = -targetAngle + Math.copySign(5, targetAngle);
        }

        if(Math.abs(gamepad2.left_stick_x) > 0.1){
            targetAngle += gamepad2.left_stick_x *2;

        }
        if(gamepad2.aWasPressed()){
            targetAngle = 0;
        }
        if(gamepad2.xWasPressed()){
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetAngle = botHeading - currentAngle;
        }



    }

    public void setAngle(double target) {
        targetAngle = (int) target;
    }

    public double getTurretAngle() {
        return (turretMotor.getCurrentPosition() / 384.5) * 180 * gearRatio + botHeading/2;
    }
}

