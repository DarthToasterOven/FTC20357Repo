package org.firstinspires.ftc.teamcode.Toros.Drive;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentric")
public class DriveFieldCentric extends LinearOpMode {

    IMU imu;
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        if(opModeIsActive()){
            FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
            BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
            FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
            BackRightMotor = hardwareMap.get(DcMotor.class, "br");
            FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            imu = hardwareMap.get(IMU.class, "imu2");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ));

            imu.initialize(parameters);
            imu.resetYaw();
            while (opModeIsActive()){

                if(gamepad1.options){
                    imu.resetYaw();
                }
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX = rotX*1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                FrontLeftMotor.setPower(frontLeftPower);
                BackLeftMotor.setPower(backLeftPower);
                FrontRightMotor.setPower(frontRightPower);
                BackRightMotor.setPower(backRightPower);
            }
        }





    }
}

