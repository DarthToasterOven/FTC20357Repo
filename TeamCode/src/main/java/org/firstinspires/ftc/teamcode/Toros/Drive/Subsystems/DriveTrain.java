package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class DriveTrain {

    private DcMotor FrontLeftMotor,BackLeftMotor,FrontRightMotor,BackRightMotor; //Motors
    IMU imu;
    private boolean Rtoggle,Xtoggle; // Toggles for turning down the speed of the robot
    Gamepad currentGamepad1 = new Gamepad(), previousGamepad1 = new Gamepad(); //fragment of the toggles. needed just in case
    Gamepad gamepad1; // the gamepad which we intialize when we construct the class in the actual drive program
    public DriveTrain(HardwareMap hardwareMap,Gamepad gamepad){
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Sets the zero power behaviors which when it stops will resist force on it causing it to stop faster
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //These two motors need to be in reverse in order for correct movement
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        imu = hardwareMap.get(IMU.class, "imu2");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));



        imu.initialize(parameters);
        imu.resetYaw();

        //creates our gamepad object in order to use controls
        this.gamepad1 = gamepad;


    }

    public void drive (){
        currentGamepad1.copy(gamepad1);
        //previousGamepad1.copy(currentGamepad1)

        //Our toggle to slow down either rotationally or just in the x or y directions
        if(currentGamepad1.dpadLeftWasPressed()){
            Xtoggle = !Xtoggle;
        }
        if(currentGamepad1.dpadRightWasPressed()){
            Rtoggle = !Rtoggle;
        }

/*
        if(currentGamepad1.x && !previousGamepad1.x){
            Xtoggle = !Xtoggle;
        }
        if(currentGamepad1.b && !previousGamepad1.b){
            Rtoggle = !Rtoggle;
        }
*/

        //Taking our gamepad inputs
        double x = gamepad1.left_stick_x * 1.1; // the *1.1 counteracts imperfect strafing
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        //Toggles just slow down the speed to 3/4 of the robots speed/power
        if(Xtoggle){
            x *= 0.5;
            y *= 0.5;
        }
        else{
            x*=1;
            y*=1;
        }
        if(Rtoggle){
            turn *= 0.5;
        }
        else{
            turn*=1;
        }


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX*1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        if(gamepad1.options){
            imu.resetYaw();
        }

        FrontLeftMotor.setPower(frontLeftPower);
        BackLeftMotor.setPower(backLeftPower);
        FrontRightMotor.setPower(frontRightPower);
        BackRightMotor.setPower(backRightPower);
    }

    //Any getter methods that we need in order for telemetry or other use
    public boolean getXToggle(){
        return Xtoggle;
    }
    public boolean getRToggle(){
        return Rtoggle;
    }


}
