package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides {
      DcMotorEx slideLeft, slideRight;

    public static double p = 0.004, i = 0.001, d = 0.0005;
    public static double f = 0.195;

//     P is for proportional which will be proportionate to the error this causes the arm to go up for us
//     I is for integral which integrates past values of error seeking to reduced the residual error by adding control and eliminate the error which gets us closer to the target point
//     D is for derivative which best estimates the trend of the error based on the rate of change to reduced the effect to dampen it to not overshoot
//     F is for feedforward which accounts for things more external and prevents disturbances in our use case showing gravity who is boss

    public static int target = 50;
    private final double ticks_in_degrees = 1440 / 180; // Ticks of the tetrix 60:1 motor in degrees (divided by 180)
    //Declares the Variables for all of our motors and servos
    private Gamepad gamepad2;

    public Slides (HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runSlides(){

        PIDController controller;
        double p1 = 0.006, i1 = 0.01, d1 = 0.00005;

        double f1 = 0.005;
        int armPos = slideLeft.getCurrentPosition();
        int target1 = armPos;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);

        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;
        if(gamepad2.right_stick_y <= 1.0 && gamepad2.right_stick_y != 0.0|| gamepad2.right_stick_y >= -1.0 && gamepad2.right_stick_y != 0){
            power = gamepad2.right_stick_y * 0.5;
            target1 = armPos;
        }

        slideLeft.setPower(power);
        slideRight.setPower(power);
    }
    public int getSlidePos(){
        return slideLeft.getCurrentPosition();
    }
}
