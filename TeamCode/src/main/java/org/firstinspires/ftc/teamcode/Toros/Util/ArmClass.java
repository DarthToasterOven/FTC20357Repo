package org.firstinspires.ftc.teamcode.Toros.Util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Disabled
public class ArmClass {
    Gamepad gamepad2;
    private final DcMotorEx pivot, slideLeft, slideRight;
    public ArmClass(HardwareMap hardwareMap,Gamepad Gamepad){
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        gamepad2 = Gamepad;
    }

    public void runPivot(){
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDController controller;
        double p1 = 0.009, i1 = 0.001, d1 = 0.0005;

        double f1 = 0.185;

        int target1 = 425;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);
        int armPos = pivot.getCurrentPosition();
        double pid = controller.calculate(armPos, target1);
        double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

        double power = pid + ff;
        if(gamepad2.right_stick_y <= 1.0 && gamepad2.right_stick_y != 0.0|| gamepad2.right_stick_y >= -1.0 && gamepad2.right_stick_y != 0){
            power = gamepad2.right_stick_y * 0.5;
            target1 = armPos;
        }

        pivot.setPower(power);
    }
    public void runSlides(){
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDController controller;
        double p1 = 0, i1 = 0, d1 = 0;

        double f1 = 0;

        int target1 = 425;
        controller = new PIDController(p1,i1,d1);
        double ticks_in_degrees = 1440/180;

        controller.setPID(p1,i1,d1);
        int armPos = slideLeft.getCurrentPosition();
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

}
