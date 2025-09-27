package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV2 {
    private DcMotorEx intakeMotor;
    private Servo gateServo;
    Gamepad gamepad2;
    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        gateServo = hardwareMap.get(Servo.class, "gateServoz");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runIntake(){
        if(gamepad2.left_bumper){
            intakeMotor.setPower(1);
        }
        if(gamepad2.right_bumper){
            intakeMotor.setPower(-1);
        }
        if(gamepad2.b){
            intakeMotor.setPower(0);
        }
        if(gamepad2.y){
            gateServo.setPosition(1);
        }
        if(gamepad2.a){
            gateServo.setPosition(0);
        }
    }
}
