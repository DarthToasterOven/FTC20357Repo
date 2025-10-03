package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV2 {
    private DcMotorEx intakeMotor;
    Gamepad gamepad2;
    public IntakeV2(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        intakeMotor = hardwareMap.get(DcMotorEx.class, " intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runIntake(){
        intakeMotor.setPower(gamepad2.left_stick_y);
        if(gamepad2.b){
            intakeMotor.setPower(0);
        }
    }
}
