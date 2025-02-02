package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private Servo specClaw, budget;
    private CRServoImplEx sampClaw;
    Gamepad currentGamepad2 = new Gamepad(), previousGamepad2 = new Gamepad();
    Gamepad gamepad2;
    private boolean breakfast;
    public Intake(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        sampClaw = hardwareMap.get(CRServoImplEx.class,"slurp");
        specClaw = hardwareMap.get(Servo.class,"specClaw");
        budget = hardwareMap.get(Servo.class,"brisket");
    }

    public void runClaw(){
        if(gamepad2.left_trigger > 0){
            sampClaw.setPower(1);
        }
        if(gamepad2.right_trigger > 0){
            sampClaw.setPower(-1);
        }
        if(gamepad2.b){
            sampClaw.setPwmDisable();
        }
        if(gamepad2.left_bumper){
            specClaw.setPosition(0);
        }
        if(gamepad2.right_bumper){
            specClaw.setPosition(1);
        }
        if(currentGamepad2.y && !previousGamepad2.y){
            breakfast = !breakfast;
        }

        if(breakfast){
            budget.setPosition(0);
        }
        else{
            budget.setPosition(1);
        }
    }
    public boolean getToggle(){
        return breakfast;
    }
    public double getPower(){
        return sampClaw.getPower();
    }
}
