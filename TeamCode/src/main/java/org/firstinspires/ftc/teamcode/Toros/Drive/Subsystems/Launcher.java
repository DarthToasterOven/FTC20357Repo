package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;

public class Launcher {
    private MotorGroup myMotors;
    private Motor laucnhLeft, launchRight;
    private Gamepad gamepad2;
    private Servo gateServo;

    private List<LynxModule> hubs;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        laucnhLeft = new Motor(hardwareMap, "launchLeft", Motor.GoBILDA.RPM_435);
        launchRight = new Motor(hardwareMap, "launchRight", Motor.GoBILDA.RPM_435);
        myMotors = new MotorGroup(laucnhLeft,launchRight);
        myMotors.setRunMode(Motor.RunMode.RawPower);
        myMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        hubs = hardwareMap.getAll(LynxModule.class);
        gateServo = hardwareMap.get(Servo.class, "gateServo");
    }
    public void runLauncher(){
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        myMotors.set(gamepad2.right_stick_y);
        if(gamepad2.a){
            myMotors.stopMotor();
        }
        if(gamepad2.right_bumper){
            gateServo.setPosition(0.4);
        }
        if(gamepad2.left_bumper){
            gateServo.setPosition(1);
        }

    }

}
