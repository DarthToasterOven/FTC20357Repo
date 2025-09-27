package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Launcher {
    private MotorGroup myMotors;
    private Motor laucnhLeft, launchRight;
    private Gamepad gamepad2;

    private List<LynxModule> hubs;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        laucnhLeft = new Motor(hardwareMap, "launchLeft", Motor.GoBILDA.RPM_435);
        launchRight = new Motor(hardwareMap, "launchRight", Motor.GoBILDA.RPM_435);
        myMotors = new MotorGroup(laucnhLeft,launchRight);
        myMotors.setRunMode(Motor.RunMode.RawPower);
        hubs = hardwareMap.getAll(LynxModule.class);

    }
    public void runLauncher(){
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        if(gamepad2.left_trigger > 0.2){
            myMotors.set(1.0);
        }
        if(gamepad2.right_trigger > 0.2){
            myMotors.set(-1.0);
        }
        if(gamepad2.left_trigger < 0.25|| gamepad2.right_trigger < 0.25){
            myMotors.stopMotor();
        }
        if(gamepad2.x){
            myMotors.stopMotor();
        }
    }
}
