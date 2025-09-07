package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    MotorGroup myMotors;
    Gamepad gamepad2;

    public Launcher(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad2 = gamepad;
        MotorGroup myMotors = new MotorGroup(new Motor(hardwareMap, "flyWheel1"),
                new Motor(hardwareMap, "flyWheel2"));

        myMotors.setRunMode(Motor.RunMode.RawPower);
    }
    public void runLauncher(){
        if(gamepad2.left_trigger > 0.2){
            myMotors.set(1);
        }
        if(gamepad2.right_trigger > 0.2){
            myMotors.set(-1);
        }
    }
}
