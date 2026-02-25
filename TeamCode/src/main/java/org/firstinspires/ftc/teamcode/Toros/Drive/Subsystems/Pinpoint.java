package org.firstinspires.ftc.teamcode.Toros.Drive.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;

public class Pinpoint{
    public static class Params {
        public double parYTicks =  -648.1950127810029   ; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -3692.469269014993; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    public double inPerTick = 0.00194723378259707 ;

    public Pinpoint(HardwareMap hardwareMap){
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();
    }
    public double getHeading(){
        return driver.getHeading(AngleUnit.RADIANS);
    }

    public double getVelx(){
        return driver.getVelX(DistanceUnit.INCH);
    }


    public double getVelY(){
        return driver.getVelY(DistanceUnit.INCH);
    }
}

