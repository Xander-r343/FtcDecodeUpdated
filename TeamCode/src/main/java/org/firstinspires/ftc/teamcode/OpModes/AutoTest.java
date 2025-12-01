package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Sensors.OdoPods;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

public class AutoTest extends OpMode {
    MecanumDrivetrain drivetrain;
    OdoPods pods;
    public double speed = 0.5;

    public double pos1x = 0, pos1y= 10 ,pos1h = 0;

    public double pos2x=0, pos2y=0, pos2h=0;

    public enum States{
        Init,Position1, Position2, Stop
    }
    States currentState = States.Init;


    @Override
    public void init() {
        drivetrain = new MecanumDrivetrain(speed, hardwareMap);
        pods = new OdoPods(hardwareMap, drivetrain);
    }

    @Override
    public void loop() {
        StateMachine();


    }

    public void StateMachine(){
        switch (currentState){

            case Init:
                currentState = States.Position1;
            break;


            case Position1:
                if(moveToPos1()){
                    currentState = States.Position2;
            }
            break;


            case Position2:
                if(moveToPos2()){
                    currentState = States.Stop;
                }
            break;


            case Stop:

            break;

        }
    }

    public boolean moveToPos1(){
       return pods.holdPosition(pos1x,pos1y,pos1h,0.8);
    }

    public boolean moveToPos2(){
        return pods.holdPosition(pos2x,pos2y,pos2h,0.8);
    }
}
