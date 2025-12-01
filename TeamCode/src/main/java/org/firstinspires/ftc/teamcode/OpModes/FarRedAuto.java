package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Configs.RedAutoPaths;
import org.firstinspires.ftc.teamcode.Sensors.OdoPods;
import org.firstinspires.ftc.teamcode.Subsystems.Aimbots;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
@Autonomous(name = "9 ball auto red")
public class FarRedAuto extends LinearOpMode {

    Servo rgb;
    ShooterSubsystem robotSubsystem;
    OdoPods pods;
    MecanumDrivetrain drivetrain;
    RedAutoPaths pathDatabase;
    Object Xpos;
    Object Ypos;
    Object Headingpos;
    Object Alliance;
    Config config;
    int AutoState;
    ElapsedTime timer;
    double speed;
    double time;
    Aimbots aimbots;
    private ElapsedTime runtime = new ElapsedTime();
    int tagID;
    int flywheelSpeed;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(1, hardwareMap);
        pods = new OdoPods(hardwareMap, drivetrain);
        pathDatabase = new RedAutoPaths();
        config = new Config();
        robotSubsystem = new ShooterSubsystem(hardwareMap);
        robotSubsystem.setServoPosition(0);
        //intialize blackboard objects
        Xpos = blackboard.getOrDefault(config.Xkey, 0);
        Ypos = blackboard.getOrDefault(config.Ykey, 0);
        Headingpos = blackboard.getOrDefault(config.HeadingKey, 0);
        Alliance = blackboard.getOrDefault(config.AllianceKey, 0);//isRedValue
        blackboard.put(config.AllianceKey, config.BlueAlliance);
        AutoState = 0;
        //initialize odopods by using the config class
        pods.setPosition(88, 9, 0);
        timer = new ElapsedTime();
        AutoState = 0;
        aimbots = new Aimbots(config.RedAlliance, pods, hardwareMap);
        rgb = hardwareMap.get(Servo.class, config.RBGName);
        flywheelSpeed = 2600;
        waitForStart();
        runtime.reset();
        timer.reset();
        while(opModeIsActive()){
            rgb.setPosition(config.Green);
            aimbots.update();
            switch (AutoState){
                case 0:{
                    robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                    aimbots.startLL();
                    aimbots.switchPipeline(0);
                    robotSubsystem.spinIntake(1);
                    timer.startTime();
                    //go to launch position and use aimbots to align to goal
                    pods.holdPosition(88, 90, 0, 0.8);
                    pods.update();
                    //start flywheel early to get up to speed and keep it
                    if (timer.seconds() < 2) {
                        pods.holdPosition(88, 90, 0, 0.8);
                        pods.update();
                    }

                    if(timer.seconds() > 2){
                        AutoState = 1;
                    }
                }
                break;
                case 1:{
                    //align for 1 secs, ended with 2
                    while(timer.seconds() < 3.2) {
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        if (aimbots.LLstatusIsValid()) {
                            targetH = pods.getHeading() - aimbots.getHeadingErrorLL();
                        } else {
                            targetH = aimbots.getIdealAngle()-9;
                        }
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                        if(timer.seconds() > 3.1){
                            AutoState = 2;
                        }
                    }

                }
                break;
                case 2:{
                    double speed = 0.4;
                    robotSubsystem.setServoPosition(1);
                    //fire the first 3 artifacts
                    while(timer.seconds() < 7) {
                        robotSubsystem.spinIntake(speed);
                        robotSubsystem.spinBelt(0.4);

                        if(timer.seconds() > 6.8){
                            shutOff();
                            robotSubsystem.setServoPosition(0);
                            AutoState = 3;
                            robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                            robotSubsystem.spinIntake(1);
                        }
                        else if(timer.seconds() > 4 && timer.seconds() < 5){
                            robotSubsystem.setServoPosition(0);
                        }
                        if(timer.seconds() > 5 && timer.seconds() < 6.7){
                            robotSubsystem.setServoPosition(1);
                        }
                        speed -= 0.0003;
                    }
                }
                break;
                case 3:{
                    pods.holdHeading(90,1);
                    pods.update();
                    //strafe to align with pickup balls
                    while(timer.seconds() < 8.5){
                        pods.holdPosition(88,95,90,1);
                        pods.update();
                    }
                    //speed up flywheel early
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.spinBelt(0.4);
                    timer.reset();
                    timer.startTime();
                    //pickup 2 more balls from far pickup
                    while(timer.seconds() < 2.1){
                        //TODO adjust this 115 x for 3rd ball if necessary
                        pods.holdPosition(125,95,90,0.5);
                        pods.update();
                    }
                    while(timer.seconds() < 2.8) {
                        //move back to fire
                        pods.holdPosition(76, 100, 90, 1);
                        pods.update();
                        if(timer.seconds() > 2.7){
                            AutoState = 4;
                        }
                    }
                    robotSubsystem.spinIntake(1);
                }
                break;
                case 4:{
                    //align to goal using limelight
                    while(timer.seconds() < 4) {
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        if (aimbots.LLstatusIsValid()) {
                            targetH = pods.getHeading() - aimbots.getHeadingErrorLL();
                        } else {
                            targetH = aimbots.getIdealAngle()-5;
                        }
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                    }
                    //fire balls
                    while(timer.seconds() < 6.7) {
                        robotSubsystem.setServoPosition(1);
                        robotSubsystem.spinBelt(0.4);
                        if(timer.seconds() > 6.6){
                            shutOff();
                            AutoState = 5;
                            robotSubsystem.setServoPosition(0);
                        }
                    }
                }
                break;
                case 5:{
                    while(timer.seconds() < 8) {
                        pods.holdHeading(90, 1);
                        pods.update();
                    }
                    robotSubsystem.spinIntake(0.7);
                    //prepare for another pickup
                    while(timer.seconds() < 10){
                        pods.holdPosition(95,72,90,1);
                        pods.update();
                    }
                    AutoState = 6;
                }
                case 6:{
                    timer.reset();
                    timer.startTime();//
                    robotSubsystem.spinIntake(1);
                    robotSubsystem.spinBelt(0.6);
                    while(timer.seconds() < 1.6){
                        pods.holdPosition(120, 72, 90, 0.5);
                        pods.update();
                    }
                    AutoState = 7;
                }
                break;
                case 7: {
                    while (timer.seconds() < 3.6) {
                        pods.holdPosition(82, 104, 90, 1);
                        pods.update();
                    }
                    robotSubsystem.setFlywheelVelocity(flywheelSpeed);
                    while(timer.seconds() < 4.6){
                        double targetH;
                        //align using limelight and pods to shoot in goal
                        if (aimbots.LLstatusIsValid()) {
                            targetH = pods.getHeading() - aimbots.getHeadingErrorLL();
                        } else {
                            targetH = aimbots.getIdealAngle()-8;
                        }
                        pods.holdHeading(targetH, 1);
                        aimbots.update();
                        if(timer.seconds() > 3.7){
                            AutoState = 2;
                        }
                    }
                    AutoState = 8;
                }
                break;
                case 8:{
                    while(timer.seconds() < 7.4) {
                        robotSubsystem.spinIntake(0.4);
                        robotSubsystem.setServoPosition(1);
                        robotSubsystem.spinBelt(0.4);
                    }
                    pods.holdPosition(110, 80, 90, 1);
                    pods.update();
                }
                break;
            }
            blackboard.put(config.Xkey, pods.getX());
            blackboard.put(config.Ykey, pods.getY());
            blackboard.put(config.HeadingKey, pods.getHeading());
        }
    }
    public void resetArtifacts(){
        robotSubsystem.setFlywheelVelocity(-300);
        robotSubsystem.spinBelt(-0.2);
        robotSubsystem.spinIntake(1);
    }
    public void shutOff(){
        robotSubsystem.spinBelt(0);
    }
}

