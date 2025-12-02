package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.color.GREEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.color.PURPLE;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.color.UNDECTED;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Retired.SpindexerState;

import java.util.Arrays;
import java.util.HashMap;

public class Spindexer {
    private Servo starboardServo;
    private Servo portServo;
    private ColorSensor slot0;
    private ColorSensor slot1;
    private ColorSensor slot2;
    public SpindexerState spindexerState;
    public enum color{
        PURPLE, GREEN, UNDECTED
    }
    Config config;
    ///          |
    ///          |
    ///       1  o    2
    ///        /   \
    ///       / 0   \
    ///     /        \ slots are unmoving
    ///      intake side
    ///        firing side

    public Spindexer(HardwareMap hardwareMap){
        config = new Config();
        //initialize servos
        starboardServo = hardwareMap.get(Servo.class, config.stbSv);
        portServo = hardwareMap.get(Servo.class, config.ptSv);
        //initialize color sensors
        slot0 = hardwareMap.get(ColorSensor.class, config.sl0);
        slot1 = hardwareMap.get(ColorSensor.class, config.sl1);
        slot2 = hardwareMap.get(ColorSensor.class, config.sl2);
        slot0.enableLed(true);
        slot1.enableLed(true);
        slot2.enableLed(true);
        spindexerState = new SpindexerState();
    }
    /**
     * spins the spindexer to a slot
     * @param givenSlot is a number between 0-2 and is the wanted spindexer slot
     */
    public void spinToPose(int givenSlot){
        if(givenSlot == 0){
            starboardServo.setPosition(config.slot0ServoPosition);
            portServo.setPosition(1-config.slot0ServoPosition);
        }
        else if(givenSlot == 1){
            starboardServo.setPosition(config.slot1ServoPosition);
            portServo.setPosition(1-config.slot1ServoPosition);
        }
        else if(givenSlot == 2){
            starboardServo.setPosition(config.slot2ServoPosition);
            portServo.setPosition(1-config.slot2ServoPosition);
        }
        else{
        }
    }
    /**
     * returns the color value
     * @param givenSlot specified slot to read
     * @return the color enum
     */
    public color readSlotColor(int givenSlot) {
        color toReturn = color.UNDECTED;
        if (givenSlot == 0) {
            double redValue = slot0.red();
            double greenValue = slot0.green();
            double blueValue = slot0.blue();
            if (blueValue > redValue && blueValue > greenValue) {
                // Detected purple (adjust thresholds as needed)
                toReturn = color.PURPLE;
            } else if (greenValue > redValue && greenValue > blueValue) {
                // Detected green (adjust thresholds as needed)
                toReturn  = GREEN;
            }
            else{
                toReturn  = color.UNDECTED;
            }
        }
        else if (givenSlot == 1) {
            double redValue = slot1.red();
            double greenValue = slot1.green();
            double blueValue = slot1.blue();
            if (blueValue > redValue && blueValue > greenValue) {
                // Detected purple (adjust thresholds as needed)
                toReturn = color.PURPLE;
            } else if (greenValue > redValue && greenValue > blueValue) {
                // Detected green (adjust thresholds as needed)
                toReturn  = GREEN;
            }
            else{
                toReturn  = color.UNDECTED;
            }
        }
        else if (givenSlot == 2) {
            double redValue = slot2.red();
            double greenValue = slot2.green();
            double blueValue = slot2.blue();
            if (blueValue > redValue && blueValue > greenValue) {
                // Detected purple (adjust thresholds as needed)
                toReturn = color.PURPLE;
            } else if (greenValue > redValue && greenValue > blueValue) {
                // Detected green (adjust thresholds as needed)
                toReturn  = GREEN;
            }
            else{
                toReturn  = color.UNDECTED;
            }
        }
        return toReturn;
    }
    public boolean slotIsFull(int givenSlot){
        return readSlotColor(givenSlot) != color.UNDECTED;
    }
    /**
     * returns the ideal routing of slots to get pattern/motif
     * @param givenFirstBall first ball color in the motif
     * @param givenSecondBall second ball color in motif
     * @param givenThirdBall etc.
     * @return int [] of the slots
     */
    public int[] determineBestSpinRoute(@NonNull color givenFirstBall, @NonNull color givenSecondBall, @NonNull color givenThirdBall){
        int[] order = new int[2];
        spindexerState.setSlotState(0, readSlotColor(0));
        spindexerState.setSlotState(1, readSlotColor(1));
        spindexerState.setSlotState(2, readSlotColor(2));

       //check if the sorter contains the wanted color
            if(spindexerState.containsColorInSpindexer(givenFirstBall)) {
                //check the sorter for color
                order[0] = spindexerState.getSlotFromColor(givenFirstBall);
                spindexerState.setSlotState(order[0], UNDECTED);
            }
            //if the sorter doesn't contain the wanted color then use opposite color ball
            else if(spindexerState.containsColorInSpindexer(oppositeBall(givenFirstBall))){
                //check the sorter for color
                order[0] = spindexerState.getSlotFromColor(oppositeBall(givenFirstBall));
                spindexerState.setSlotState(order[0], UNDECTED);
            }
            //second ball
            if(spindexerState.containsColorInSpindexer(givenSecondBall)) {
                //check the sorter for color
                order[1] = spindexerState.getSlotFromColor(givenSecondBall);
                spindexerState.setSlotState(order[1], UNDECTED);
            }
            //if the sorter doesn't contain the wanted color then use opposite color ball
            else if(spindexerState.containsColorInSpindexer(oppositeBall(givenFirstBall))){
                //check the sorter for color
                order[1] = spindexerState.getSlotFromColor(oppositeBall(givenFirstBall));
                spindexerState.setSlotState(order[1], UNDECTED);
            }
            //add safe for if color is undetected
            else{
                order[1] = Integer.parseInt(null);
            }
            //last ball
            if(spindexerState.containsColorInSpindexer(givenThirdBall)) {
             //check the sorter for color
                 order[2] = spindexerState.getSlotFromColor(givenThirdBall);
                spindexerState.setSlotState(order[2], UNDECTED);
            }
            //if the sorter doesn't contain the wanted color then use opposite color ball
            else if(spindexerState.containsColorInSpindexer(oppositeBall(givenThirdBall))){
                 //check the sorter for color
                order[2] = spindexerState.getSlotFromColor(oppositeBall(givenThirdBall));
                 spindexerState.setSlotState(order[2], UNDECTED);
            }
            //add safe for if color is undetected
            else{
                order[2] = Integer.parseInt(null);
            }
            //return oder in slot #'s with int []
        return order;
    }
    private color oppositeBall(color currentBall){
        if(currentBall == GREEN){
            return PURPLE;
        }
        else if(currentBall == PURPLE){
            return GREEN;
        }
        else{
            return UNDECTED;
        }

    }
}
