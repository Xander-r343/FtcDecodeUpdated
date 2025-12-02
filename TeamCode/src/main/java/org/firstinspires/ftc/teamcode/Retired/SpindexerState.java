package org.firstinspires.ftc.teamcode.Retired;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class SpindexerState {

    public Spindexer.color slotZero;

    public Spindexer.color slotOne;
    public Spindexer.color slotTwo;
    public SpindexerState(){

    }
    public void setSlotState(int slot, Spindexer.color givenState){
        if(slot == 0){
            slotZero = givenState;
        }
        else if(slot == 1){
            slotOne = givenState;
        }
        else if(slot == 2){
            slotTwo = givenState;
        }
    }
    public int getSlotFromColor(Spindexer.color givenColor){
        if(slotZero == givenColor && slotZero != Spindexer.color.UNDECTED){
            return 0;
        }
        else if(slotOne == givenColor && slotOne != Spindexer.color.UNDECTED){
            return 1;
        }
        else if(slotTwo == givenColor && slotTwo != Spindexer.color.UNDECTED) {
            return 2;
        }
        else{
            return 2;
        }
    }
    public boolean containsColorInSpindexer(Spindexer.color givenColor){
        if(slotZero == givenColor){
            return true;
        }
        else if(slotOne == givenColor){
            return true;
        }
        else if(slotTwo == givenColor){
            return true;
        }
        else{
            return false;
        }
    }



}
