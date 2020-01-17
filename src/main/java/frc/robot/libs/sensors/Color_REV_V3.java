package frc.robot.libs.sensors;

import java.util.HashMap;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Color_REV_V3  extends ColorSensorV3{
    protected final HashMap<String,Color> colors = new HashMap<>(); // Storage the colors in a HashMap
    protected final ColorMatch colorMatch = new ColorMatch(); // Class which operate the colors properties
    /**
     * Create the Object, use this always.
     * @param source define what port is the Sensor
     */
    public Color_REV_V3(I2C.Port source) {
        super(source);
    }
    /**
     * Set diferents colors on a HashMap to verify posteriorly
     * @param colors You have two options:
     * <li> The first is to use a matrix with a String in the index 0 and a Color in the index 1
     * <li> The second is to use a matrix with a String in the index 0 and 3 double what means a color data
     */
    public void setColors(Object[][] colors) {
        if(colors[0].length!=4){ //if the length are equal 4 it means the array is a double and means a {@link Color}, 
                                //if there aren't 4 it means that is a 1 object Color
            try{
                for(Object[] color : colors){ //split the matrix in arrays of object 
                    colorMatch.addColorMatch(ColorMatch.makeColor((double)color[1], (double)color[2], (double)color[3]));//add a color to the colorMatch
                    this.colors.put((String)color[0], ColorMatch.makeColor((double)color[1], (double)color[2], (double)color[3]));//add a color to the HashMap with the name String
                }
            }
            catch(ArrayIndexOutOfBoundsException ex){
                System.out.println("Use a vector with a size of 3"); // if you are here means the data enter are part Colors and part double
            }
            return;
        }
        for (Object[] color : colors) { //split the matrix in array of object
            this.colors.put((String)color[0], (Color)color[1]); //add a color to the HashMap with the name String
            colorMatch.addColorMatch((Color)color[1]); //add a color to the colorMatch
        }
    }
    /**
     * Return the closest color set in the storage data.
     * @return Return a Object array with the name of the color and the object Color matching, 
     * if the actual value of the color doesn`t match with any storaged then return a void Object array.
     */
    public Object[] getActualColorArray(){
        Color color = colorMatch.matchClosestColor(getColor()).color; //get the actual color from sensor
        for (HashMap.Entry<String,Color> entry : colors.entrySet()) { //split the HashMap
            if(entry.getValue() == color) 
                return new Object[] {entry.getKey(), entry.getValue()}; //if the color is equal to any in the storage then return the value of it
        }
        return new Object[0]; //if you don't have any equals then return a void Object array
    }

    /**
     * Return the name of the actual color
     * @return Return the name of the actual color in the sensor,
     * if the color doesn't match with any storage, then return a void String.
     */
    public String getActualColorName(){
        Color color = colorMatch.matchClosestColor(getColor()).color;//Get the actual color from the sensor
        for (HashMap.Entry<String,Color> entry : colors.entrySet()) { //Split the array
            if(entry.getValue() == color)
                return entry.getKey();//if the value is storaged, then return the name of the color
        }
        return "";//return a empty String if the value isn't storaged
    }

}