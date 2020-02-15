/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs.logger;

import java.io.File;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;


/**
 * Class for logging information on the roborio FTP Server
 */
public class Logger {
    //#region VARIABLES
    private PrintWriter log;
    private String irl = "/home/lvuser/Logs/";
    private DateTimeFormatter dtf = DateTimeFormatter.ofPattern("HH:mm:ss");
    //#endregion  

    //#region CONSTRUCTOR
    /**
     * Constructor for the logger class
     */
    public Logger() {
        // FILE
        File file = new File(irl);
			if (!file.exists()) {
				if (file.mkdir()) {
					System.out.println("Log Directory is created!");
				} else {
					System.out.println("Failed to create Log directory!");
				}
            }
        // LOGGER
		try {
            log = new PrintWriter(irl + DateTimeFormatter.ofPattern("yyyy-MM-dd").format(LocalDateTime.now()) + "-Log.log");
            log.println(" STARTED ");
            log.flush();
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
    }
    //#endregion

    //#region LOG (SINGLE)
    /**
     * Command to log information on the file
     * 
     * @param text The text to be logged in the file, only one item.
     */
    public void Log(String name, Object text) {
        log.println("name: " + name + 
            "info:" + text.toString() + 
            "time: "+ dtf.format(LocalDateTime.now()));
        log.flush();
    }
    //#endregion

    //#region LOG (MULTI)
    /**
     * Command to log information on the file
     * 
     * @param listData The text to be logged in the file in a for of hash map <p>
     * which the key consists of the item name and it`s value is the data it`s sending.
     */
    public void Log(HashMap<String,Object> listData) {
        for(HashMap.Entry<String, Object> entry : listData.entrySet()){ 
            log.println("name: " + entry.getKey() + 
                "info:" + entry.getValue().toString() + 
                "time: "+ dtf.format(LocalDateTime.now()));
            log.flush();
        }
    }
    //#endregion

    //#region END
    /**
     * Use this function to end the call of the logger
     */
    public void end() {
        log.println(" ENDED ");
        log.flush();
        log.close();
    }
    //#endregion
}
