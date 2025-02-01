package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;

/** Class to pick an auto from a multitude of 'folders' from Shuffleboard */
public class AutoPicker {
    Command mCurrentCommand;
    String mCurrentName;
    HashMap<String, SendableChooser<Command>> mChoosers;

    public AutoPicker() {
        mChoosers = new HashMap<>();
        AutoConstants.kAutoFolders.forEach(this::setupChooser);
    }

    /** Initializes any manually user entered commands */
    public void initializeCommands(String folder, Command... commands) {
        for (Command command : commands) {
            mChoosers.get(folder).addOption(command.getName(), command);
        }
    }

    /**
     * Sets up each of the sendable choosers per each folder
     * @param folderName Name of the folder to get command names from
     * @param names List of command names
     * @see List
     */
    private void setupChooser(String folderName, List<String> names) {
        var chooser = new SendableChooser<Command>();
        mChoosers.putIfAbsent(folderName, chooser);

        // creates default option, to avoid issues with folders containing only one command
        var noneCommand = new InstantCommand();
        noneCommand.setName("None");
        chooser.setDefaultOption("None", noneCommand);
        
        names.forEach((name) -> createChooserOptions(chooser, name));
        chooser.onChange((command) -> setCurrentCommand(chooser));
        
        SmartDashboard.putData(folderName, chooser);
    }

    /**
     * Creates options for each of the commands using AutoBuilder and places them in their respective chooser
     * @param chooser Sendable chooser to create options for
     * @param name Name of the command option 
     * @see SendableChooser
     * @see Command
     * @see PathPlannerAuto
     */
    private void createChooserOptions(SendableChooser<Command> chooser, String name) {
        var command = new PathPlannerAuto(name);
        command.setName(name);
        
        chooser.addOption(name, command);
    }

    /**
     * Called any time the chooser is changed and sets current command to 
     * the selected command from whichever chooser has been changed
     * @param chooser Chooser to get selected command from
     * @see SendableChooser
     */
    private void setCurrentCommand(SendableChooser<Command> chooser) {
        mCurrentCommand = chooser.getSelected();
        SmartDashboard.putString("Current Auto", mCurrentCommand.getName());
    }

    /**
     * Gets the selected active command from the auto folders.
     * @return Selected Command
     * @see Command
     */
    public Command getSelected() {
        return mCurrentCommand;
    }
}
