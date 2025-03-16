IS A WORK IN PROGRESS

Notes:
Pov = d-pad on the controller; “pov” is how the d-pad is indicated for the Xbox controller on WPIlib
SafetySubsystem: this is code that coordinates the movements of the arm and elevator to ensure that the arm does not move when the elevator is below a certain threshold (called the “danger zone”). This prevents the robot from damaging itself (ex. The arm rotates at too low of a height and snaps the arm.)



Robot Container

  Safety subsystem default command → stow
    Returns to stow if no button is held
    
  Right trigger → CoralIntakeL2AlgaeCommand
    NOTE: AKA coral pick-up command. The name is funky because it was also going to be used to intake the L2 algae, but this idea was scrapped.
    Runs sequence while button is held
      1. Move arm + elevator to target via safety subsystem
      2. Arm at target → coral intake start
      3. Once coral sensed via CANRange, move to stow but keep intaking for certain time (HOLD_BACK_DURATION in the command) before setting intake to hold speed (HOLD_SPEED in CoralIntake subsystem) 
        NOTE: Hold speed currently set to 0
        
  LeftTrigger → m_algaeIntake.reverse()
    Sets the algae intake to reverse while button is held 
    
  Pov UP → L4 Score Command
    This runs a sequence on pov up = true (not a button hold)
      1. Set elevator height to L4, set arm angle to stow (0 degrees)
      2. Once elevator is at the target height, rotate the arm to L4
      3. Once the arm is at target to score, outtake coral intake
      4. After “scoring time” has elapsed, move the arm back to 0 degrees (arm value for stow)
      5. Once arm is at zero, stop coral intake and bring elevator to stow
      6. Once currentState = done, the command is finished and the default command resumes application (also breaks out of command if it is interrupted)
      
  Pov Left → L3 Score Command
    Runs when pov left = true (not a button hold)
      1. Set elevator height to L3, set arm angle to stow (0 degrees)
      2. Once elevator is at the target height, rotate the arm to L3
      3. Once the arm is at target to score, outtake coral intake
      4. After “scoring time” has elapsed, move the arm back to 0 degrees (arm value for stow)
      5. Once arm is at zero, stop coral intake and bring elevator to stow
      6. Once currentState = done, the command is finished and the default command resumes application (also breaks out of command if it is interrupted)
  
  
