IS A WORK IN PROGRESS; please view as code becasue the formatting gets messed up in preview mode

  Notes:
  
  This is the current state of the code, and is not ideal. This gave me a better idea of what is going on, and as I fix problems, I will try to update this file to better reflect the code.
  
  Pov = d-pad on the controller; “pov” is how the d-pad is indicated for the Xbox controller on WPIlib
  
  SafetySubsystem: this is code that coordinates the movements of the arm and elevator to ensure that the arm does not move when the elevator is below a certain threshold (called the “danger zone”). This prevents the robot from damaging itself (ex. The arm rotates at too low of a height and snaps the arm.)


--------------------------------------------------------------------------------------------------------------------------

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
      6. Once currentState = done, the command is finished and the default command resumes application (if command is interupted, goes to stow and then resumes default commmand)
      
  Pov Left → L3 Score Command
    Runs when pov left = true (not a button hold)
      1. Set elevator height to L3, set arm angle to stow (0 degrees)
      2. Once elevator is at the target height, rotate the arm to L3
      3. Once the arm is at target to score, outtake coral intake
      4. After “scoring time” has elapsed, move the arm back to 0 degrees (arm value for stow)
      5. Once arm is at zero, stop coral intake and bring elevator to stow
      6. Once currentState = done, the command is finished and the default command resumes application (if command is interupted, goes to stow and then resumes default commmand)

  Pov Down → L2 Score Command
      Runs when pov down = true (not a button hold)
      1. Moves arm to L2 position first
      2. After a certain time has elapsed, set the elevator to L2 height
      3. Once both are at target, begin outaking coral intake
      4. After certain time has elapsed, stop outaking and return to stow
      5. Once currentState = done, the command is finished and the default command resumes application (if command is interupted, goes to stow and then resumes default commmand)
  
  Pov Right → ArmElevatorToPositionCommand to stow position
    Runs when pov right = true (not a button hold)
    1. Goes to set position (in this case, the stow position) via the safety subsystem.
    2. Ends command when saftey is at target

  Back → ArmClimbPositionCommand
    Runs when back = true (not a button hold)
    1. Goes to stow via safety subsystem
    2. Once safety is at target (double checks this), set the arm to the climb position (NOT VIA SAFETY)
    3. Once arm is at target, set elevator height to climb position (NOT VIA SAFETY)
    4. Once at target, does not break out of command (therefore holds position) (if interupted, goes to stow)

  x → L2AlgaeCommand
    Runs when x = true (not a button hold)
    1. Set to L2 position via safety subsytem
    2. If at target, runs algae intake and maintains posistion once ball is held
    3. If it is interupted...
      a) If no ball, stops intake
      b) If ball, continue holding ball (still intaking to hold ball)

  
