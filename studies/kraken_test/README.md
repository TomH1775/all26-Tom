# Kraken Test

This is a project that anyone on the team should be able to use
to run motors for "alpha" or "beta" level hardware.

To use it, follow these steps:

1. Set up a laptop.
   * Get one of the team Windows laptops from the locked cabinet.
   * Find an AC power adapter and use it.
   * Turn it on.  The Windows password is written on it: "wildhats".
   * Start a browser and make sure it works.
1. Make an account on github, if you don't already have one.
   * Using a browser, go to https://github.com/signup
   * Follow the directions.
1. Fork the team repository.
   * Using a browser, go to https://github.com/
   * Sign in, using your github account.
   * Go to https://github.com/team100/all26
   * Click the "Fork" button near the upper right
   * This makes a copy on github, for your use. 
1. Set up Github Desktop.
   * Start the app called "Github Desktop".
   * Go to "File >> Options" and click "Sign Out"
   * Sign in with your github account.
   * Go to "File >> Clone repository ..." and find all26 in the list
   * Save it in a folder on the desktop, with your name. 
1. Set up a test board
   * Clear a workspace of a few square feet
      * Do not pile your work on top of something else
   * Find a board with a PDP and a RoboRIO in the storeroom
   * Plug your motor power wires in to the PDP
   * Plug your motor CAN wires into the RoboRIO CAN ports.
   * Plug in a freshly charged battery
   * Connect your laptop to the RoboRIO, using a USB-A to USB-B cable
1. Set up the Driver Station
   * On the desktop, click "FRC Driver Station"
      * It often also pops up a dashboard app.  Close that.
   * In the Driver Station, click the little gear and choose "Console"
      * This pops up a test window showing output from the robot code.
1. Set up VsCode.
   * On the desktop, click "2026 WPILib VS Code"
      * Do not use the Windows search feature to find vscode
   * Go to "File >> Open Folder..." and navigate to "kraken_test"
   * VsCode will start trying to build the project
      * This can take a long time (many minutes)
   * Go to "File >> Auto Save" and make sure it's checked.
1. Change the code if desired
   * In VsCode, navigate to Robot.java and edit as desired
   * Click the WPILib extension button and choose "Test Robot Code"
      * You will need help to find this button the fist time
   * Look at the output in the "Terminal" pane at the bottom
      * You will need help to interpret the terminal output the first time.
1. Deploy code
   * Turn on the testboard, using the main breaker
   * After awhile, the Driver Station should indicate it is connected.
   * In VsCode, click the WPILib extension button and choose "Deploy Robot Code"
      * You will need help to find this button the fist time
   * The robot code will restart, and the connected motors will jingle
   * Look at the Console window to see if there are any errors
      * You will need help to interpret the console the first time
1. Enable the test
   * Make sure everyone's hands are clear of the moving parts
   * Say "Enabling" in a loud voice, and make sure everyone nearby hears you.
   * Put your finger over the "Enter" key on the laptop keyboard
      * Alternatively, a second person can do this job
      * The "Enter" key disables the test.
   * In the Driver Station, click "Enable"
   * The motors will run, as specified in the code
   * If anything bad happens, press "Enter".

If you haven't changed anything, that's the whole process.
You can repeat the "Change - Deploy - Enable" steps as many
times as you like.

If you have made changes, you should save them:

1. Check in your changes
   * In VsCode, navigate to the source control pane
      * You will need help to find this the first time
   * Make sure the changes make sense
      * Ask for help the first time
   * Click the "+" near the top of the pane
   * Add a description of your changes
   * Click "Commit"
1. Push your changes to your fork
   * Switch to the Github Desktop app
   * Click "Push Origin"
1. Submit your changes for review
   * Using a browser, go to your fork on github.com
   * Click "Contribute"
   * Add a description
   * Click "Create pull request"

That's it!

