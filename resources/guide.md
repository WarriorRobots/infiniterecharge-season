# Guide

This is a guide to writing robot code for the robot.

## Table of Contents

- [To-Do](#To-Do)
- [Setting up your environment](#Setting-up-your-environment)
- [Reading material](#Reading-material)
- [Visual Studio Code](#Visual-Studio-Code)
- [Github Desktop](#Github-Desktop)
- [Tip](#Tips)

## To-Do

[Here](https://github.com/WarriorRobots/infiniterecharge-season/projects/1) you can find the To-Do list. It has 3 types of tabs: To do, In progress, and Done. Make sure to add things to the list as they need to be done and move them to in progress and done when appropriate.

To do: Items that need to be done will be shown here.

In progress: There is a In Progress tab for each person who is programming. This is where you put cards of what you are working on (so other people don't do what you are trying to do).

Done: After finishing a task, move it here.

## Setting up your environment

### Git Bash download

Download Git Bash from [here](https://git-scm.com/downloads). When it presents options, continually hit next.

(You will not normally need to use Git Bash but is so if you run into issues, you have a powerful termial for git to solve issues.)

### Visiual Stuido from WPI download

[Here](http://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html) are the instructions on how to set up your Visual Studio Code environment (and uninstall it).

TL;DR: Chose current user for where to install. Choose `Select/Download VS Code` and then `Download`. After that downloads do `Execute Install`. Finally hit `OK`.

### Github.com

On [Github.com](github.com), make an account (or sign in). This is important for editing the [To-Do](#To-Do) and pushing code on [Github Desktop](#Github-Desktop) and Git Bash.

### Github Desktop download

Download Github Desktop from [here](https://desktop.github.com/).
Then log in (you can leave the email commit as is).

## Reading material

<!-- TODO Update this list of material -->
(No current material is available to read over as it is out of date.)

## Visual Studio Code

Use to edit the robot code (and any other files).

Triple Dots (only works when a .java file is open):

- `Build Robot Code`: builds the robot code (use before trying to flash to check for compilation errors and download vendor dependencies).
- `Deploy Robot Code`: builds and flashes the code to the RoboRIO.
- `Start Tool`: Use to open PathWeaver.

Command palette: `CTRL+SHIFT+P`

- `Developer: Reload Window`: reloads Visual Stuidio.
- `Preferences: Color Theme`: changes the color of the window.
- `WPILib: Manage Vendor Libraries`: use to download/manage 3rd party code dependencies. Installed code goes into vendordeps folder.
- `Markdown: Open Preview to the Side`: **requires markdownlint**, opens a preview of a markdown file.
- `WPILib: Create a new project`: creates a new wpi project.
- `WPILib: Open API Documentation`: Opens the WPI Java API.

Color coding (of code and files in the tree):

- Red:
  - Tree/File: Means the code has some error.
  - Tree (folder): A file in this folder was deleted.
- Yellow:
  - Tree: File is modified or has a warning.
  - File: Line has a warning.
- Green:
  - Tree: File is untracked.
- Blue:
  - Tree: Git conflic is present
  - File: Bookmark to come back to.

## Github Desktop

Use Github Desktop to keep code separated; the point of this is to keep code clean and not contaminate different comonents with each other (Ex. the unstable turret code should **not** be mixed with intake code). This is solved by using branches. Branches allow the code to be separate while retaining a core idea of changes. This allows for all changes pertaining to one particular thing stay together and are not interfering with other code.

Do not try to write code to the master branch, it will physically push code if you are not on the programming laptop.

Commits should have a Title/Summary and Description. The Title/Summary should give an idea of what was changed. The Descritption should describe **what** was changed and **why** it was changed (if the why is not infered from the what).

Make sure to sync (fetch origin/pull/push) before editing your code and after making a commit.

## Tips

- Save early; Save often.
- Stuck? Ask someone, check any local resources (like this), then search [Google](https://google.com).
- [WPI Lib](http://docs.wpilib.org/en/latest/) is **heavily** documented. There is plenty of information about how to do things and don't be afraild to look at old code.
- If all the code seems to have errors/issues, do `CRTL+SHIFT+P` and type `Developer: Reload Window`.
