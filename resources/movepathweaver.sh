# Moves the output from PathWeaver to be deployed on the robot

# This file only works when it is in the resources folder of the robot project

# robot project root folder
robot=${0%/*}/..

# remove old paths
rm -r $robot/src/main/deploy/paths

# move new paths to the deploy folder
cp -r $robot/PathWeaver/output $robot/src/main/deploy/paths

echo "Paths have been moved."