#!/bin/bash

# Number of iterations
NUM_ITERATIONS=2

# Paths
SOURCE_DIR="/home/aminghanbarzadeh/Cheetah-Software-Vision/user/MIT_Controller/Controllers/convexMPC"
BUILD_DIR="/home/aminghanbarzadeh/Cheetah-Software-Vision/build"
SAVE_DIR="/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN"
YAML_FILE="/home/aminghanbarzadeh/Cheetah-Software-Vision/config/default-terrain.yaml"

# GUI coordinates (adjust based on your setup)
START_BUTTON_X=247
START_BUTTON_Y=174
BOX_X=771
BOX_Y=191
TAB_BUTTON_X=472
TAB_BUTTON_Y=1068

# Signal file
SIGNAL_FILE="/tmp/robot_position_reached"
rm -f $SIGNAL_FILE

# Increment value
INCREMENT=0.0
INCREMENT1=0.0
INCREMENT2=0.0
INCREMENT3=0
# Function to extract the current C++ parameter value
extract_current_value() {
  grep "^const double bezierHeight" "$SOURCE_DIR/ConvexMPCLocomotion.cpp" | awk -F ' = ' '{print $2}' | tr -d ';'
}

extract_current_value_cycle() {
  grep "^const int cycletime" "$SOURCE_DIR/ConvexMPCLocomotion.cpp" | awk -F ' = ' '{print $2}' | tr -d ';'
}

# Function to extract the current YAML parameter value using sed
extract_current_value_yaml() {
  grep "rise:" "$YAML_FILE" | awk '{print $2}'
}

# Function to extract the current run value from the YAML file using sed
extract_current_run_value_yaml() {
  grep "run:" "$YAML_FILE" | awk '{print $2}'
}

# Function to extract the steps parameter value using sed
extract_current_steps_value_yaml() {
  grep "steps:" "$YAML_FILE" | awk '{print $2}'
}

# Function to run the simulation and save parameters
run_simulation() {
  rm -f $SIGNAL_FILE
  iteration=$1
  parameter_value=$2
  yaml_parameter_value=$3
  pitch_ascension=$4
  steps=$5
  run_parameter_value=$6
  cycle_parameter_value=$7
  echo "Iteration $iteration: Updating C++ parameter to $parameter_value"
  echo "Iteration $iteration: Updating YAML parameter to $yaml_parameter_value"
  echo "Iteration $iteration: Calculated pitch_ascension to $pitch_ascension"
  echo "Iteration $iteration: Number of steps is $steps"
  echo "Iteration $iteration: Updating C++ cycle parameter to $cycle_parameter_value"
  # Update the C++ file
  #sed -i "s/^const double bezierHeight .*/const double bezierHeight = $parameter_value;/" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  
  sed -i "s/^const int cycletime .*/const int cycletime = $cycle_parameter_value;/" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  
  # Update pitch_ascension in the C++ header file
  sed -i "s/^\s*float pitch_ascension = .*/  float pitch_ascension = $pitch_ascension;/" "$SOURCE_DIR/ConvexMPCLocomotion.h"
  
  # Update the YAML file using sed
  sed -i "s/^\(.*rise: \).*/\1$yaml_parameter_value/" "$YAML_FILE"
  sed -i "s/^\(.*steps: \).*/\1$steps/" "$YAML_FILE"
  sed -i "s/^\(.*run: \).*/\1$run_parameter_value/" "$YAML_FILE"
  # Read the updated rise parameter
  new_rise=$(grep "rise:" "$YAML_FILE" | awk '{print $2}')
  new_run=$(grep "run:" "$YAML_FILE" | awk '{print $2}')
  
  new_height=$(echo "2 * $steps * $yaml_parameter_value" | bc)
  new_position=$(echo "((($steps * $current_run_value_yaml) + 0.3) * 2 + 1) / 2" | bc -l)
  
  sed -i "/box-22:/,/transparent:/{s/\(height: \).*/\1$new_height/}" "$YAML_FILE"
  sed -i "/box-22:/,/transparent:/{s/\(position: \[\)[^,]*/\1$new_position/}" "$YAML_FILE"
  # Update the delta and steps parameters in the ConvexMPCLocomotion.cpp file
  sed -i "s/^const double delta = .*/const double delta = $new_rise;/" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  sed -i "s/^const double runstairs = .*/const double runstairs = $new_run;/" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  sed -i "s/^const int steps = .*/const int steps = $steps;/" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  
  
  log_filename="$SAVE_DIR/${iteration}_${yaml_parameter_value}_${steps}_${pitch_ascension}.txt"
  echo "New log filename: $log_filename"
  
  # Update the log_filename definition in the C++ file
  sed -i "s|std::string log_filename = .*;|std::string log_filename = \"$log_filename\";|" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  
  # Verify changes
  #echo "Updated C++ parameter:"
  #grep "^const double bezierHeight" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  echo "Updated cycle C++ parameter:"
  grep "^const int cycletime" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  echo "Updated pitch_ascension:"
  grep "pitch_ascension" "$SOURCE_DIR/ConvexMPCLocomotion.h"
  echo "Updated delta parameter:"
  grep "^const double delta" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  echo "Updated runstairs parameter:"
  grep "^const double runstairs" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  echo "Updated steps parameter:"
  grep "^const int steps" "$SOURCE_DIR/ConvexMPCLocomotion.cpp"
  echo "Updated YAML parameter:"
  grep "rise:" "$YAML_FILE"
  echo "Updated box-22 height:"
  grep -A 1 "box-22:" "$YAML_FILE" | grep "height:"
  echo "Updated box-22 position:"
  grep -A 2 "box-22:" "$YAML_FILE" | grep "position:"
  
  # Navigate to the build directory and build the software
  cd $BUILD_DIR
  make -j4
  sleep 15

  # Start the simulation GUI in the background
  ./sim/sim &
  GUI_PID=$!

  # Wait a few seconds to ensure the GUI is fully loaded
  sleep 3

  # Press the start button in the GUI
  xdotool mousemove $START_BUTTON_X $START_BUTTON_Y click 1
  sleep 2

  # Start the controller in a new terminal and capture its PID
  gnome-terminal --title="controller_$iteration" -- bash -c "cd $BUILD_DIR && ./user/MIT_Controller/mit_ctrl m s; exec bash" &
  CTRL_PID=$!
  sleep 10

  # Interact with the GUI to set the value
  xdotool mousemove $TAB_BUTTON_X $TAB_BUTTON_Y click 1
  sleep 1
  xdotool mousemove $BOX_X $BOX_Y click --repeat 2 --delay 100 1
  sleep 1
  xdotool type 4
  xdotool key Return

  timeout 100 bash -c "while [ ! -f $SIGNAL_FILE ]; do sleep 1; done"
  result=$?
  rm -f $SIGNAL_FILE

  if [ $result -eq 124 ]; then
    echo "Timeout reached for iteration $iteration. Killing GUI and controller."
    kill $GUI_PID
    kill $CTRL_PID
    wmctrl -l | grep "controller_$iteration" | awk '{print $1}' | xargs -I {} wmctrl -ic {}
    rm -f "$log_filename"
    return 1
  fi
  rm $SIGNAL_FILE

  # Save necessary parameters (customize based on your requirements)
  cp "$BUILD_DIR/parameters_file" "$SAVE_DIR/parameters_$iteration.txt"

  # Kill the GUI and controller processes
  kill $GUI_PID
  kill $CTRL_PID

  # Close the terminal running the controller
  # Ensure all controller terminals are closed
  wmctrl -l | grep "controller_$iteration" | awk '{print $1}' | xargs -I {} wmctrl -ic {}
  sleep 5  # Wait a bit to ensure the process is terminated
}

# Main loop to run simulations
for ((i=1; i<=NUM_ITERATIONS; i++))
do
  echo "Running iteration $i..."

  # Extract current values
  current_value=$(extract_current_value)
  current_value_yaml=$(extract_current_value_yaml)
  current_run_value_yaml=$(extract_current_run_value_yaml)
  current_steps_value_yaml=$(extract_current_steps_value_yaml)
  current_cycle_value=$(extract_current_value_cycle)
  
  echo "Current C++ parameter value: $current_value"
  echo "Current YAML parameter value: $current_value_yaml"
  echo "Current run value from YAML: $current_run_value_yaml"
  echo "Current steps value from YAML: $current_steps_value_yaml"
  echo "Current cycle C++ parameter value: $current_cycle_value"
  
  # Calculate new values
  parameter_value=$(echo "$current_value + $INCREMENT" | bc)
  yaml_parameter_value=$(echo "$current_value_yaml + $INCREMENT1" | bc)
  run_parameter_value=$(echo "$current_run_value_yaml + $INCREMENT2" | bc)
  cycle_parameter_value=$(echo "$current_cycle_value + $INCREMENT3" | bc)
  # Calculate pitch_ascension in radians
  pitch_ascension=$(echo "scale=4; -a(($yaml_parameter_value / $current_run_value_yaml)+ 0.06)" | bc -l)
  
  echo "New C++ parameter value: $parameter_value"
  echo "New YAML parameter value: $yaml_parameter_value"
  echo "Calculated pitch_ascension (radians): $pitch_ascension"
  echo "New run parameter value: $run_parameter_value"
  echo "New cycle C++ parameter value: $cycle_parameter_value"
  # Run the simulation with updated values
  run_simulation $i $parameter_value $yaml_parameter_value $pitch_ascension $current_steps_value_yaml $run_parameter_value $cycle_parameter_value 
  if [ $? -ne 0 ]; then
    echo "Skipping iteration $i due to timeout or error"
    continue
  fi
  sleep 3
done

echo "All iterations completed."
