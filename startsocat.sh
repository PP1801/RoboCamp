#!/bin/bash

# Function to handle cleanup on script termination
cleanup() {
    echo "Removing links..."
    sudo rm /dev/ttyROBO0
    sudo rm /dev/ttyROBO1

    kill $SOCAT_PID #
    exit
}

# Trap the INT signal (Ctrl+C) and call the cleanup function
trap cleanup INT

# Start socat and get the pseudo-terminal devices
echo "Starting socat..."
#output=$(socat -d -d pty,raw,echo=0 pty,raw,echo=0 2>&1) # | tee /tmp/socat_output.log)
socat -d -d pty,raw,echo=0 pty,raw,echo=0 2>&1 | tee /tmp/socat_output.log & SOCAT_PID=$!

# Wait a moment to ensure socat has started and output is available
sleep 1

# Extract the pty devices from the socat output
#pts1=$(echo "$output" | grep -oP 'PTY is \K/dev/pts/\d+' | head -1)
#pts2=$(echo "$output" | grep -oP 'PTY is \K/dev/pts/\d+' | tail -1)
pts1=$(grep -oP 'PTY is \K/dev/pts/\d+' /tmp/socat_output.log | head -1)
pts2=$(grep -oP 'PTY is \K/dev/pts/\d+' /tmp/socat_output.log | tail -1)

# Configuring pty devices
echo "Configuring $pts1..."
sudo stty -F "$pts1" 115200

echo "Configuring $pts2..."
sudo stty -F "$pts2" 115200

# Creating a symbolic link
echo "Crating symbolic links..."
sudo ln -s "$pts1" /dev/ttyROBO0
sudo ln -s "$pts2" /dev/ttyROBO1

echo "Write input $pts1 is on /dev/ttyROBO0"
echo "Read output $pts2 is on /deb/ttyROBO1"

echo "Ready!"
# Wait for socat to terminate (Ctrl+C)
#wait
wait $SOCAT_PID