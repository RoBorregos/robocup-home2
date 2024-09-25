#!/bin/bash

# Creates a custom SSH configuration file and adds an 'Include' line to the main SSH config file.
# This file provides the most up-to-date config for user - ip pairs.

# Define the file paths
SSH_CONFIG="$HOME/.ssh/config"
RBRGS_HOME="$HOME/.ssh/rbrgs_home"
INCLUDE_LINE="Include $RBRGS_HOME"

# Overwrite ~/.ssh/rbrgs_home with current robot ip addresses
cat <<EOL > "$RBRGS_HOME"
Host xavier 192.168.31.105
  HostName 192.168.31.105
  User nvidia

Host jetson 192.168.31.106
  HostName 192.168.31.106
  User jetson
EOL

echo "Overwritten $RBRGS_HOME with updated hosts."

# Check if ~/.ssh directory exists, if not, create it
if [ ! -d "$HOME/.ssh" ]; then
    mkdir -p "$HOME/.ssh"
    echo "Created ~/.ssh directory."
fi

# Check if ~/.ssh/config file exists, if not, create it
if [ ! -d "$HOME/.ssh" ]; then
    touch "$SSH_CONFIG"
    echo "Created the SSH config file."
fi

add_include_if_missing() {
  local include_line="$1"
  
  # Check for the first Host line
  if ! sed '/^Host/ q' "$SSH_CONFIG" | grep -q "^$include_line"; then
    # Prepend the Include directive if it's not found before the first Host
    { echo -e "$include_line\n"; cat "$SSH_CONFIG"; } > "$SSH_CONFIG.tmp" && mv "$SSH_CONFIG.tmp" "$SSH_CONFIG"
    echo "Added $include_line at the top of the SSH config file."
  else
    echo "The Include directive $include_line is already present."
  fi
}

add_include_if_missing "$INCLUDE_LINE"

chmod 700 "$HOME/.ssh"
chmod 600 "$SSH_CONFIG"



