#!/bin/bash

clear

# Save cursor position
tput sc

# Hide cursor
tput civis

function cleanup {
	tput cnorm
}

# Display cursor on exit
trap cleanup EXIT

while :; do
	# Reset cursor position
	tput rc

	# Display a sorted list of Docker service outputs
	docker-compose logs --tail=1 | grep -oP '(?<=benchmark_)[0-9][0-9]* .*' | sort -nk1

	sleep ${1:-1}
done
