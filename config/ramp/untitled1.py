#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 04:15:55 2024

@author: aminghanbarzadeh
"""

with open("Sobhan_ramp.txt", "r") as file:
    # Read all the numbers from the file
    numbers = file.read().split()

    # Create a new list to store the numbers with spaces and newlines added
    numbers_with_spaces_and_newlines = []

    # Iterate through the numbers and add six spaces after each number
    for index, number in enumerate(numbers):
        numbers_with_spaces_and_newlines.append(number)
        numbers_with_spaces_and_newlines.append(" " * 7)

        # Check if it's the 100th number and insert a newline
        if (index + 1) % 100 == 0:
            numbers_with_spaces_and_newlines.append("\n")

# Write the numbers with added spaces and newlines to a new text file
with open("numbers_with_spaces_and_newlines.txt", "w") as file:
    # Join the numbers list with added spaces and newlines and write it to the new file
    file.write("".join(numbers_with_spaces_and_newlines))