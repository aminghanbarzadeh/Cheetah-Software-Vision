#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 23 07:33:47 2024

@author: aminghanbarzadeh
"""

# Open the text file in read mode
with open("numbers_with_spaces.txt", "r") as file:
    # Read all the numbers from the file
    numbers = file.read().split()

    # Create a new list to store the numbers with newlines
    numbers_with_newlines = []

    # Iterate through the numbers and insert a newline after every 100 numbers
    for index, number in enumerate(numbers):
        numbers_with_newlines.append(number)
        if (index + 1) % 200 == 0:
            numbers_with_newlines.append("\n")

# Write the numbers with newlines to a new text file
with open("numbers_with_newlines.txt", "w") as file:
    file.write(" ".join(numbers_with_newlines))
