# script to go through a csv file and remove any ,, replacing it with a single ,
# usage: python dataclean.py <file.csv>
# example: python dataclean.py data.csv

import sys


def clean_data(file):
    with open(file, "r") as f:
        lines = f.readlines()

        # check the number of entries from the header
        header = lines[0].split(",")
        num_entries = len(header)
        numOmits = 0
    with open(file, "w") as f:
        for line in lines:
            # Strip the newline character from the end of the line
            line = line.rstrip("\n")
            # strip whitespace
            line = line.strip()
            # Replace any ,, with ,
            line = line.replace(",,", ",")
            # write if the number of entries is correct
            # remove spaces from line
            line = line.strip()

            if len(line.split(",")) == num_entries:
                f.write(line + "\n")
            else:
                print("Error: Incorrect number of entries in the line: " + line)
                print(
                    "Expected: "
                    + str(num_entries)
                    + " Got: "
                    + str(len(line.split(",")))
                )
                numOmits += 1

    print("Data cleaned successfully, " + str(numOmits) + " lines omitted.")


if __name__ == "__main__":
    clean_data(sys.argv[1])
