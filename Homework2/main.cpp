#include <bits/stdc++.h>
#include "Assignment.h"

using namespace std;

const string file = "instance1.txt";

Assignment getInstanceFromFile() {
    ifstream fin(file);
    // read the grid first
    TableType<int> table;
    TableType<bool> isGrey;

    for (int i = 1; i <= 9; i++)
        for (int j = 1; j <= 9; j++)
            fin >> table[i][j];
    for (int i = 1; i <= 9; i++)
        for (int j = 1; j <= 9; j++)
            fin >> isGrey[i][j];

    fin.close();
    return {table, isGrey};
}

optional<Assignment> bkt(Assignment &assignment) {
    if (assignment.isComplete())
        return assignment;

    auto var = assignment.nextUnassignedVariable();
    for (auto value: assignment.getDomainOfVariable(var))
        if (assignment.isConsistent(var, value)) {
            assignment.addValue(var, value);
            auto res = bkt(assignment);
            if (res.has_value())
                return res;
            assignment.rollback();
        }

    return std::nullopt;
}

void printSolution(Assignment assignment) {
    auto res = bkt(assignment);
    if (!res.has_value())
        cout << "No solution found\n";
    else
        cout << res.value();
}


int main() {
    auto instance = getInstanceFromFile();
    printSolution(instance);


    return 0;
}
