#pragma once

#include<bits/stdc++.h>

using namespace std;

template<typename T>
using TableType = array<array<T, 10>, 10>;

struct Assignment {
    // we'll only use 1..9
    TableType<int> table;
    TableType<bool> isGrey;
    stack<pair<int, int>> s; // the cells that we're filled

    int cntEmpty;

    Assignment(TableType<int> table, TableType<bool> isGrey) {
        this->table = table;
        this->isGrey = isGrey;
        cntEmpty = 0;
        for (int i = 1; i <= 9; i++)
            for (int j = 1; j <= 9; j++)
                if (table[i][j] == 0)
                    cntEmpty++;
    }

    [[nodiscard]] bool isComplete() const {
        return cntEmpty == 0;
    }


    [[nodiscard]] pair<int, int> nextUnassignedVariable() const {
        // TODO optimize
        for (int i = 1; i <= 9; i++)
            for (int j = 1; j <= 9; j++)
                if (table[i][j] == 0)
                    return {i, j};
        throw exception();
    }

    [[nodiscard]] vector<int> getDomainOfVariable(const pair<int, int> &var) const {
        assert(table[var.first][var.second] == 0);
        if (isGrey[var.first][var.second])
            return {2, 4, 6, 8};
        return {1, 2, 3, 4, 5, 6, 7, 8, 9};
    }

    [[nodiscard]] bool isConsistent(const pair<int, int> &var, int value) const {
        // check domain
        auto dom = getDomainOfVariable(var);
        if (find(dom.begin(), dom.end(), value) == dom.end())
            return false;

        // check if it is different on lines and columns
        for (int i = 1; i <= 9; i++) {
            if (i != var.first && table[i][var.second] == value)
                return false;
            if (i != var.second && table[var.first][i] == value)
                return false;
        }

        // check if it is different in the 3x3 square
        int squareRow = 3 * ((var.first - 1) / 3);
        int squareCol = 3 * ((var.second - 1) / 3);

        for (int i = squareRow + 1; i <= squareRow + 3; i++)
            for (int j = squareCol + 1; j <= squareCol + 3; j++)
                if (i != var.first && j != var.second && table[i][j] == value)
                    return false;

        return true;
    }

    void addValue(const pair<int, int> &p, int value) {
        table[p.first][p.second] = value;
        cntEmpty--;
        s.push(p);
    }

    void rollback() {
        table[s.top().first][s.top().second] = 0;
        cntEmpty++;
        s.pop();
    }

    friend std::ostream &operator<<(std::ostream &os, const Assignment &assignment) {
        for (int i = 1; i <= 9; i++) {
            for (int j = 1; j <= 9; j++) {
                os << assignment.table[i][j] << ' ';
            }
            os << '\n';
        }
        return os;
    }
};