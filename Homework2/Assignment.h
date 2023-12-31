#pragma once

#include<bits/stdc++.h>

using namespace std;

template<typename T>
using TableType = array<array<T, 10>, 10>;

struct Assignment {
    // we'll only use 1..9
    TableType<int> table{};
    TableType<bool> isGrey{};
    stack<pair<int, int>> s; // the cells that we're filled
    TableType<set<int>> domains;

    vector<pair<int, int>> emptyCells;

    Assignment(TableType<int> table, TableType<bool> isGrey) {
        this->isGrey = isGrey;
        this->table = table;

        for (int i = 1; i <= 9; i++)
            for (int j = 1; j <= 9; j++)
                if (table[i][j] == 0) {
                    emptyCells.emplace_back(i, j);
                    domains[i][j] = getFreeValues({i, j});
                }
    }

    [[nodiscard]] bool isComplete() const {
        return emptyCells.empty();
    }

    [[nodiscard]] pair<int, int> nextUnassignedVariable() {
        ranges::sort(emptyCells, [dom = this->domains](const pair<int, int> &x, const pair<int, int> &y) {
            return dom[x.first][x.second].size() < dom[y.first][y.second].size();
        });
        return emptyCells[0];
    }


    [[nodiscard]] set<int> getDomain(const pair<int, int> &var) const {
        return domains[var.first][var.second];
    }

    [[nodiscard]] bool shouldContinue() {
        auto x = nextUnassignedVariable();
        return !getDomain(x).empty();
    }

    [[nodiscard]] set<int> getDefaultDomain(const pair<int, int> &var) const {
        if (isGrey[var.first][var.second])
            return {2, 4, 6, 8};
        return {1, 2, 3, 4, 5, 6, 7, 8, 9};
    }

    [[nodiscard]] set<int> getFreeValuesOnLinesAndColumns(const pair<int, int> &var) const {
        array<bool, 10> isTaken{};
        for (int i = 1; i <= 9; i++) {
            isTaken[table[i][var.second]] = true;
            isTaken[table[var.first][i]] = true;
        }
        set<int> v;
        for (int i = 1; i <= 9; i++)
            if (!isTaken[i])
                v.insert(i);
        return v;
    }

    [[nodiscard]] set<int> getFreeValuesInSquare(const pair<int, int> &var) const {
        array<bool, 10> isTaken{};
        auto sq = getSquareFromCell(var);

        for (int i = sq.first + 1; i <= sq.first + 3; i++)
            for (int j = sq.second + 1; j <= sq.second + 3; j++)
                isTaken[table[i][j]] = true;

        set<int> v;
        for (int i = 1; i <= 9; i++)
            if (!isTaken[i])
                v.insert(i);
        return v;
    }

    [[nodiscard]] set<int> getFreeValues(const pair<int, int> &var) const {
        auto v1 = getFreeValuesOnLinesAndColumns(var);
        auto v2 = getFreeValuesInSquare(var);
        auto v3 = getDefaultDomain(var);
        set<int> st;
        for (auto it: v1)
            if (v2.contains(it) && v3.contains(it))
                st.insert(it);
        return st;
    }

    [[nodiscard]] bool isConsistent(const pair<int, int> &var, int value) const {
        auto domain = getFreeValues(var);
        return ranges::find(domain, value) != domain.end();
    }

    [[nodiscard]] static pair<int, int> getSquareFromCell(const pair<int, int> &p) {
        int squareRow = 3 * ((p.first - 1) / 3);
        int squareCol = 3 * ((p.second - 1) / 3);
        return {squareRow, squareCol};
    }

    void recalculateDomainForCell(const pair<int, int> &p) {
        auto [i, j] = p;
        if (!table[i][j])
            domains[i][j] = getFreeValues(p);
    }

    void updateConcernedDomains(const pair<int, int> &p) {
        for (int i = 1; i <= 9; i++) {
            recalculateDomainForCell({i, p.second});
            recalculateDomainForCell({p.first, i});
        }

        auto sq = getSquareFromCell(p);
        for (int i = sq.first + 1; i <= sq.first + 3; i++)
            for (int j = sq.second + 1; j <= sq.second + 3; j++)
                recalculateDomainForCell({i, j});
    }

    void addValue(const pair<int, int> &p, int value) {
        table[p.first][p.second] = value;
        emptyCells.erase(ranges::find(emptyCells, p));
        s.push(p);
        updateConcernedDomains(p);
    }

    void rollback() {
        table[s.top().first][s.top().second] = 0;
        emptyCells.push_back(s.top());
        updateConcernedDomains(s.top());
        s.pop();
    }

    static bool areLinked(const pair<int, int> &a, const pair<int, int> &b) {
        return (a.first == b.first || a.second == b.second || getSquareFromCell(a) == getSquareFromCell(b));
    }

    void forceArcConsistency() {
        queue<pair<pair<int, int>, pair<int, int>>> q;

        //using only empty cells
        for (auto it1: emptyCells)
            for (auto it2: emptyCells)
                if (it1 != it2 && areLinked(it1, it2))
                    q.emplace(it1, it2);

        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();

            bool ok = true;
            for (auto itr = domains[x.first][x.second].cbegin(); itr != domains[x.first][x.second].cend();)
                if (domains[y.first][y.second].size() == 1 && domains[y.first][y.second].contains(*itr)) {
                    ok = false;
                    itr = domains[x.first][x.second].erase(itr);
                } else
                    ++itr;

            if (!ok) {
                for (auto z: emptyCells)
                    if (z != x && areLinked(z, x))
                        q.emplace(z, x);
            }
        }
    }

    friend ostream &operator<<(ostream &os, const Assignment &assignment) {
        for (int i = 1; i <= 9; i++) {
            for (int j = 1; j <= 9; j++)
                os << assignment.table[i][j] << ' ';
            os << '\n';
        }
        return os;
    }
};