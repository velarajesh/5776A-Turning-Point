#include "motion/path/bezier.hpp"
#include <vector>
#include <cmath>

namespace path
{
Bezier::Bezier(std::vector<Point> points, int resolution, int lookahead) : Path::Path(resolution, lookahead),
                                                                           points(points) {}

std::vector<int> getPascalCoeff(int rowIndex)
{
    std::vector<int> row;
    row.push_back(1);
    if (rowIndex == 0)
        return row;
    row.push_back(1);
    if (rowIndex == 1)
        return row;

    std::vector<int> result;
    for (int j = 2; j <= rowIndex; j++)
    {
        result.clear();
        result.push_back(1);
        for (int i = 1; i <= j - 1; i++)
        {
            result.push_back(row[i - 1] + row[i]);
        }
        result.push_back(1);
        row = result;
    }
    return row;
}

int factorial(int n)
{
    if (n == 0 || n == 1)
    {
        return 1;
    }
    else
    {
        return factorial(n - 1) * n;
    }
}

double Bezier::combination(int n, int r)
{
    int nFactorial = factorial(n);
    int rFactorial = factorial(r);
    int diffFactorial = factorial(n - r);
    return (double)nFactorial / ((double)rFactorial * ((double)diffFactorial));
}

/**
 * x = (1-t)((1-t)((1-t) * x0 + tx1) + t((1-t)x1 + tx2)) + t((1-t)((1-t)x1 + tx2) + t((1-t)((1-t)x2 + tx3)))
 */

Point Bezier::pointAt(int T)
{
    Point point = {0 * okapi::inch, 0 * okapi::inch};

    int n = points.size() - 1;

    double sumX = 0;
    double sumY = 0;

    int lilT = (T > resolution) ? resolution : T;
    if (lilT < 0)
        lilT = 0;
    double sT = (double)lilT / (double)resolution; // scaled t

    for (int i = 0; i <= n; i++)
    {
        double currX = points[i].x.convert(okapi::inch);
        double currY = points[i].y.convert(okapi::inch);

        sumX += combination(n, i) * std::pow(1 - sT, n - i) * std::pow(sT, i) * currX;
        sumY += combination(n, i) * std::pow(1 - sT, n - i) * std::pow(sT, i) * currY;
    }

    point.x = sumX * okapi::inch;
    point.y = sumY * okapi::inch;
    point.t = lilT;

    return point;
}
} // namespace path