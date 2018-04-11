#ifndef LINEAR_INTERPOLATION_H
#define LINEAR_INTERPOLATION_H

#include <vector>
#include <map>
#include <assert.h>

class LinearInterpolation
{
  public:
    LinearInterpolation(const std::vector<double> &xList, const std::vector<double> &yList)
    {
        assert(xList.size() == yList.size());

        for (size_t i = 0; i < xList.size(); i++)
        {
            xyMap.insert(std::make_pair(xList[i], yList[i]));
        }
    }

    double Y(double x, double &k) const
    {
        double y;
        auto upperPair = xyMap.upper_bound(x);
        auto lowerPair = prev(upperPair);
        if (upperPair == xyMap.end())
        {
            y = lowerPair->second;
            k = 0;
        }
        else
        {
            auto &x0 = lowerPair->first, &x1 = upperPair->first;
            auto &y0 = lowerPair->second, &y1 = upperPair->second;
            k = (y1 - y0) / (x1 - x0);
            y = y0 + (x - x0) * k;
        }

        return y;
    }

    double Y(double x) const
    {
        double k;
        return Y(x, k);
    }

  private:
    std::map<double, double> xyMap;
};

#endif
