#ifndef RANGEDDATA_H
#define RANGEDDATA_H

//  Represents data in which the variable has a range
//  and a current value.

class RangedData
{
public:
    double min;
    double max;
    double curr;

    RangedData()
        : min(0), max(0), curr(0)
    {}

    RangedData(double min, double max)
        : min(min), max(max), curr(min)
    {}
};

#endif // RANGEDDATA_H
